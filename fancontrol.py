#!/usr/bin/env python3

# Copyright 2023 Dan Smith <dellfancontrol@f.danplanet.com>
# Released under the GPLv3

import argparse
import atexit
import logging
import os
import signal
import socket
import subprocess
import sys
import time
import yaml

from simple_pid import PID

LOG = logging.getLogger('controller')
HOSTNAME = socket.gethostname()


def report_values(host, port, values):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))
    for key, value in values.items():
        s.send(('%s %s %i\n' % (key, value, time.time())).encode())
    s.close()


class Controller:
    def __init__(self, config):
        self.conf_file = config
        self.load_config()
        self.run = True
        atexit.register(self.fanmode_default)
        signal.signal(signal.SIGTERM, self.signal)
        self.current_target = object()
        self.sensors = {}

    def temp(self, tempC):
        if self.config.get('units', 'c').lower() == 'f':
            return '%iF' % (tempC * 9 / 5) + 32
        else:
            return '%.1fC' % tempC

    def load_config(self):
        conf_file = open(self.conf_file, 'r')
        self.config = yaml.load(conf_file, Loader=yaml.CLoader)
        self.min_fan = self.config.get('min_fan', 10)
        panic_defaults = {
            'Exhaust Temp': 40,
            'Temp': 60,
        }
        self.panic_temps = self.config.get('panic_temps', panic_defaults)
        self.monitor_temp = self.config.get('sensor', 'Exhaust Temp')
        self.poll_time = self.config.get('poll_time', 10)
        self.config_ts = os.lstat(self.conf_file).st_mtime

    def check_config(self):
        current = os.lstat(self.conf_file).st_mtime
        if current > self.config_ts:
            LOG.info('Reloading changed config')
            self.load_config()

    def signal(self, signum, frame):
        LOG.warning('Signal received, stopping')
        self.run = False

    def run_ipmitool(self, *args):
        # LOG.debug('Running ipmitool %s' % ' '.join(str(a) for a in args))
        out = subprocess.check_output(['/usr/bin/ipmitool'] + list(args))
        return out

    def fanmode_default(self):
        # System-controlled
        if self.current_target is not None:
            LOG.info('Changing fan target to default')
            self.run_ipmitool('raw', '0x30', '0x30', '0x01', '0x01')
            self.current_target = None

    def fanmode_set(self, pct):
        # Manual set to a speed
        pct = min(100, pct)
        if self.current_target != pct:
            LOG.info('Changing fan target to %i%%' % pct)
            self.run_ipmitool('raw', '0x30', '0x30', '0x01', '0x00')
            self.run_ipmitool('raw', '0x30', '0x30', '0x02',
                              '0xff',  # 0xFF means "all", otherwise fan index
                              '0x%02x' % pct)
            self.current_target = pct

    def get_sensors(self):
        o = self.run_ipmitool('sensor').decode()
        lines = [line.strip() for line in o.split('\n') if line.strip()]
        last_sensors = self.sensors
        self.sensors = {}
        for line in lines:
            fields = [x.strip() for x in line.split('|')]
            val = fields[1]
            if val.startswith('0x'):
                val = int(val, 16)
            else:
                try:
                    val = float(fields[1])
                except ValueError:
                    pass
            if fields[0] in self.sensors:
                # Some sensors (like CPU "Temp") are not unique,
                # so take the higher of the values
                try:
                    self.sensors[fields[0]] = max(self.sensors[fields[0]], val)
                except TypeError:
                    # Multi-value sensor with differing types, just ignore
                    pass
            else:
                self.sensors[fields[0]] = val

        log_sensors = []
        for key in self.sensors:
            if key not in self.config.get('log_sensors', []):
                continue
            if self.sensors[key] != last_sensors.get(key):
                log_sensors.append('%s=%s' % (key, self.sensors[key]))
        if log_sensors:
            LOG.info('Other sensors: %s', ','.join(log_sensors))

    def monitor_loop(self):
        while self.run:
            self.get_sensors()

            for temp, limit in self.panic_temps.items():
                if self.sensors[temp] >= limit:
                    LOG.warning(
                        'Temp %s %s over panic temp %s; going to default',
                        temp, self.temp(self.sensors[temp]), self.temp(limit))
                    self.fanmode_default()
                    time.sleep(self.poll_time)
                    continue

            temp = self.sensors[self.monitor_temp]
            target = self.target_logic(temp)
            if target is None:
                self.fanmode_default()
            else:
                self.fanmode_set(target)

            if self.config.get('graphite'):
                if self.config['graphite'].get('prefix_hostname') == 'short':
                    prefix = HOSTNAME.split('.')[0]
                else:
                    prefix = HOSTNAME
                report_values(self.config['graphite']['host'],
                              self.config['graphite'].get('port', 2003),
                              {'%s.fancontrol.temp' % prefix: temp,
                               '%s.fancontrol.target' % prefix: target})

            time.sleep(self.poll_time)
            self.check_config()

        self.fanmode_default()


class SimpleController(Controller):
    def __init__(self, config):
        super().__init__(config)
        self.warn_temp = self.config['simple']['warn_temp']

    def target_logic(self, temp):
        LOG.debug('Temp %r is %i' % (self.monitor_temp, self.temp(temp)))
        if temp < self.warn_temp:
            target = self.min_fan
        elif temp < self.panic_temp:
            target = 40
        else:
            target = None
        return target


class PIDController(Controller):
    def __init__(self, config):
        self.pid = PID()
        super().__init__(config)

        self._last_temp = None
        self._last_target = None

    def load_config(self):
        super().load_config()
        self.pid_config = self.config.get('pid', {})

        self.pid.Kp = self.pid_config.get('kp', 2) * -1
        self.pid.Ki = self.pid_config.get('ki', 0.02) * -1
        self.pid.Kd = self.pid_config.get('kd', 0.0) * -1
        self.pid.setpoint = self.pid_config.get('target_temp', 32)
        self.pid.sample_time = self.pid_config.get('sample_time', 30)
        self.pid.proportional_on_measurement = bool(
            self.pid_config.get('pom', False))
        self.pid.derivative_on_measurement = bool(
            self.pid_config.get('dom', False))
        self.pid.output_limits = (0, None)
        LOG.info('PID params %.2f,%.2f,%.2f '
                 '(set %s, sample %i, pom %s, dom %s)',
                 self.pid.Kp, self.pid.Ki, self.pid.Kd,
                 self.temp(self.pid.setpoint),
                 self.pid.sample_time,
                 self.pid.proportional_on_measurement,
                 self.pid.derivative_on_measurement)

    def target_logic(self, temp):
        output = self.pid(temp)

        if output < 0:
            # If PID wants warmer, we're in good shape, so we can be at minimum
            target = self.min_fan
        else:
            target = self.min_fan + (output * self.pid_config.get(
                'fan_scale', 1))

        if target != self._last_target or temp != self._last_temp:
            LOG.debug('%s is %s Target %i%% want=%.1f',
                      self.monitor_temp, self.temp(temp), target, output)

        self._last_target = target
        self._last_temp = temp

        return int(target)


def main():
    conf = os.path.expanduser('~/.config/fancontrol.yaml')
    p = argparse.ArgumentParser()
    p.add_argument('--config', default=conf, required=False)
    p.add_argument('--debug', default=False, action='store_true')
    args = p.parse_args()
    logging.basicConfig(level=args.debug and logging.DEBUG or logging.INFO)
    c = PIDController(args.config)
    c.monitor_loop()


if __name__ == '__main__':
    sys.exit(main())
