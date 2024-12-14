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

import pySMART
from simple_pid import PID

LOG = logging.getLogger('controller')
HOSTNAME = socket.gethostname()


def report_values(host, port, values):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((host, port))
        for key, value in values.items():
            key = key.replace(' ', '_')
            key = key.replace('/', '_')
            s.send(('%s %s %i\n' % (key, value, time.time())).encode())
        s.close()
    except:
        ...


def run_ipmitool(*args):
    # LOG.debug('Running ipmitool %s' % ' '.join(str(a) for a in args))
    for retry in range(3):
        try:
            return subprocess.check_output(
                ['/usr/bin/ipmitool'] + list(args))
        except Exception as e:
            LOG.exception('Failed to run ipmitool %s: %s; retrying',
                          str(args), e)
            time.sleep(5)
    LOG.error('PANIC: Unable to run ipmitool!')


class Panic(Exception):
    pass


class IgnoreSensor(Exception):
    pass


class Sensor:
    def __init__(self, name, target, panic, sample_time=0):
        self.name = name
        self.target = target
        self.panic = panic
        self._current = target
        self.sample_time = sample_time
        self.last_sample = 0

    def get_sample(self):
        pass

    def sample(self):
        if time.monotonic() - self.last_sample >= self.sample_time:
            self._current = self.get_sample()
            self.last_sample = time.monotonic()
            if self.panic is not None and self.current >= self.panic:
                raise Panic('Sensor panic: %s is %i > %i' % (
                    self.name, self.current, self.panic))

    @property
    def current(self):
        return self._current

    @property
    def delta(self):
        if self.target is None:
            return 0
        return self._current - self.target


class CompositeSensor(Sensor):
    def __init__(self, sensors):
        self.sensors = sensors

    def sample(self):
        for i in self.sensors:
            i.sample()

    @property
    def delta(self):
        return max([s.delta for s in self.sensors])


class IPMISensor(Sensor):
    # Cache of sensor values from the last refresh
    sensors = {}
    # Timestamp of last sensor refresh from IPMI
    _last_get = 0

    @classmethod
    def get_sensors(cls):
        if time.monotonic() - cls._last_get < 10:
            return

        o = run_ipmitool('sensor').decode()
        lines = [line.strip() for line in o.split('\n') if line.strip()]
        cls.sensors = {}
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
            if fields[0] in cls.sensors:
                # Some sensors (like CPU "Temp") are not unique,
                # so take the higher of the values
                try:
                    cls.sensors[fields[0]] = max(cls.sensors[fields[0]], val)
                except TypeError:
                    # Multi-value sensor with differing types, just ignore
                    pass
            else:
                cls.sensors[fields[0]] = val
        cls._last_get = time.monotonic()

    def get_sample(self):
        self.get_sensors()
        try:
            return int(self.sensors[self.name])
        except (TypeError, ValueError):
            raise Panic('Invalid sensor value for %r received from IPMI!' % (
                self.name))


class SMARTSensor(Sensor):
    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        if not pySMART.Device(self.name).interface:
            raise IgnoreSensor('SMART device %s not accessible!' % self.name)

    def get_sample(self):
        return pySMART.Device(self.name).if_attributes.temperature


class Controller:
    def __init__(self, config):
        self.conf_file = config
        self.load_config()
        self.run = True
        self.current_target = object()
        self.sensors = {}
        self.errors = 0

        # Attempt to always revert to default on exit
        atexit.register(self.fanmode_default)
        signal.signal(signal.SIGTERM, self.signal)

    def temp(self, tempC):
        if self.config.get('units', 'c').lower() == 'f':
            return '%iF' % (tempC * 9 / 5) + 32
        else:
            return '%.1fC' % tempC

    def load_config(self):
        conf_file = open(self.conf_file, 'r')
        self.config = yaml.load(conf_file, Loader=yaml.CLoader)
        self.min_fan = self.config.get('min_fan', 10)

        self.monitor_sensors = []
        for data in self.config['sensors']:
            try:
                if data['type'] == 'ipmi':
                    s = IPMISensor(data['name'], data.get('target'),
                                   data.get('panic'))
                elif data['type'] == 'smart':
                    s = SMARTSensor(data['name'], data.get('target'),
                                    data.get('panic'), sample_time=60)
                else:
                    raise IgnoreSensor('Unknown sensor type %r' % data['type'])
            except IgnoreSensor as e:
                LOG.warning(e)
                continue
            self.monitor_sensors.append(s)

        self.monitor_temp = CompositeSensor(self.monitor_sensors)
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

    def fanmode_default(self):
        # System-controlled
        if self.current_target is not None:
            LOG.info('Changing fan target to default')
            run_ipmitool('raw', '0x30', '0x30', '0x01', '0x01')
            self.current_target = None

    def fanmode_set(self, pct):
        # Manual set to a speed
        pct = min(100, pct)
        if self.current_target != pct:
            LOG.info('Changing fan target to %i%%' % pct)
            run_ipmitool('raw', '0x30', '0x30', '0x01', '0x00')
            run_ipmitool('raw', '0x30', '0x30', '0x02',
                         '0xff',  # 0xFF means "all", otherwise fan index
                         '0x%02x' % pct)
            self.current_target = pct

    def get_sensors(self):
        last = {s.name: s.current for s in self.monitor_temp.sensors}

        self.monitor_temp.sample()

        vals = {}
        for s in self.monitor_temp.sensors:
            # Only log extra sensors when they get close to the limit
            if (s.current != last[s.name] and
                    s.delta >= -2 and
                    s.target is not None):
                vals[s.name] = '%i(%+i)' % (s.current, s.delta)
            elif (s.current != last[s.name] and s.panic is not None and
                  s.current / s.panic > 0.90):
                vals[s.name] = '%i(!%i)' % (s.current, s.panic)

        if vals:
            LOG.debug('Sensors: %s', ','.join(['%s=%s' % (k, v)
                                               for k, v in vals.items()]))

    def calculate_target(self):
        target = self.target_logic()
        if target is None:
            self.fanmode_default()
        else:
            self.fanmode_set(target)

        if self.config.get('graphite'):
            if self.config['graphite'].get('prefix_hostname') == 'short':
                prefix = HOSTNAME.split('.')[0]
            else:
                prefix = HOSTNAME

            vals = {'target': target,
                    'delta': self.monitor_temp.delta}
            for s in self.monitor_temp.sensors:
                vals['sensor_%s' % s.name] = s.current

            report_values(self.config['graphite']['host'],
                          self.config['graphite'].get('port', 2003),
                          {'%s.fancontrol.%s' % (prefix, k): v
                           for k, v in vals.items()})

        if self.errors:
            LOG.info('Recovered from %i sensor errors', self.errors)
        self.errors = 0

    def default_on_error(self):
        if self.errors >= 3:
            LOG.error('%i sensor errors - going to default', self.errors)
            self.fanmode_default()
        else:
            LOG.warning('%i sensor errors, retrying before default',
                        self.errors)
            self.errors += 1

    def monitor_loop(self):
        while self.run:
            self.check_config()
            try:
                self.get_sensors()
            except Panic as e:
                LOG.warning(e)
                self.default_on_error()
            except Exception as e:
                LOG.exception('Unknown failure: %s' % e)
                self.default_on_error()
            else:
                self.calculate_target()
            time.sleep(self.poll_time)

        # If we exit the control loop for any reason, back to default.
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
        self.pid = PID(setpoint=0)
        super().__init__(config)

        self._last_delta = None
        self._last_target = None

    def load_config(self):
        super().load_config()
        self.pid_config = self.config.get('pid', {})

        self.pid.Kp = self.pid_config.get('kp', 2) * -1
        self.pid.Ki = self.pid_config.get('ki', 0.02) * -1
        self.pid.Kd = self.pid_config.get('kd', 0.0) * -1
        self.pid.sample_time = self.pid_config.get('sample_time', 30)
        self.pid.proportional_on_measurement = bool(
            self.pid_config.get('pom', False))
        self.pid.derivative_on_measurement = bool(
            self.pid_config.get('dom', False))
        self.pid.output_limits = (0, None)
        LOG.info('PID params %.2f,%.2f,%.2f '
                 '(sample %i, pom %s, dom %s)',
                 self.pid.Kp, self.pid.Ki, self.pid.Kd,
                 self.pid.sample_time,
                 self.pid.proportional_on_measurement,
                 self.pid.derivative_on_measurement)

    def target_logic(self):
        delta = self.monitor_temp.delta
        output = self.pid(delta)

        if output < 0:
            # If PID wants warmer, we're in good shape, so we can be at minimum
            target = self.min_fan
        else:
            target = self.min_fan + (output * self.pid_config.get(
                'fan_scale', 1))

        if target != self._last_target or delta != self._last_delta:
            LOG.debug('Max delta is %s Target %i%% want=%.1f',
                      delta, target, output)

        self._last_target = target
        self._last_delta = delta

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
