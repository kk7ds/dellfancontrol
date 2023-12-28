# Dell R730xd PID fan controller

I wrote this because my R730xd's fans run high with 3rd party drives. This is a known issue that Dell won't fix (probably to sell drives). This uses the IPMI interface to control the fan speed.

## Goals

1. Run the fans as quiet as possible to maintain a specific system temperature
2. React continuously to load (i.e. not just at a static percentage) smoothly to avoid spiking the fans when not needed, but also without letting things get too hot.
3. Be conservative and restore the system-managed fans any time anything fails
4. Maintain a minimum fan speed to move air through the system

## Design

This is a single python script and a YAML file for config. It uses a simple PID controller algorithm to try to find the fan speed that will keep temperatures at a specific point. Above that point and it will run fans to try to get it down to the desired point. Below and it will assume no (additional) cooling is needed.

## Requirements

You need IPMI local and `ipmitool` installed to work. You also need the `simple_pid` python module installed.

## Installation
```
$ sudo python3 -mpip install simple_pid
$ curl -O https://raw.githubusercontent.com/kk7ds/dellfancontrol/master/fancontrol.py
$ curl -O https://raw.githubusercontent.com/kk7ds/dellfancontrol/master/fancontrol.yaml
$ chmod +x fancontrol.py
```

## Running

Review the settings available in the YAML file and then run the script:
```
$ sudo ./fancontrol.py --config fancontrol.yaml --debug
```
Whenever the script runs, it will put the fan controller into manual mode and control the speed. Whenever the script exits, it will restore system-managed mode for safety. If the script fails, is killed, or detects that any of the `panic_temps` sensors are over limit, it will place the fans back into system-managed mode.

After tuning, you can run without `--debug` for less-verbose output.

## Tuning

PID tuning is a black art. See the plethora of available articles on the topic that google has to offer. The default `kp`, `ki`, and `kd` values work well for me, but my machine runs in a stable, cool room with very predictable load.

## Warning

This script makes every effort to fail into a safe situation, but no OS-based approach can truly do this perfectly. Use this with caution, monitor things closely, and know that you accept all the risk that comes from using this!
