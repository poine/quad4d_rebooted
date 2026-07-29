# Paparazzi GCS strip icons

These SVG icons are vendored from **PprzGCS**
(<https://github.com/paparazzi/PprzGCS>, `resources/pictures/`), the
Paparazzi UAV ground control station, and are licensed under the GPL
like Paparazzi itself (compatible with this project's LICENSE).

Used in the operator window's drones panel to show, per drone, the
state of the position source (mocap), the RC link, the telemetry
downlink and the battery — the same visual language operators already
know from the GCS.

| file            | meaning                        |
|-----------------|--------------------------------|
| `gps_ok/nok`    | position source (mocap here)   |
| `rc_ok/nok`     | RC link                        |
| `link_ok/nok/warning` | telemetry downlink       |
| `battery_ok/warning/nok` | pack voltage (see below)  |

The `battery_*` files are the PprzGCS battery icons, renamed to this
project's `base_state` convention so they load through the same code
path as the others: `battery_ok` = `bat_ok` (green, full), `warning` =
`bat_low` (yellow, plan to land), `nok` = `bat_catastrophic` (red, land
now). PprzGCS also ships `bat_critic` (orange) between low and
catastrophic, unused here since the panel has three states. The voltage
thresholds live in `drones_panel.py` (`battery_state`).
