# auv_pi_stack

ROS 2 (Humble) stack for a low-cost AUV: one Raspberry Pi 4, three
thrusters, a Pixhawk as IMU/compass, a Bar02 depth sensor and a GPS on a
tethered surface buoy. Teleoperation, heading/depth hold and GPS waypoint
missions, with a safety layer where every failure ends in "thrusters
stopped, vehicle floats up".

MSc dissertation project, Sheffield Hallam University. This is **v2**: a
fresh workspace for the Pi + three-thruster vehicle. The v1 stack
(`auv_stack`, Jetson + fins + ballast) stays as the record of the first water
tests; what was learned there is kept in [`docs/DECISIONS.md`](docs/DECISIONS.md).

---

## Hardware and wiring

```
                     ┌─────────────── SURFACE BUOY (passive) ────────────────┐
                     │  u-blox M8N GPS                                       │
                     └───────────┬───────────────────────────────────────────┘
                                 │ serial (TX/RX/GND) down the tether
┌──────────────────────── VEHICLE HULL ─────────┼──────────────────────────────────────────┐
│                                               ▼                                          │
│   Pixhawk 2.4.8 ──USB──▶  RASPBERRY PI 4  ◀── UART /dev/serial0 (pins 8/10)              │
│   (ArduSub: IMU +         │   ▲                                                         │
│    internal compass)      │   │ WiFi (works only when surfaced) ──▶ operator laptop     │
│                           │ I²C bus 1 (pins 3/5) — Pi is the ONLY master                 │
│                           ▼                                                             │
│                     PCA9685 (0x40) ──pass-through I²C──▶ Bar02 depth (0x76)             │
│                       │ch2        │ch0          │ch1                                    │
│                       ▼           ▼             ▼                                       │
│                  T200 + ESC   T200 + ESC   generic thruster + forward-only ESC          │
│                    SURGE         YAW          PITCH                                     │
└─────────────────────────────────────────────────────────────────────────────────────────┘
```

| Part | Connected to | Used for |
|---|---|---|
| Raspberry Pi 4, Ubuntu 22.04, ROS 2 Humble | — | Every node in this repo |
| Pixhawk 2.4.8, ArduSub | Pi USB (`/dev/pixhawk`) | IMU + internal compass via mavros. **Not** in the actuation path (D9) |
| PCA9685 | Pi I²C bus 1 | PWM for all three ESCs |
| Bar02 | PCA9685 pass-through I²C → Pi bus 1 | Depth |
| u-blox M8N on the buoy | Pi UART, down the tether | Position (of the buoy) |
| Stern T200 + Basic ESC | PCA ch2 | Surge, bidirectional |
| Lateral T200 + Basic ESC | PCA ch0 | Yaw, bidirectional |
| Generic thruster + ESC | PCA ch1 | Pitch, **forward-only** |

Channels are carried over from v1 wiring — confirm against your harness and
`calibration.yaml`. **The Pixhawk must not be connected to the PCA9685's
SDA/SCL** (D3).

The vehicle is trimmed **positively buoyant**. There is no ballast; floating
up when stopped is the fail-safe (D1).

---

## How it fits together

```
 laptop:  joy_node ──/joy──┐                         (WiFi)
                           ▼
 Pi:      teleop_node ──/cmd/teleop──────┐   /heartbeat ──▶ safety_supervisor ──/estop (latched)──┐
                                         ▼                         │ /cmd/safety                   │
          navigator ──setpoints──▶ autopilot ──/cmd/autonomy──▶ actuator_mixer ──/cmd/actuators──▶ thruster_driver ──▶ PCA9685
             ▲  │ /heartbeat              ▲  ▲                                                     ▲
             │  └───────────────────────▶ │  │                                                     │
          gps_node ──/gps/fix             │  └── /depth ── depth_sensor             /estop ─────────┘
                              mavros ─────┘  (IMU: yaw, yaw rate)
```

Commands fan in to **one** choke point (the mixer) and **one** node touches
hardware (the thruster driver). Priority is safe > teleop > autonomy. The
e-stop reaches both the mixer and the driver independently.

### Nodes

| Node | Package | Does |
|---|---|---|
| `thruster_driver` | auv_drivers | `/cmd/actuators` → PCA9685 pulses, per-ESC calibration. Stops all ESCs on e-stop, on mixer silence, at boot and at exit |
| `depth_sensor` | auv_drivers | Bar02 → `/depth` (m, +down, only after tare), `/pressure` |
| `gps_node` | auv_drivers | NMEA on `/dev/serial0` → `/gps/fix` |
| `safety_supervisor` | auv_control | E-stop latch, heartbeat watchdog, `clear_estop` service |
| `actuator_mixer` | auv_control | Chooses safe / teleop / autonomy, clamps, publishes at 20 Hz |
| `teleop_node` | auv_control | `/joy` → thrust, heartbeat, e-stop and clear buttons |
| `autopilot` | auv_control | Heading hold (yaw thruster) + depth hold (pitch thruster) → `/cmd/autonomy` |
| `navigator` | auv_control | Waypoint mission: GPS → local frame → LOS → setpoints; geofence, timeout, GPS-loss abort |

### Topics and services you will use

| Name | Type | Notes |
|---|---|---|
| `/estop` | Bool, latched | Current stop state |
| `/estop_request` | Bool | Publish `true` to stop. `false` is ignored |
| `/safety_supervisor/clear_estop` | Trigger | Only way to clear; needs a live heartbeat |
| `/depth_sensor/tare` | Trigger | Call **at the surface** before any depth control |
| `/navigator/start`, `/navigator/stop` | Trigger | Start / abort the loaded mission |
| `/navigator/status` | String | State, waypoint, distance, cross-track |
| `/autopilot/heading_setpoint` | Float32 | rad, ENU, CCW from **east** (0 = east, 1.571 = north) |
| `/autopilot/depth_setpoint` | Float32 | m, +down |
| `/autopilot/surge_setpoint` | Float32 | normalised 0..1 |
| `/cmd/actuators` | ActuatorCommand | What the thrusters are actually being told |

Setpoints expire after 1 s, so send them with `-r 5` when testing by hand.

### Workspace layout

```
auv_ws/src/auv_pi_stack/
├── auv_interfaces/     ActuatorCommand.msg
├── auv_drivers/        thruster_driver, depth_sensor, gps_node (+ pca9685, esc, ms5837)
├── auv_control/        safety_supervisor, actuator_mixer, teleop_node, autopilot, navigator
│                       (+ pure maths: pid, geo, los, arbitration, mission — unit-tested)
├── auv_bringup/
│   ├── launch/         vehicle.launch.py (Pi), operator.launch.py (laptop)
│   └── config/         calibration.yaml (MEASURED), vehicle.yaml (TUNING), mavros.yaml, missions/
├── scripts/            esc_calibrate.py, bar02_test.py, i2c_check.sh, kill_stack.sh, 99-auv.rules
└── docs/DECISIONS.md   why things are the way they are
```

---

## Setting up the Pi (once)

Ubuntu Server 22.04 (arm64) + ROS 2 Humble (`ros-humble-ros-base`).

```bash
# ROS packages
sudo apt install ros-humble-mavros ros-humble-mavros-extras ros-humble-joy \
                 ros-humble-rosbag2 i2c-tools python3-serial python3-yaml
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
pip3 install --user smbus2 pynmea2          # never sudo pip into system Python

# Interfaces: in /boot/firmware/config.txt
#   dtparam=i2c_arm=on
#   enable_uart=1
#   dtoverlay=disable-bt        # gives /dev/serial0 the stable PL011 UART
# and remove "console=serial0,115200" from /boot/firmware/cmdline.txt, then reboot.
sudo usermod -aG dialout,i2c $USER

# Stable Pixhawk name (/dev/pixhawk)
sudo cp scripts/99-auv.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger

# Same ROS domain on the Pi AND the laptop
echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
```

On the laptop: ROS 2 Humble, `ros-humble-joy`, this repo built (for
`operator.launch.py`), same WiFi network and `ROS_DOMAIN_ID=42`. ROS 2
discovery uses multicast; a dedicated travel router is more reliable than a
shared network.

## Build

```bash
mkdir -p ~/auv_ws/src && cd ~/auv_ws/src
git clone <this repo> auv_pi_stack
cd ~/auv_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash          # in every new terminal
colcon test --packages-select auv_control auv_drivers && colcon test-result --verbose
```

---

## First bring-up (bench, propellers OFF)

Do these in order the first time, and again after any hardware change.

1. **I²C:** `scripts/i2c_check.sh` → `0x40` and `0x76` present, nothing odd.
2. **Depth:** `python3 scripts/bar02_test.py` → sensible pressure (~1000 mbar in air).
3. **GPS:** `cat /dev/serial0` shows `$GPGGA…` lines (outdoors for a fix).
4. **Pixhawk:** `ros2 launch auv_bringup vehicle.launch.py depth:=false gps:=false`,
   then `ros2 topic list | grep imu` — put the IMU topic you see into
   `vehicle.yaml` → `autopilot.imu_topic`. Check heading on `ros2 topic echo`:
   pointing **east** must read yaw ≈ 0, north ≈ +1.57. Calibrate the compass
   in QGroundControl **with the thrusters and battery installed** — the
   motors' magnets bias it.
5. **Calibrate each ESC** (below), write the values into `calibration.yaml`.
6. **Directions:** launch with `local_joy:=true` (or the laptop), hold LB:
   right trigger → surge forward; left stick left → nose turns **left**;
   right stick pushed → nose **down**. Fix any wrong one with its `*_sign`
   in `calibration.yaml` (hardware) — not the teleop scale.
7. **Safety acceptance tests** (below). All must pass before water.

## Calibration

```bash
# one ESC per run; script streams stop first — power the ESC AFTER it starts
python3 scripts/esc_calibrate.py --channel 2 --stop 1500 --min 1100 --max 1900   # surge T200
python3 scripts/esc_calibrate.py --channel 0 --stop 1500 --min 1100 --max 1900   # yaw T200
python3 scripts/esc_calibrate.py --channel 1 --stop 1000 --min 1000 --max 2000   # pitch, forward-only
```

Use `u`/`d` (±5 µs) to find the pulse where the motor is silent and still —
that is `*_stop_us`. For the forward-only ESC, `stop` is its minimum
throttle; `span` is (full throttle − stop). If the pitch thruster pushes the
nose **up** rather than down, set `pitch_sign: -1` **and** the autopilot's
`depth_out_min: -1.0`, `depth_out_max: 0.0`.

---

## Running

### Teleop

```bash
# Pi
ros2 launch auv_bringup vehicle.launch.py
# laptop
ros2 launch auv_bringup operator.launch.py
```

Press **Y** to clear the e-stop (refused unless `/joy` is arriving), hold
**LB** and drive. **B** = e-stop. Releasing LB → thrusters stop.

### Heading / depth hold by hand

```bash
ros2 launch auv_bringup vehicle.launch.py autopilot:=true
ros2 service call /depth_sensor/tare std_srvs/srv/Trigger          # at the surface
ros2 topic pub -r 5 /autopilot/heading_setpoint std_msgs/Float32 "{data: 1.57}"   # north
ros2 topic pub -r 5 /autopilot/surge_setpoint   std_msgs/Float32 "{data: 0.2}"
ros2 topic pub -r 5 /autopilot/depth_setpoint   std_msgs/Float32 "{data: 0.5}"
```

Something must be sending a heartbeat for the e-stop to clear — keep the
laptop gamepad running (don't hold LB, or teleop takes over).

### Waypoint mission

```bash
ros2 launch auv_bringup vehicle.launch.py mission:=bench_local record:=true
ros2 service call /depth_sensor/tare std_srvs/srv/Trigger     # if depth_m > 0
ros2 service call /navigator/start std_srvs/srv/Trigger       # needs a GPS fix
# then press Y on the gamepad (or call /safety_supervisor/clear_estop)
ros2 topic echo /navigator/status
```

Missions live in `auv_bringup/config/missions/` (format in `navigator.py`).
`frame: local` waypoints are east/north metres from the first GPS fix;
`frame: global` are lat/lon. The GPS measures the **buoy**, so the
acceptance radius must cover GPS error plus how far the buoy trails the
vehicle on its tether.

When the mission completes, the navigator goes quiet: the autopilot's
setpoints expire, the mixer falls back to stopped and the vehicle floats. If
no gamepad is connected (the usual case once submerged) the watchdog also
latches the e-stop. Aborts (geofence, timeout, GPS lost) request the e-stop
immediately.

---

## Safety behaviour

| Event | Result |
|---|---|
| Boot | Every ESC at its own stop pulse; e-stop latched |
| B pressed / `true` on `/estop_request` / navigator abort | E-stop latched; mixer and driver both stop |
| Gamepad or WiFi lost while in teleop | Heartbeat stops → watchdog latches e-stop (0.5 s) |
| LB released | Teleop goes quiet → autonomy if running, else stop |
| Mixer crashes | Thruster driver stops all ESCs (0.5 s) |
| Supervisor crashes | Mixer forces safe (0.5 s) |
| Autopilot / navigator crashes | Their commands/setpoints expire → stop |
| IMU stale | Heading loop **and surge** muted |
| `/depth` stale or untared | Depth loop muted (pitch thruster off) |
| Mission complete | Navigator quiet → setpoints expire → stopped, floats (watchdog latches if no gamepad) |
| Submerged during a mission | **No remote e-stop** (D4): geofence, timeout, GPS-loss abort and buoyancy only |

### Acceptance tests (before every water session)

1. Boots with every ESC at its stop pulse (`ros2 topic echo /cmd/actuators` all 0; motors silent)
2. `clear_estop` is refused with no gamepad running; succeeds with it
3. Teleop commands are ignored while stopped
4. Releasing LB stops the thrusters
5. Switching the laptop's WiFi off during teleop stops the thrusters within 0.5 s
6. `ros2 topic pub --once /estop_request std_msgs/Bool "{data: false}"` is ignored and logged
7. A pitch command in the direction the forward-only thruster can't push leaves it stopped
8. Killing `actuator_mixer` (`pkill -f actuator_mixer`) stops the thrusters
9. `/navigator/start` is refused without a GPS fix
10. **In the water, all thrusters stopped, the vehicle floats** (D1)

---

## Tuning notes

All gains in `vehicle.yaml` are starting guesses; nothing transfers from the
v1 fin vehicle. Suggested order: yaw (P then D) at zero surge, then heading
with cruise, then depth (P, D, then a little I to absorb buoyancy trim).
The yaw thruster lets the vehicle turn on the spot — set `pivot_error_rad`
(e.g. 0.8) to turn in place before driving each leg. Mixer limits are 0.6
for first water sessions.

## Licence

MIT. See `LICENSE`.
