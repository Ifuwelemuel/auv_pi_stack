# Design decisions

Each entry is a decision this stack depends on and why. Many were learned the
hard way on v1 (`auv_stack`, Jetson + fins); the old ADR number is given so
the history can still be traced. If you change one of these, change it here
first.

---

### D1 — Safe state is "all thrusters stopped"; the vehicle floats up
*v1: fail-to-surface principle; ballast ADR-004 retired.*

There is no ballast. The vehicle is trimmed **positively buoyant**, so a
stopped vehicle rises. This is the only fail-to-surface mechanism that still
works when the electronics are dead, so it is a **pre-dive check**, not a
software feature. Every safety path in the stack ends in "all three ESCs at
their own stop pulse".

### D2 — One computer; the buoy GPS is wired straight to it
*v1: supersedes ADR-018 (two-machine chrony time sync).*

The Raspberry Pi in the hull is the only computer. The buoy is passive: the
u-blox receiver's serial line runs down the tether to the Pi. GPS, IMU and
depth are all stamped on the Pi's clock, so the inter-machine time offset
that v1 had to control for no longer exists. TTL serial at 9600 baud is fine
over a few metres of tether; for long tethers use RS-485 transceivers.

### D3 — I²C: the Pi is the only master
Pi bus 1 → PCA9685 (0x40) → Bar02 (0x76, on the PCA's pass-through header).
The Pixhawk must **not** be on this bus. Two masters on one bus show up as
`Errno 121`, frozen PWM outputs and an ESC holding its last pulse.
`scripts/i2c_check.sh` checks the bus.

### D4 — Operator link is WiFi, surfaced only
Only `joy_node` runs on the laptop; `teleop_node` runs on the Pi, so the
heartbeat is produced on the vehicle from arriving `/joy`. Losing WiFi is
treated exactly like losing the controller: if teleop was in command, the
watchdog stops the vehicle and it floats up.
**Consequence: there is no remote e-stop during a submerged autonomous run.**
The navigator's geofence, mission timeout and GPS-staleness abort, plus
positive buoyancy, are the safety net. Start every new mission small.

### D5 — PCA9685 oscillator is measured, not assumed
*v1: ADR-016, ADR-020.*

This board's oscillator measured **28.73 MHz**, not the nominal 25 MHz. With
the nominal value every pulse came out ~15% short (1500 µs left the pin as
1307 µs) and no ESC would arm. The prescale is computed from the measured
value (`oscillator_hz` in `calibration.yaml`). It belongs to the PCA board:
re-measure (Arduino `pulseIn` or a scope) if the board is replaced.

### D6 — Every ESC has its own measured stop pulse
*v1: ADR-016.*

Two ESCs of the same model differ, and the generic pitch ESC is a different
type entirely: **forward-only**, stop = minimum throttle (~1000 µs), and it
arms only if it sees that pulse at power-up. Giving it the T200's 1500 µs
neutral would be half throttle. Each ESC's stop, span, sign and
bidirectional flag live in `calibration.yaml`, measured with
`scripts/esc_calibrate.py`. A command in the direction the pitch thruster
cannot push is sent as "stopped", never as a below-minimum pulse.
The driver also reads the PCA9685's configuration back after init, because a
failed init is silent (chip asleep, writes "succeed", nothing moves).

### D7 — E-stop: easy to engage, deliberate to clear
*v1: ADR-019.*

The vehicle boots stopped. Anyone may publish `true` on `/estop_request` at
any time; a replayed or duplicated stop is harmless. `false` on that topic is
ignored and logged. The **only** way to clear is the
`/safety_supervisor/clear_estop` service, which is refused unless a commander
heartbeat is live — you cannot make the vehicle live with nobody in command.
(v1 bug that led here: a latched `false` was replayed to a late subscriber
and cleared the boot e-stop.)

### D8 — QoS profiles are shared, never re-typed
State (`/estop`) is reliable + transient-local, so late starters get it.
Commands and requests are reliable + volatile + depth 1, so they are never
replayed and never queued. Mismatches fail silently: a volatile subscriber
on `/estop` connects but misses the value published before it started, and a
transient-local subscriber does not connect to a volatile publisher at all.
So both sides import the profiles from `auv_control/qos.py`.

### D9 — Pixhawk is a sensor hub only
*v1: ADR-002, ADR-011, ADR-012.*

ArduSub provides the calibrated IMU and compass through mavros. It is not in
the actuation path: mavros command/override paths were found non-functional
on this autopilot combination in v1, and no ArduSub frame models this
vehicle. All control runs in ROS 2 on the Pi.

### D10 — Never stack two launches
*v1: ghost nodes.*

A second launch on top of a running one gives two publishers per topic and
very confusing behaviour. `vehicle.launch.py` refuses to start if stack
nodes are running. `scripts/kill_stack.sh` clears them.

### D11 — One choke point, and every link fails safe
*v1: mixer principle; driver and supervisor checks are new in v2.*

Only the mixer publishes `/cmd/actuators`, and only the thruster driver
touches the PCA9685. Each link has a staleness rule measured on the
receiver's own clock:

| If this goes quiet… | …this happens |
|---|---|
| teleop / autonomy command | mixer drops that source (0.3 s) |
| safety supervisor (`/cmd/safety`) | mixer forces safe (0.5 s) — a dead supervisor is a dead watchdog |
| mixer (`/cmd/actuators`) | thruster driver stops all ESCs (0.5 s) — v1 would have held the last pulse |
| commander heartbeat | supervisor latches the e-stop (0.5 s) |
| autopilot setpoints | autopilot stops publishing (1 s) |
| GPS fix during a mission | navigator aborts and requests the e-stop (3 s) |

### D12 — Setpoints expire; integrators reset only on real changes
*New in v2.*

v1's heading controller reset its integrator on every setpoint message, and
the guidance re-sent the setpoint at 5 Hz, so the I-term could never build.
Now integrators reset when a loop starts or its setpoint jumps by more than a
threshold. Setpoints expire after `setpoint_timeout_s`, so a crashed
navigator cannot leave the autopilot steering a stale course.

### D13 — One heading convention, explicit mission frames
*v1: ADR-028, ADR-031.*

Heading is ENU yaw, radians, counter-clockwise from **east**, everywhere:
mavros IMU, LOS bearing, autopilot. The local frame is east/north metres
about the first good GPS fix of the session. Mission files must say
`frame: global` or `frame: local`; anything else is rejected, not guessed.

### D14 — No depth until tared, and tare is manual
Depth is published only after `/depth_sensor/tare` is called **at the
surface**. Wrong-datum depth is worse than none. An automatic tare after a
node restart underwater would read the current depth as zero.

### D15 — Calibration and tuning are separate files
`calibration.yaml` holds measured facts about this hardware (oscillator, ESC
stops, channels, ports). `vehicle.yaml` holds gains, limits and behaviour.
You re-tune often; you re-calibrate only when hardware changes.

---

### Not carried over from v1
- **Learned dead reckoning (`auv_learning`)** — reserved for the follow-on
  paper; it stays in the v1 repo. This stack still records the bags it will
  need (`record:=true`).
- **Fins, servo ranges, ballast, BTS7960** (ADR-004, ADR-013) — hardware gone.
- **Jetson L4T package holds** (ADR-010) — no Jetson.
- **Two-machine networking and chrony** (ADR-018) — see D2.

### D16 — Timeouts run on the monotonic clock
*New in v2, found on first boot.*

The Pi has no battery-backed clock: it booted three months behind, and NTP
steps it when the Pi gets internet. Every staleness check (watchdog, mixer
source ages, driver command timeout, autopilot sensor/setpoint ages,
navigator GPS age and mission timer) uses `ClockType.STEADY_TIME`, which
never jumps. In simulation, a backward clock step at the moment the gamepad
dropped left the vehicle driving at 60% surge with the watchdog blind; with
the monotonic clock it stops within 0.35 s. Message stamps still use ROS
time, so set the clock (`timedatectl`) before recording bags.

### D17 — No automatic updates on the vehicle
`unattended-upgrades` upgraded mavros without libmavconn and broke mavros.
Automatic updates are disabled; update deliberately, on the bench, all ROS
packages at once.
