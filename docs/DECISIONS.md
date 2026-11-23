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

### D2 — One computer; the buoy GPS goes to the Pixhawk
*v1: supersedes ADR-018 (two-machine chrony time sync). Revised 28 Oct 2026.*

The Raspberry Pi in the hull is the only computer. The buoy is passive: the
u-blox receiver's serial line runs down the tether to the Pixhawk's GPS
port, and mavros forwards it (`/mavros/global_position/raw/fix`). The Pi
talks to the Pixhawk over its UART ↔ TELEM2 at 921600. There is still one
clock that stamps everything (the Pi's), so v1's inter-machine time offset
does not come back. `gps_node` stays for a GPS wired to the Pi instead
(`gps:=true`, `navigator.gps_topic: /gps/fix`). TTL serial is fine over a few
metres of tether; for long tethers use RS-485 transceivers.

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

Two ESCs of the same model differ. The generic pitch ESC was assumed
forward-only (stop = minimum throttle, ~1000 µs) until a bench test on
30 Oct 2026 showed it is bidirectional: 1100 / 1500 / 1900 µs, like the
T200s. The driver still supports forward-only ESCs (`*_bidirectional: false`):
there, a command in the direction it cannot push is sent as "stopped",
never as a below-minimum pulse. Each ESC's stop, span, sign and
bidirectional flag live in `calibration.yaml`; each stop is also set on the
Pixhawk as `AUVW_STOPn`, and the driver checks the two agree (D18). With
the PCA9685 fallback, the
driver also reads its configuration back after init, because a failed init
is silent (chip asleep, writes "succeed", nothing moves).

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

### D9 — No control on the Pixhawk
*v1: ADR-002, ADR-011, ADR-012. Revised 29 Oct 2026 (see D18).*

ArduSub provides the calibrated IMU, compass and GPS through mavros. It runs
no control loop for this vehicle: no ArduSub frame models it, and every
controller, limit and safety rule lives in ROS 2 on the Pi. Since D18 the
Pixhawk is also the PWM output stage, but only as a pass-through: it copies
the pulse widths the Pi computed, or holds the stops. v1's "mavros override
paths don't work" finding was about fin servos on that build; on ArduSub 4.7
both DO_SET_SERVO and RC overrides drove the T200 (bench test, 29 Oct 2026).

### D10 — Never stack two launches
*v1: ghost nodes.*

A second launch on top of a running one gives two publishers per topic and
very confusing behaviour. `vehicle.launch.py` refuses to start if stack
nodes are running. `scripts/kill_stack.sh` clears them.

### D11 — One choke point, and every link fails safe
*v1: mixer principle; driver and supervisor checks are new in v2.*

Only the mixer publishes `/cmd/actuators`, and only the thruster driver
commands the ESCs. Each link has a staleness rule measured on the
receiver's own clock:

| If this goes quiet… | …this happens |
|---|---|
| teleop / autonomy command | mixer drops that source (0.3 s) |
| thruster driver, mavros, the Pi, TELEM2 | Pixhawk script stops every ESC (≤ 0.35 s, D18) |
| the Pixhawk Lua script | ArduPilot returns the pins to `SERVOn_TRIM` = stop (0.1 s, D18) |
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

### D16 — Timeouts run on the monotonic clock
*New in v2, found on first boot.*

The Pi has no battery-backed clock: it booted showing a date three months
old, and NTP will step it when the Pi reaches the internet. Every staleness
check (watchdog, mixer source ages, driver command timeout, autopilot sensor
and setpoint ages, navigator GPS age and mission timer) uses
`ClockType.STEADY_TIME`, which never jumps. A backward wall-clock step would
otherwise make old commands look fresh and blind the watchdog. Message
header stamps still use ROS time, so bags line up with the wall clock; run
`sudo timedatectl set-ntp true` and check `date` before recording a dataset.

### D17 — No automatic updates on the vehicle
*New in v2, found on first boot.*

unattended-upgrades upgraded half of the mavros packages and held the dpkg
lock mid-bring-up (undefined symbol between libmavros and libmavconn). It and
the apt timers are disabled; upgrade deliberately, all mavros packages
together.

### D18 — Thruster PWM from the Pixhawk, with a watchdog on the Pixhawk
*New 29 Oct 2026. Supersedes the PCA9685 as output stage.*

The PCA9685's RC oscillator drifts with temperature and supply (D5), so ESC
stop pulses kept needing re-measurement, and a T200 creeps when its neutral
moves by more than its ~±25 µs dead band. The Pixhawk's PWM comes from a
crystal, and it is already in the hull.

**Path.** `thruster_driver` computes each ESC's pulse width exactly as
before (`esc.py`, per-ESC stop/span/sign) and streams them at 20 Hz to
`/mavros/rc/override`: RC9-11 = AUX1-3, plus RC16 flipped between 1100 and
1900 on every message. `pixhawk/auv_watchdog.lua` copies RC9-11 to AUX1-3
(SERVO9-11, functions Script1-3) only while RC16 keeps flipping.

**Why the toggle and the script.** Tested on the bench: ArduSub holds the
last value on its own. DO_SET_SERVO holds forever; an RC override that times
out leaves `radio_in` at the last value (no receiver to fall back to), so
the thruster kept spinning after mavros was killed. A flipping channel is the
only proof of a live sender: if it stops flipping for `AUVW_TMO_MS` (300 ms),
the script drives every ESC to `AUVW_STOPn`. This closes a gap the PCA9685
always had: it too held the last pulse forever if the Pi died.

**If the script itself dies.** Each script write lasts 100 ms. AUX1-3 use
the Script1-3 functions, which are angle-type, and the script clears
ArduPilot's "PWM set directly" flag before each write, so when a write
expires the pin is recomputed as `SERVOn_TRIM`. The script keeps
`SERVOn_TRIM = AUVW_STOPn` (RAM only). With function 0 (Disabled) the pin
would hold its last value (read from the Sub-4.7.0 source,
`SRV_Channels::set_output_pwm_chan_timeout`).

**Two copies of each stop pulse, checked.** `AUVW_STOPn` (Pixhawk) and
`*_stop_us` (`calibration.yaml`) must agree, or the failsafe would send the
wrong pulse: a "stop" that is not the ESC's real stop is a running
thruster (1000 µs is close to full reverse on these ESCs). At every
start the driver sends nothing, reads `/mavros/rc/out` (the Pixhawk is then
in failsafe) and refuses to run unless AUX1-3 are at the expected stops.
`AUVW_STOPn = 0` means "never drive this output", so an unconfigured ESC
cannot arm. While running, the driver also warns if the reported outputs do
not follow its commands.

**At boot** the AUX pins send no pulse until the script starts, then the
stops, with or without the Pi. ESCs arm whenever their battery is switched
on after the Pixhawk has booted.

**Kept:** the PCA9685 output (`output: pca9685`) as a fallback, and the
board itself as the Bar02's I²C pass-through.

**Tested:** `pixhawk/test_auv_watchdog.lua` runs the real script against a
model of the ArduSub behaviour above (34 checks). The software-in-the-loop sim
runs the whole stack against a fake Pixhawk: driver, mavros and script death
all stop the ESCs. On the bench, still to do: the acceptance tests 9 and 10
in the README.

### D19 — Two hardware configurations; actuators are just outputs
*New 2 Nov 2026.*

The hull takes either three thrusters (surge, pitch, yaw) or a torpedo tail:
the stern T200 plus four fin servos in a + layout (top/bottom rudders,
left/right elevators).

**Nothing above the driver changes.** Teleop, the autopilot and the mixer
keep producing normalised surge / pitch / yaw. `thruster_driver` reads a
list of outputs (`outputs`), each an ESC or a servo that follows one of
those axes, on its own stage (Pixhawk AUX or PCA9685), with its own
calibration (`actuators.py`). `torpedo.yaml` is an overlay loaded after
`calibration.yaml` and `vehicle.yaml` by `config:=torpedo`; it replaces the
output list and turns the autopilot's pivot off (fins cannot turn the
vehicle on the spot).

**Fins on the PCA9685, thruster on the Pixhawk.** The fins could have gone
on Pixhawk AUX4-6 behind the watchdog, which meant a 6-output script and
more Pixhawk setup. On the PCA they need nothing on the Pixhawk, and the
PCA's two weaknesses (D5, D18) matter little for servos: a few µs of drift is
a fraction of a degree, and a fin stuck at its last angle when the Pi dies
does nothing once the Pixhawk has stopped the thruster. The fail-safe chain
(D1) is unchanged: thruster stops, vehicle floats up. While the driver is
alive, every stop path (e-stop, mixer or teleop silence) also centres the
fins.

**Servos get mechanical limits** (`*_min_us`, `*_max_us`): the pulse is
clamped there whatever the command or span, so a fin is never driven into
its end stop.

**Unused Pixhawk AUX outputs get an explicit "no command".** ArduSub keeps
an RC override value forever, so an AUX output the current configuration
does not use (AUX1/2 in torpedo) would replay what an earlier session sent,
e.g. a yaw-thruster command, to an ESC that may still be plugged in. The
driver sends NO_CMD (900 µs) on every AUX it does not use; the script holds
those at their stop. Found while adding the fins; covered by a sim scenario.

---

### Not carried over from v1
- **Learned dead reckoning (`auv_learning`)** — reserved for the follow-on
  paper; it stays in the v1 repo. This stack still records the bags it will
  need (`record:=true`).
- **Fins, servo ranges, ballast, BTS7960** (ADR-004, ADR-013) — hardware gone.
- **Jetson L4T package holds** (ADR-010) — no Jetson.
- **Two-machine networking and chrony** (ADR-018) — see D2.
