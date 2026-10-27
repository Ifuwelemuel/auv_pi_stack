# One actuator command for the whole vehicle.
#
# Only actuator_mixer publishes this on /cmd/actuators. Safety, teleop and
# autonomy each publish their request on their own topic (/cmd/safety,
# /cmd/teleop, /cmd/autonomy) and the mixer picks one.
#
# Every field is normalised intent in [-1, 1], NOT a pulse width. The
# thruster driver owns the conversion to PWM and the per-ESC calibration.
#
# Body frame FLU (REP-103): x forward, y left, z up.
#   thrust_surge > 0 : forward                        (stern T200)
#   thrust_pitch > 0 : nose DOWN (rotation about +y)  (generic, forward-only)
#   thrust_yaw   > 0 : nose LEFT (rotation about +z)  (lateral T200)
#
# The pitch thruster can only push one way. The driver turns a command in
# the direction it cannot push into "stopped".

std_msgs/Header header

float32 thrust_surge
float32 thrust_pitch
float32 thrust_yaw

uint8 source

uint8 SOURCE_SAFETY=0
uint8 SOURCE_TELEOP=1
uint8 SOURCE_AUTONOMY=2
