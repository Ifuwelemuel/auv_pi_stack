#one actuator command for the while vehicle
#only one acyuator mixer publishers this on command /cmd/actuators
#   thrust_surge > 0 : forward                        (stern T200)
#   thrust_pitch > 0 : nose DOWN (rotation about +y)  (generic, forward-only)
#   thrust_yaw   > 0 : nose LEFT (rotation about +z)  (lateral T200)


std_msgs/Header header

float32 thrust_surge
float32 thrust_pitch
float32 thrust_yaw

uint8 source

uint8 SOURCE_SAFETY=0
uint8 SOURCE_TELEOP=1
uint8 SOURCE_AUTONOMY=2

