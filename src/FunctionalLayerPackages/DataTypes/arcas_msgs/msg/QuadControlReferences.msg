# This contains the control references for a quadrotor. That is, the desired values of position, velocity and heading for the quadrotor.

# The position control reference measured in meters [m] with respect to the testbed frame (also known as map frame or global frame).
Position position_ref

# The cruise velocity control reference measured in meters per second [m/s] with respect to the global frame.
float64 velocity_ref

# The heading control reference measured in radians [rad] with respect to the quadrotor frame (body axes).
float64 heading