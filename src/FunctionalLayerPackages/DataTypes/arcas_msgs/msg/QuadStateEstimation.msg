#It contains the state estimation of a quadrotor. It is formed by the data of position, attitude, linear and angular velocity, acceleration, altitude above ground and attitude commands.
#The quadrotor position measured in meters [m] with respect to the testbed frame (also known as map frame or global frame).
Position position
#The quadrotor orientation measured in radians [rad] with respect to the quadrotor frame (body axes).
Attitude attitude
#The quadrotor linear velocity measured in meters per second [m/s] with respect to the testbed frame (also known as map frame or global frame).
LinearVelocity linear_velocity
#The quadrotor angular velocity measured in radians per second [rad/s] with respect to the quadrotor frame (body axes).
AngularVelocity angular_velocity
#The quadrotor altitude above ground measured in meters [m] with respect to the testbed frame (also known as map frame or global frame).
float64 altitude
#The quadrotor linear acceleration measured in meters per second squared with respect to the testbed frame (also known as map frame or global frame).
LinearAcceleration linear_acceleration
#The quadrotor attitude commands.
AttitudeCommands attitude_commands
#The flying state of the UAV.
uint8 flying_state
#
uint8 LANDED=1
#
uint8 TAKING_OFF=2
#
uint8 FLYING=3
#
uint8 LANDING=4
