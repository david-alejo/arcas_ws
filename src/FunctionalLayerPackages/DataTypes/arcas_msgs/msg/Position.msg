# This contains the position of a point in free space measured in meters [m].

# It indicates if the message is valid or not.
# If status = 1 => The data is valid.
# If status = 0 => The data is invalid (its values can be corrupted).
# A different value has no meaning but may be established by a particular application.
byte status

# The position value in the X axis.
float64 x

# The position value in the Y axis.
float64 y

# The position value in the Z axis.
float64 z