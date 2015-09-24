#This contains a list with the waypoints that defines the configuration of every ARM. The first and last waypoint  will be the initial and final position of the ARM and the others, positions that the ARM must be reached.
#The topic prefix to use when the UAV pathWithCruise generated are sended.
string topic_prefix
#The main waypoints of the ARM.
PathWithCruise[] arm_waypoints
