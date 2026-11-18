# One scan of the ground radar: every echo (target, the follower itself, clutter), unlabelled.
std_msgs/Header header             # stamp = time the scan was TAKEN (it arrives later: datalink)
geometry_msgs/Point site           # radar position, world ENU
RadarPlot[] plots
