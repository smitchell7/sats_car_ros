# Message that transports user commands. 

# the header contains basic timestamp info
std_msgs/Header header
# manual activates / deactivates the high_level controller
bool master_on
# update parameters on all vehicles
bool update_param

