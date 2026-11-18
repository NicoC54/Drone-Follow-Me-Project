# Region of interest sent by the tracker to the detector ("cueing"): where to look in the next
# image, from the predicted target position projected with the current camera pose.
std_msgs/Header header
bool valid
float64 u
float64 v
float64 half_size    # [px]
