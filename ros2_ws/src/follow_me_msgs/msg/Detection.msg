# Output of the detector for one camera image.
# Measurement z = [u, v, size]: bounding-circle centre and apparent diameter, in pixels.
std_msgs/Header header      # stamp = image capture time
bool detected               # false: nothing found (or injected miss)
float64 u
float64 v
float64 size
float64 thickness           # silhouette: 4 x vertical standard deviation of the blob pixels [px] (0: n/a)
bool size_valid             # false: beacon truncated (occluded, image border) -> use u, v only
bool injected_outlier       # true if this detection was injected on purpose (fault injection), for metrics only
