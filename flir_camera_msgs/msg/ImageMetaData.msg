#
# meta data messages for camera images
#
std_msgs/Header header

# time stamp produced by camera, interpretation depends on camera model
uint64 camera_time

# Camera-provided monotonically increasing frame identifier. This permits
# offline detection of physical acquisition gaps independently of ROS/DDS.
uint64 camera_frame_id

# True only when header.stamp was assigned by the shared synchronized-camera
# estimator. False messages are archival fallbacks whose header uses the host
# arrival time; camera_time and camera_frame_id remain the authoritative raw
# timing identifiers for post-processing.
bool synchronized_timestamp_valid

# computed brightness, either 0...255 (valid) or -1 (invalid)
int16 brightness

# exposure time in microseconds
uint32 exposure_time

# max allowed exposure time in microseconds
uint32 max_exposure_time

# gain in db
float32 gain

# line status
int64 line_status
