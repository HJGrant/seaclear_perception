std_msgs/Header header
int32 track_id
float32 u
float32 v
float32 w
float32 h
float32 depth_m
float32 confidence
float32[] keypoints_uv
string source

# Tracker / selector contract for the active visual-servo path.
# Existing fields above keep the selected target geometry for compatibility.
bool selected_valid
int32 selected_track_id
string track_state
string reason_code
uint32 age_frames
uint32 miss_count
float32 freshness
