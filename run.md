source /opt/ros/humble/setup.bash
source /workspaces/oakd_isaac_ros/scripts/source_workspace_env_safe.sh /workspaces/oakd_isaac_ros/install/setup.bash
python3 - <<'PY'
import numpy, sys
print("python:", sys.executable)
print("numpy:", numpy.__version__, numpy.__file__)
from cv_bridge import CvBridge
CvBridge()
print("cv_bridge ok")
PY



cd /workspaces/oakd_isaac_ros
./scripts/run_oakd_system_check.sh --profile jetson_live --launch-seconds 20
