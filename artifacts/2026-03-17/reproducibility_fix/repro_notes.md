# Fresh-clone reproduction notes for `dev/aoki`

## Intended host-side setup

```bash
git clone --recurse-submodules <repo-url>
cd oakd_isaac_ros
git submodule sync --recursive
git submodule update --init --recursive

# If private access is needed, keep ssh-agent active in this same terminal.
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/<your_key>

SKIP_LFS_CHECK=1 ./src/isaac_ros_common/scripts/run_dev.sh --isaac_ros_dev_dir "$(pwd)"
```

## Intended container-side MVP validation

```bash
cd /workspaces/oakd_isaac_ros
./scripts/ensure_dev_container_ready.sh

./scripts/colcon_build_venv.sh --packages-up-to \
  sc2_visual_servo_bringup \
  sc2_visual_servo_controller \
  sc2_visual_servo_perception \
  sc2_visual_servo_interfaces \
  sc2_visual_servo_synth

source install/setup.bash

./scripts/colcon_test_venv.sh --packages-select \
  sc2_visual_servo_perception \
  sc2_visual_servo_controller \
  sc2_visual_servo_bringup

ros2 launch sc2_visual_servo_bringup mvp_sim.launch.py use_stub_target:=false
```

Manual check targets:
- Publish a centered `TargetObservation` and confirm `/sc2/visual_servo/cmd_vel_body` is near zero.
- Publish an offset `TargetObservation` and confirm the command magnitude increases in the error-reducing direction.

## What this pass verified locally

- `run_dev.sh` now supports an explicit `--skip_lfs_check` option and the existing `SKIP_LFS_CHECK=1` environment variable is documented as the supported workaround for MVP-only reproduction.
- `.gitmodules` now matches the gitlinks in HEAD for the active submodules used by the repository layout.
- The stale `src/grip_points_service` gitlink was removed from the index because it had no matching `.gitmodules` entry and no active references in the current workflow.
- The stale `.gitmodules` entry for `src/foxglove_compressed_video` was removed because HEAD no longer contains that gitlink.
- `git submodule sync --recursive` succeeds after the metadata fix.
- `git submodule update --init --recursive` now reaches actual checkout/fetch work instead of dying immediately on `.gitmodules` inconsistency.

## Limits of this sandbox run

- Full Docker/container build, test, and launch were not rerun in this environment.
- The bounded `git submodule update --init --recursive` log shows a checkout collision in `src/isaac_ros_visual_slam` caused by pre-existing local files in this non-fresh working tree. A fresh clone should not have that collision.
