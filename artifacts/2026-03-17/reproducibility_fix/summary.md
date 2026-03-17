# Reproducibility fix summary

## Files changed

- `.gitmodules`
- `README_quickstart.md`
- `src/isaac_ros_common/scripts/run_dev.sh`
- `src/grip_points_service` removed as a stale gitlink from the index

## Why these changes were made

- Fresh-clone reproduction for the visual-servo MVP was blocked by an over-strict LFS preflight in `run_dev.sh`.
- Fresh-clone submodule initialization was blocked by a mismatch between HEAD gitlinks and `.gitmodules`.
- The quickstart did not document the current LRZ `dev/aoki` assumptions around WSL, Docker Desktop, `ssh-agent`, or the supported MVP-only LFS skip workflow.

## Verification summary

- `bash -n` passed for the touched host/container helper scripts.
- `git submodule status --recursive` now parses all active submodules instead of failing on missing `.gitmodules` mappings.
- `git submodule sync --recursive` succeeds.
- `git submodule update --init --recursive` no longer fails immediately on metadata inconsistency; in this existing non-fresh workspace it proceeds to real checkout work and then stops on a local checkout collision under `src/isaac_ros_visual_slam`.

## Remaining external/local blockers

- Full build/test/launch/manual publish were not rerun in this sandbox.
- Submodule checkout in this workspace is affected by pre-existing files under `src/isaac_ros_visual_slam`; that is a local-state issue, not a `.gitmodules` mismatch.
- Any submodule fetch still depends on normal network/credential access from the host.
