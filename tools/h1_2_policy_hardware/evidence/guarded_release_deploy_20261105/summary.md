# Guarded release v3 board deployment — 2026-11-05

- Persistent release: `/home/unitree/h1_nav2_ros2_ws/releases/guarded_release_v3_20261105`.
- Formal policy package symlink now resolves to this release's `install/h1_2_policy_bridge`; formal source and paired runner synchronized. NavTools/bringup were not switched.
- Board core/node fake-IO tests: 2/2 passed. Source node/core/runner hashes matched local before promotion.
- Standalone CMake installation lacked the colcon package dependency index. Restored `share/colcon-core/packages/h1_2_policy_bridge` from the previous installation; ordinary workspace setup then discovered the new package.
- Output-disabled check from ordinary workspace setup: protocol v3, valid LowState, authorized=false, publisher_present=false, write_count=0, node exit 0. No authorization, shadow inference, or physical action was requested.
- Backup/rollback: release `backup/pre_promote/rollback.sh`; stop control processes before rollback. Source and runner must be rolled back together with the package symlink.
- This verifies deployment and communication only. No real motor/ONNX action or safe physical release is established.
