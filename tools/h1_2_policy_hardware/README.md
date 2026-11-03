# Preparation acceptance tools

The monitor subscribes to LowState and LowCmd. Build it against the same Unitree SDK and its CycloneDDS libraries as the policy node; for example, on the board with the existing dependency layout:

```bash
SDK=/home/unitree/h1_nav2_ros2_ws/third_party/h1_policy_runtime/unitree_sdk2
ARCH=$(uname -m)
g++ -std=c++17 -O2 -I"$SDK/include" -I"$SDK/thirdparty/include" -I"$SDK/thirdparty/include/ddscxx" tools/h1_2_policy_hardware/prepare_monitor.cpp "$SDK/lib/$ARCH/libunitree_sdk2.a" "$SDK/thirdparty/lib/$ARCH/libddscxx.so" "$SDK/thirdparty/lib/$ARCH/libddsc.so" -pthread -Wl,--disable-new-dtags,-rpath,"$SDK/thirdparty/lib/$ARCH" -o /tmp/h1_prepare_monitor
```

After review, install that binary at `/home/unitree/h1_nav2_ros2_ws/tools/hardware-preflight/h1_prepare_monitor`. The acceptance runner rejects monitor telemetry without schema 2, before authorizing output. `--fixstand-only` forbids the later services and requires a zero policy budget. Every `--execute` requires an explicitly selected finite positive `--fixstand-max-error` in radians; failure returns through Passive and revocation. No threshold is recommended here. The 2 rad/s cumulative velocity protection remains in effect. Dry-run is the default.

Execution also requires both `--confirm-debug-mode` and `--confirm-exclusive-control`. These flags are human assertions, not machine verification. Use them only after confirming the actual mode and exclusive control through the procedure applicable to the specific H1-2 and firmware; a keypress alone does not establish either condition. They default to false in dry-run.

Trace columns include q, dq, estimated torque and motor mode from the latest state and tau, mode, kp, kd, q and dq from the latest command, plus command mode_machine and mode_pr for each SDK index. They are asynchronous latest-value snapshots sampled about every 10 ms, not matched samples or a complete high-frequency recording. The state and command receipt timestamps, trace timestamp, and service request/response timestamps use the same Linux host's monotonic clock; service responses bound, but do not identify, the exact transition instant. DDS observation and an estimated torque do not prove ownership or physical execution. Confirm the node, monitor and trace are from the reviewed versions before interpreting records.

Run offline checks with `python3 tools/h1_2_policy_hardware/test_acceptance_offline.py`.

`get_fsm_readonly.cpp` is a separate read-only H1 loco GET_FSM_ID diagnostic. It sends one query on eth0/domain 0 with a 3-second RPC timeout, prints the return code and raw response before attempting to parse a successful response, and never publishes LowCmd or calls a mode-changing API. A successful query does not establish debug mode or exclusive control. Build it with the same command above, replacing `prepare_monitor.cpp` with `get_fsm_readonly.cpp` and the output path with an independent temporary filename. Building does not run the probe; review before any execution.
