# H1-2 policy hardware adaptation

## Scope and contract

This adaptation adds an explicitly selected, initially read-only hardware profile to the existing simulation policy bridge. Current authorized real-robot operation is limited to read-only LowState and shadow inference; no motor action or control transfer is authorized by the preparation described below. Do not infer successful physical operation from offline tests or shadow logs.

| Launch argument | Default | Hardware selection |
| --- | --- | --- |
| `profile` | `sim` | `hardware` |
| `network_interface` | `lo` | `eth0` |
| `dds_domain` | `42` | `0` |
| `enable_hardware_output` | `false` | remains false for read-only |
| `max_policy_steps` | `0` | remains 0 for read-only; hardware ArmHold is rejected |
| `operator_confirmed_debug_mode` | `false` | human confirmation, not machine evidence |
| `operator_confirmed_exclusive_control` | `false` | human confirmation, not machine evidence |
| `model_path` | `/home/llx/policy.onnx` | board model path must be explicit |
| `cmd_vel_topic` | `/cmd_vel_final` | unchanged |

Only the two complete profile/interface/domain combinations are valid. For read-only and shadow on the board, use `ROS_DOMAIN_ID=43` and `ROS_LOCALHOST_ONLY=1` to isolate ROS from the existing Web ROS domain 0. Unitree DDS independently uses `eth0`, domain 0; its domain is not `ROS_DOMAIN_ID`. The main navigation launch remains phase-1 simulation only; hardware is not integrated into Web or online navigation. ROS supports `rmw_fastrtps_cpp` only. The simulator's `mode_machine=0` and hardware's `mode_machine=6` are profile-specific machine identifiers; both require `mode_pr=0`. Neither machine identifier proves firmware ownership, release or debug readiness.

At 50 Hz, the policy forms a 92-element observation, runs ONNX for 27 actions, then computes `q_target = default + scale * action` and clipped explicit PD torque `kp * (q_target - q) - kd * dq`. LowCmd embeds zero kp/kd. Phase period is 0.6 s; phase observation is zero for command norm below 0.1 while internal phase advances. The supplied guide is the source for joint mapping, offsets, gains and limits. Wrist training order and real PD timing remain unverified; static numerical regression does not establish joint semantics or safe motion.

## Read-only and output lifecycle

Configuration parameters are read-only after startup. Runtime parameter changes
are rejected rather than silently disagreeing with the controller's cached
settings; use `~/revoke_output` to withdraw authorization.

Hardware state freshness is at most 100 ms, and a computed command additionally
requires its own input snapshot to be no older than one 20 ms control period
at publication. Receiving newer packets during a slow inference does not make
an old computed command eligible for publication.

On hardware startup no LowCmd publisher is created. `~/shadow` requires fresh, finite, profile-matching state with valid CRC and advancing ticks, and runs zero-command inference while output is inhibited. A fault stops shadow. See the [package README](../src/h1_2_policy_bridge/README.md) for read-only launch and service examples. These examples do not transfer control.

Existing FSM services are rejected until output is authorized. `~/authorize_output` checks the output enable setting, both operator confirmations and valid advancing state; on success it creates the publisher and begins Passive output. These booleans only record that a person has confirmed the official debug and exclusive-control procedures; software does not verify the firmware owner. `~/revoke_output` and faults inhibit output and destroy the publisher, but software cannot establish how motors respond to loss of the command stream. The node does not automatically call `ReleaseMode`. Hardware `max_policy_steps=0` rejects ArmHold. Only a positive budget explicitly configured after review of a specific future action permits entry. Each authorization permits one budget; exhaustion returns software to Passive damping and re-entry requires revoke and reauthorization. Faults revoke authorization and stop publication. A frame count does not guarantee a hard real-time duration or firmware behavior. A future output test requires a specific reviewed procedure, including control release, recovery and interruption behavior, before its parameters or service calls are used. Sling, emergency stop and personnel being ready do not constitute approval of a particular output test.

## Evidence and remaining verification

On 2026-11-01 an SDK-only read-only probe on `eth0`, domain 0 received 2499 LowState samples in 5 seconds (about 500 Hz), with `mode_machine=6`, `mode_pr=0`, and finite 27-joint q/dq and IMU values. Knees were 1.115991 and 1.118930 rad, outside the active ArmHold requirement `abs(q - 0.9) < 0.07`; do not attempt active takeover on that evidence. A read-only Loco `GetFsmId` returned 0 with FSM 0; it establishes neither debug mode nor released ownership. Web remained active at the check time.

The isolated simulation tests cover core arithmetic, static ONNX regression, simulated DDS behavior and manual FSM transitions. Hardware tests exercise profile and gate/tick logic without a robot; launch parameter evaluation uses a ROS `LaunchContext` without starting a node or network. These seams cannot verify actual ownership, physical mapping, motor response, release/recovery, timing, or safety of a real output. Validate those separately through a reviewed test procedure before any motor action.

After migration to the board main workspace, build this package alone with the board dependencies and the model path configured, for example:

```bash
cd /home/unitree/h1_nav2_ros2_ws
colcon build --packages-select h1_2_policy_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release -DORT_ROOT=/home/unitree/h1_nav2_ros2_ws/third_party/h1_policy_runtime/onnxruntime -DUNITREE_SDK2_ROOT=/home/unitree/h1_nav2_ros2_ws/third_party/h1_policy_runtime/unitree_sdk2 -DH1_POLICY_TEST_MODEL_PATH=/home/unitree/h1_nav2_ros2_ws/models/h1_2_armhold/policy.onnx
colcon test --packages-select h1_2_policy_bridge
colcon test-result --verbose
```

This documents a migration procedure, not a claim that this revision has been deployed or tested on the board. The previous board build and model regression passed, but its full CTest did not run because the model path was hard-coded. Record fresh build, test and read-only shadow evidence for this revision separately.
