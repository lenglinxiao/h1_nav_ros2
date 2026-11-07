"""Evaluate cumulative SDK-order joint-speed peaks without controlling hardware."""
import math

SDK_JOINT_NAMES = (
    "left_hip_yaw", "left_hip_pitch", "left_hip_roll", "left_knee", "left_ankle_pitch", "left_ankle_roll",
    "right_hip_yaw", "right_hip_pitch", "right_hip_roll", "right_knee", "right_ankle_pitch", "right_ankle_roll",
    "torso", "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow",
    "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw", "right_shoulder_pitch", "right_shoulder_roll",
    "right_shoulder_yaw", "right_elbow", "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
)


def speed_violation(sample, leg_limit, pulse_started):
    peak = sample['peak_dq']
    if isinstance(peak, bool) or not isinstance(peak, (int, float)) or not math.isfinite(peak) or peak < 0:
        raise RuntimeError('Invalid cumulative peak velocity')
    peaks = sample.get('peak_dq_by_joint')
    if peaks is None and leg_limit == 2.0:
        return (dict(group='all', cumulative_peak_dq=peak, limit=2.0)
                if peak > 2.0 else None)
    if (not isinstance(peaks, list) or len(peaks) != 27 or
            any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                not math.isfinite(v) or v < 0 for v in peaks)):
        raise RuntimeError('Monitor requires 27 finite nonnegative cumulative joint speed peaks')
    if not math.isclose(peak, max(peaks), rel_tol=1e-6, abs_tol=1e-6):
        raise RuntimeError('Cumulative scalar and joint speed peaks disagree')
    for i, value in enumerate(peaks):
        limit = leg_limit if pulse_started and i < 12 else 2.0
        if value > limit:
            return dict(sdk_joint_index=i, joint_name=SDK_JOINT_NAMES[i], group='legs' if i < 12 else 'upper_body',
                        cumulative_peak_dq=value, limit=limit)
    return None
