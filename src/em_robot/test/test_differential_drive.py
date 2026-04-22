import math

from em_robot.differential_drive import PoseState, cmd_vel_to_motor_speeds, integrate_fake_motion


def test_cmd_vel_to_motor_speeds_clamps_output():
    right_speed, left_speed = cmd_vel_to_motor_speeds(
        linear_x=100.0,
        angular_z=10.0,
        max_speed=0.5,
    )

    assert -410 <= right_speed <= 410
    assert -410 <= left_speed <= 410


def test_integrate_fake_motion_updates_pose():
    pose_state = PoseState()

    result = integrate_fake_motion(
        linear_x=0.2,
        angular_z=0.0,
        dt=1.0,
        pose_state=pose_state,
        max_speed=1.0,
        max_pos_step=1.0,
    )

    assert math.isclose(result["pose"].x, 0.2, rel_tol=1e-6)
    assert math.isclose(result["pose"].y, 0.0, abs_tol=1e-9)
    assert math.isclose(result["vx"], 0.2, rel_tol=1e-6)
