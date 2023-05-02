from src import servo_controls

min_angle = 30
max_angle = 100


class MockedServo:
    angle = 0


def mocked_servo_init():
    return MockedServo()


def test_servo_rotate_to():
    servo, servo_data = servo_controls.init(
        mocked_servo_init, "test-rotate-to", 0, 20, min_angle, max_angle
    )

    # Verify initialization is between min and max
    assert servo_data['angle'] == 30
    assert servo.angle == 30

    # Verify rotate between min and max
    servo, servo_data = servo_controls.rotate_to(servo, servo_data, 90)
    assert servo_data['angle'] == 90
    assert servo.angle == 90

    # Verify rotate between min and max
    servo, servo_data = servo_controls.rotate_to(servo, servo_data, max_angle + 10)
    assert servo_data['angle'] == max_angle
    assert servo.angle == max_angle
    servo, servo_data = servo_controls.rotate_to(servo, servo_data, min_angle - 10)
    assert servo_data['angle'] == min_angle
    assert servo.angle == min_angle


def test_sweep_step():
    servo, servo_data = servo_controls.init(
        mocked_servo_init, "test-sweep-step", 0, 20, min_angle, max_angle
    )

    servo, servo_data = servo_controls.step(servo, servo_data)
    assert servo_data['angle'] == min_angle + servo_data['sweep_step_current']
    assert servo.angle == min_angle + servo_data['sweep_step_current']


def test_step_toward_target():
    servo, servo_data = servo_controls.init(
        mocked_servo_init, "test-step-toward-target", 0, 20, min_angle, max_angle, sweep_tracking_sleep=0
    )

    # Test doesn't move
    servo, servo_data = servo_controls.step_toward_target(servo, servo_data, 100, 100)
    assert servo_data['angle'] == min_angle
    assert servo.angle == min_angle

    # Test moves positively
    servo, servo_data = servo_controls.step_toward_target(servo, servo_data, 1000, 100)
    assert servo_data['angle'] == min_angle + servo_data['sweep_tracking_step']
    assert servo.angle == min_angle + servo_data['sweep_tracking_step']

    # Do it again to make sure angel is not bound to min_angle
    servo, servo_data = servo_controls.step_toward_target(servo, servo_data, 1000, 100)

    # Test moves negatively
    servo, servo_data = servo_controls.step_toward_target(servo, servo_data, 0, 100)
    assert servo_data['angle'] == min_angle + servo_data['sweep_tracking_step']
    assert servo.angle == min_angle + servo_data['sweep_tracking_step']

    # Test does not go beyond max angle
    for i in range(1, round((180 - servo.angle) / servo_data['sweep_tracking_step'])):
        servo, servo_data = servo_controls.step_toward_target(servo, servo_data, 1000, 100)

    assert servo_data['angle'] == max_angle
    assert servo.angle == max_angle

    # Test does not go beyond min angle
    for i in range(1, round(servo.angle / servo_data['sweep_tracking_step'])):
        servo, servo_data = servo_controls.step_toward_target(servo, servo_data, 0, 100)

    assert servo_data['angle'] == min_angle
    assert servo.angle == min_angle

