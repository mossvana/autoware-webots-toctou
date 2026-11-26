import math

from controller import Keyboard, Robot


robot = Robot()
time_step = int(robot.getBasicTimeStep())

keyboard = Keyboard()
keyboard.enable(time_step)

front_left_motor = robot.getDevice("front_left_motor")
front_right_motor = robot.getDevice("front_right_motor")
rear_left_motor = robot.getDevice("rear_left_motor")
rear_right_motor = robot.getDevice("rear_right_motor")

front_left_steer = robot.getDevice("front_left_steer")
front_right_steer = robot.getDevice("front_right_steer")

front_distance = robot.getDevice("front_distance")
gps = robot.getDevice("gps")
imu = robot.getDevice("imu")
gyro = robot.getDevice("gyro")
accelerometer = robot.getDevice("accelerometer")

front_left_steer_sensor = robot.getDevice("front_left_steer_sensor")
front_right_steer_sensor = robot.getDevice("front_right_steer_sensor")

wheel_motors = [
    front_left_motor,
    front_right_motor,
    rear_left_motor,
    rear_right_motor,
]

for motor in wheel_motors:
    motor.setPosition(float("inf"))
    motor.setVelocity(0.0)

front_left_steer.setVelocity(4.0)
front_right_steer.setVelocity(4.0)
front_left_steer.setPosition(0.0)
front_right_steer.setPosition(0.0)

for sensor in [
    front_distance,
    gps,
    imu,
    gyro,
    accelerometer,
    front_left_steer_sensor,
    front_right_steer_sensor,
]:
    sensor.enable(time_step)

# Geometry copied from the original RC car.
WHEELBASE = 0.22
TRACK_WIDTH = 0.21

MAX_WHEEL_SPEED = 6.0
WHEEL_SPEED_STEP = 1.0

# Center/bicycle steering angle. Ackermann conversion produces separate
# left and right tire angles.
MAX_CENTER_STEER = 0.45
STEER_STEP = 0.05

wheel_speed = 0.0
center_steer = 0.0
gear = "P"


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def ackermann_angles(center_angle):
    if abs(center_angle) < 1.0e-6:
        return 0.0, 0.0

    radius = WHEELBASE / math.tan(center_angle)
    left_angle = math.atan(WHEELBASE / (radius - TRACK_WIDTH / 2.0))
    right_angle = math.atan(WHEELBASE / (radius + TRACK_WIDTH / 2.0))
    return left_angle, right_angle


print("RC car ready")
print("P/R/N/D: gear (gear changes are accepted only while stopped)")
print("UP/DOWN: speed, LEFT/RIGHT: steering, C: center, SPACE: stop")

counter = 0

while robot.step(time_step) != -1:
    key = keyboard.getKey()

    while key != -1:
        if key == ord("P") and wheel_speed == 0.0:
            gear = "P"
        elif key == ord("R") and wheel_speed == 0.0:
            gear = "R"
        elif key == ord("N") and wheel_speed == 0.0:
            gear = "N"
        elif key == ord("D") and wheel_speed == 0.0:
            gear = "D"
        elif key == Keyboard.UP and gear in ("D", "R"):
            wheel_speed = clamp(
                wheel_speed + WHEEL_SPEED_STEP,
                0.0,
                MAX_WHEEL_SPEED,
            )
        elif key == Keyboard.DOWN:
            wheel_speed = clamp(
                wheel_speed - WHEEL_SPEED_STEP,
                0.0,
                MAX_WHEEL_SPEED,
            )
        elif key == Keyboard.LEFT:
            center_steer = clamp(
                center_steer + STEER_STEP,
                -MAX_CENTER_STEER,
                MAX_CENTER_STEER,
            )
        elif key == Keyboard.RIGHT:
            center_steer = clamp(
                center_steer - STEER_STEP,
                -MAX_CENTER_STEER,
                MAX_CENTER_STEER,
            )
        elif key == ord("C"):
            center_steer = 0.0
        elif key == ord(" "):
            wheel_speed = 0.0

        key = keyboard.getKey()

    left_angle, right_angle = ackermann_angles(center_steer)
    front_left_steer.setPosition(left_angle)
    front_right_steer.setPosition(right_angle)

    if gear == "D":
        commanded_wheel_speed = wheel_speed
    elif gear == "R":
        commanded_wheel_speed = -wheel_speed
    else:
        commanded_wheel_speed = 0.0

    for motor in wheel_motors:
        if gear == "N":
            # Neutral: disconnect drive torque so the wheels can coast.
            motor.setAvailableTorque(0.0)
        else:
            # D/R drive normally; P holds the requested zero velocity.
            motor.setAvailableTorque(5.0)
        motor.setVelocity(commanded_wheel_speed)

    counter += 1
    if counter % 30 == 0:
        position = gps.getValues()
        roll, pitch, yaw = imu.getRollPitchYaw()
        measured_left_steer = front_left_steer_sensor.getValue()
        measured_right_steer = front_right_steer_sensor.getValue()
        print(
            f"gear={gear}, wheel={commanded_wheel_speed:+.1f} rad/s, "
            f"speed={gps.getSpeed():.3f} m/s, "
            f"center={center_steer:+.3f} rad, "
            f"steer=({measured_left_steer:+.3f}, {measured_right_steer:+.3f}), "
            f"xyz=({position[0]:+.2f}, {position[1]:+.2f}, {position[2]:+.2f}), "
            f"yaw={yaw:+.3f}, distance={front_distance.getValue():.3f}"
        )
