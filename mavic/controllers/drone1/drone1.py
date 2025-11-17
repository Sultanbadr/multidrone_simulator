# ============================================================
#  DJI Mavic 2 Pro – DRONE1 (LEADER)
# ============================================================

from controller import Robot, Camera, Compass, GPS, Gyro, InertialUnit, Keyboard, Motor, DistanceSensor
import struct, math

# PID constants
K_VERTICAL_THRUST = 68.5
K_VERTICAL_P = 3.0
K_VERTICAL_OFFSET = 0.6
K_ROLL_P = 50.0
K_PITCH_P = 30.0

SAFE_DIST = 1.3
AVOID_FORCE = 2.0
MAX_AVOID = 2.0

def clamp(v, lo, hi):
    return max(lo, min(v, hi))

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Devices
camera = robot.getDevice("camera"); camera.enable(timestep)
imu = robot.getDevice("inertial unit"); imu.enable(timestep)
gps = robot.getDevice("gps"); gps.enable(timestep)
gyro = robot.getDevice("gyro"); gyro.enable(timestep)
keyboard = Keyboard(); keyboard.enable(timestep)

ds_front = robot.getDevice("ds_front"); ds_front.enable(timestep)
ds_left  = robot.getDevice("ds_left");  ds_left.enable(timestep)
ds_right = robot.getDevice("ds_right"); ds_right.enable(timestep)
ds_back  = robot.getDevice("ds_back");  ds_back.enable(timestep)

front_left_motor  = robot.getDevice("front left propeller")
front_right_motor = robot.getDevice("front right propeller")
rear_left_motor   = robot.getDevice("rear left propeller")
rear_right_motor  = robot.getDevice("rear right propeller")

for m in [front_left_motor, front_right_motor, rear_left_motor, rear_right_motor]:
    m.setPosition(float('inf'))
    m.setVelocity(1.0)

emitter = robot.getDevice("emitter")

print("✅ Drone1 (Leader) running")

def compute_avoidance():
    f = ds_front.getValue()
    b = ds_back.getValue()
    l = ds_left.getValue()
    r = ds_right.getValue()

    ax = ay = 0.0

    if f < SAFE_DIST:
        ay -= AVOID_FORCE * (1 - f / SAFE_DIST)
    if b < SAFE_DIST:
        ay += AVOID_FORCE * (1 - b / SAFE_DIST)
    if l < SAFE_DIST:
        ax += AVOID_FORCE * (1 - l / SAFE_DIST)
    if r < SAFE_DIST:
        ax -= AVOID_FORCE * (1 - r / SAFE_DIST)

    return clamp(ax, -MAX_AVOID, MAX_AVOID), clamp(ay, -MAX_AVOID, MAX_AVOID)

target_altitude = 1.0

# ============================= LOOP =============================

while robot.step(timestep) != -1:

    roll, pitch, yaw = imu.getRollPitchYaw()
    alt = gps.getValues()[2]
    roll_rate, pitch_rate, _ = gyro.getValues()
    x, y, z = gps.getValues()

    # Send GPS to Drone2 & Drone3
    emitter.send(struct.pack("fff", x, y, z))
    print(f"[DRONE1 SEND] x={x:.2f}, y={y:.2f}, z={z:.2f}")

    roll_disturb = pitch_disturb = 0.0

    # Keyboard control
    key = keyboard.getKey()
    while key > 0:
        if key == Keyboard.UP:
            pitch_disturb = -2
        elif key == Keyboard.DOWN:
            pitch_disturb = 2
        elif key == Keyboard.RIGHT:
            roll_disturb = 1
        elif key == Keyboard.LEFT:
            roll_disturb = -1
        elif key == (Keyboard.SHIFT + Keyboard.UP):
            target_altitude += 0.05
        elif key == (Keyboard.SHIFT + Keyboard.DOWN):
            target_altitude -= 0.05
        key = keyboard.getKey()

    # Obstacle avoidance
    ax, ay = compute_avoidance()
    roll_disturb += ax
    pitch_disturb -= ay

    # PID
    roll_input = K_ROLL_P * roll + roll_rate + roll_disturb
    pitch_input = K_PITCH_P * pitch + pitch_rate + pitch_disturb

    alt_error = clamp(target_altitude - alt + K_VERTICAL_OFFSET, -1, 1)
    vertical = K_VERTICAL_P * (alt_error ** 3)

    # Mix motors
    fl = K_VERTICAL_THRUST + vertical - roll_input + pitch_input
    fr = K_VERTICAL_THRUST + vertical + roll_input + pitch_input
    rl = K_VERTICAL_THRUST + vertical - roll_input - pitch_input
    rr = K_VERTICAL_THRUST + vertical + roll_input - pitch_input

    front_left_motor.setVelocity(fl)
    front_right_motor.setVelocity(-fr)
    rear_left_motor.setVelocity(-rl)
    rear_right_motor.setVelocity(rr)
