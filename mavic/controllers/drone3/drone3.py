# ============================================================
#  DJI Mavic 2 Pro – DRONE3 (Right Side Follower)
# ============================================================

from controller import Robot, GPS, Gyro, InertialUnit, Motor, DistanceSensor
import struct, math

K_VERTICAL_THRUST = 68.5
K_VERTICAL_P = 3.0
K_VERTICAL_OFFSET = 0.6
K_ROLL_P = 50.0
K_PITCH_P = 30.0

SIDE_OFFSET = 2.0      # RIGHT of leader
SPEED_GAIN = 0.30

SAFE_DIST = 1.3
AVOID_FORCE = 2.0
MAX_AVOID = 2.0
SEP_DIST = 1.5
SEP_FORCE = 1.0

def clamp(v, lo, hi):
    return max(lo, min(v, hi))

robot = Robot()
timestep = int(robot.getBasicTimeStep())

imu = robot.getDevice("inertial unit"); imu.enable(timestep)
gps = robot.getDevice("gps"); gps.enable(timestep)
gyro = robot.getDevice("gyro"); gyro.enable(timestep)

ds_front = robot.getDevice("ds_front"); ds_front.enable(timestep)
ds_left  = robot.getDevice("ds_left");  ds_left.enable(timestep)
ds_right = robot.getDevice("ds_right"); ds_right.enable(timestep)
ds_back  = robot.getDevice("ds_back");  ds_back.enable(timestep)

receiver = robot.getDevice("receiver"); receiver.enable(timestep)

front_left_motor  = robot.getDevice("front left propeller")
front_right_motor = robot.getDevice("front right propeller")
rear_left_motor   = robot.getDevice("rear left propeller")
rear_right_motor  = robot.getDevice("rear right propeller")

for m in [front_left_motor, front_right_motor, rear_left_motor, rear_right_motor]:
    m.setPosition(float('inf'))
    m.setVelocity(1.0)

print("✅ Drone3 (Right Follower) running")

leader_pos = (0,0,0)
target_altitude = 1.0

def compute_avoidance():
    f, b = ds_front.getValue(), ds_back.getValue()
    l, r = ds_left.getValue(),  ds_right.getValue()
    ax = ay = 0.0

    if f < SAFE_DIST: ay -= AVOID_FORCE*(1-f/SAFE_DIST)
    if b < SAFE_DIST: ay += AVOID_FORCE*(1-b/SAFE_DIST)
    if l < SAFE_DIST: ax += AVOID_FORCE*(1-l/SAFE_DIST)
    if r < SAFE_DIST: ax -= AVOID_FORCE*(1-r/SAFE_DIST)

    return clamp(ax,-MAX_AVOID,MAX_AVOID), clamp(ay,-MAX_AVOID,MAX_AVOID)

def compute_separation(dx, dz):
    dist = math.sqrt(dx*dx + dz*dz)
    if 0 < dist < SEP_DIST:
        k = SEP_FORCE * (1 - dist/SEP_DIST)
        return -(dx/dist)*k, -(dz/dist)*k
    return 0,0

# ============================= LOOP =============================
while robot.step(timestep) != -1:

    if receiver.getQueueLength() > 0:
        leader_pos = struct.unpack("fff", receiver.getData())
        receiver.nextPacket()

    lx, ly, lz = leader_pos
    fx, fy, fz = gps.getValues()

    print(f"[DRONE3 POS] x={fx:.2f}, y={fy:.2f}, z={fz:.2f}")

    # RIGHT side formation (correct roll sign)
    target_x = lx + SIDE_OFFSET
    target_z = lz

    dx = target_x - fx
    dz = target_z - fz

    roll_disturb  = -dx * SPEED_GAIN    # IMPORTANT FIX
    pitch_disturb =  dz * SPEED_GAIN

    ax, ay = compute_avoidance()
    roll_disturb  += ax
    pitch_disturb -= ay

    sx, sz = compute_separation(dx, dz)
    roll_disturb  += sx
    pitch_disturb -= sz

    roll, pitch, yaw = imu.getRollPitchYaw()
    alt = gps.getValues()[2]
    roll_rate, pitch_rate, _ = gyro.getValues()

    roll_in = K_ROLL_P * roll + roll_rate + roll_disturb
    pitch_in = K_PITCH_P * pitch + pitch_rate + pitch_disturb
    alt_err = clamp(target_altitude - alt + K_VERTICAL_OFFSET, -1, 1)
    vertical = K_VERTICAL_P * (alt_err ** 3)

    fl = K_VERTICAL_THRUST + vertical - roll_in + pitch_in
    fr = K_VERTICAL_THRUST + vertical + roll_in + pitch_in
    rl = K_VERTICAL_THRUST + vertical - roll_in - pitch_in
    rr = K_VERTICAL_THRUST + vertical + roll_in - pitch_in

    front_left_motor.setVelocity(fl)
    front_right_motor.setVelocity(-fr)
    rear_left_motor.setVelocity(-rl)
    rear_right_motor.setVelocity(rr)
