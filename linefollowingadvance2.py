from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0)
right_motor.setVelocity(0)

lidar = robot.getDevice('lidar')
lidar.enable(timestep)
lidar.enablePointCloud()  

# IR SENSORS
ir_sensors = []
for i in range(5):
    s = robot.getDevice(f'ps{i}')
    s.enable(timestep)
    ir_sensors.append(s)

# Threshold
threshold = 210.0

# PID
kp = 0.75
kd = 0.3
last_error = 0.0
base_speed = 3.0

# Weights for 5 sensors
weights = [-2, -1, 0, 1, 2]

# STATES
STATE_FOLLOW = 0
STATE_TURN_LEFT = 1
STATE_TURN_RIGHT = 2
STATE_U_TURN = 3
STATE_OBJECT_AHEAD = 4

state = STATE_FOLLOW


# ------------------------ 90° TURN DETECTOR -----------------------
def detect_90_bend(binary):
    # 90° LEFT bend patterns
    left_patterns = [
        [0, 0, 1, 1, 1],
        [0, 1, 1, 1, 1]
    ]

    # 90° RIGHT bend patterns
    right_patterns = [
        [1, 1, 1, 0, 0],
        [1, 1, 1, 1, 0]
    ]

    if binary in left_patterns:
        return "LEFT"

    if binary in right_patterns:
        return "RIGHT"

    return None

def detact_object_ahead(front):
    if front < 0.08:
        return True
    return False


# ----------------------- MAIN LOOP ---------------------------------
while robot.step(timestep) != -1:

    lidar_data = lidar.getRangeImage()
    print("LIDAR Ranges:", lidar_data[0:10])  # Print first 5 LIDAR ranges for debugging

    n = len(lidar_data)

    lidar_data = lidar.getRangeImage()
    n = len(lidar_data)

    # FRONT sector
    front = sum(lidar_data[n//2 - 5 : n//2 + 5]) / 10

    # RIGHT sector (index 0 → 100)
    right = sum(lidar_data[0 : 10]) / 10

    # LEFT sector  (last 100 samples)
    left = sum(lidar_data[n-10 : n]) / 10

    # print("front:", front)
    # print("left:", left)
    # print("right:", right)

    # Read IR sensors
    values = [s.getValue() for s in ir_sensors]
    binary = [1 if v > threshold else 0 for v in values]

    # Detect turns
    turn = detect_90_bend(binary)

    # PID error using binary
    error = sum(w * b for w, b in zip(weights, binary))

    # Derivative
    derivative = error - last_error
    last_error = error

    # PID output
    correction = (kp * error) + (kd * derivative)

    # ---------------- STATE MACHINE ----------------


    
            

    
    # FOLLOWING LINE
    if state == STATE_FOLLOW:

        if detact_object_ahead(front):
            print("Obstacle detected ahead! Stopping.")
            state = STATE_OBJECT_AHEAD

        if turn == "LEFT":
            print("Detected 90° LEFT — Turning...")
            state = STATE_TURN_LEFT
            continue

        if turn == "RIGHT":
            print("Detected 90° RIGHT — Turning...")
            state = STATE_TURN_RIGHT
            continue

        if sum(binary) == 0:  # No sensors see line → U turn
            print("U-Turn Detected")
            state = STATE_U_TURN
            continue

            # Normal PID Line Following
        left_speed = base_speed - correction
        right_speed = base_speed + correction

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)


        # TURN LEFT (90°)
    elif state == STATE_TURN_LEFT:
        left_motor.setVelocity(-1.5)
        right_motor.setVelocity(4.0)

            # Center sensor sees line → done turn
        if binary[2] == 1:
            print("Finished LEFT turn — Resume")
            state = STATE_FOLLOW


        # TURN RIGHT (90°)
    elif state == STATE_TURN_RIGHT:
        left_motor.setVelocity(4.0)
        right_motor.setVelocity(-1.5)

        if binary[2] == 1:
            print("Finished RIGHT turn — Resume")
            state = STATE_FOLLOW


        # U-TURN
    elif state == STATE_U_TURN:
        left_motor.setVelocity(-1.5)
        right_motor.setVelocity(4.0)

        if binary[2] == 1:
            print("Finished U-TURN — Resume")
            state = STATE_FOLLOW
            

    elif state == STATE_OBJECT_AHEAD:
    
        if right < 0.055:
            
            
            if binary[2] == 1:
                print(error)
                print("object following finish")
                
                
            else:
                left_motor.setVelocity(3.0)
                right_motor.setVelocity(1.0)
                
         
    
        elif right < 0.06:
            print("Path clear on the RIGHT — Turning RIGHT")
            left_motor.setVelocity(1.0)
            right_motor.setVelocity(3.0)
            
            if binary[2] == 1:
                print(error)
                print("object following finish")
                left_motor.setVelocity(6.0)
                right_motor.setVelocity(1.0)
                state = STATE_FOLLOW
                
           
                
            
   
        elif right > 0.2 :
            print("Path clear on the LEFT — Turning LEFT")
            left_motor.setVelocity(5.0)
            right_motor.setVelocity(1.0)
            
           
            
    