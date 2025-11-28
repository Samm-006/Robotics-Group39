"""text_to_speech controller."""

from controller import Robot, Keyboard 
import pyttsx3 # For audio feedback
import random 
import time

# create the Robot instance.
robot = Robot()

# Obtain time step (64ms from the code)
timestep = 64 # int(robot.getBasicTimeStep())

left_motor= robot.getDevice("left wheel motor")
right_motor= robot.getDevice("right wheel motor")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

#Lidar
lidar = robot.getDevice('LDS-01')
lidar.enable(timestep)
lidar.enablePointCloud()

# Initialisation of keyboard for movement 
keyboard = Keyboard()
keyboard.enable(timestep)

# Initialisation TTS module
engine = pyttsx3.init()
engine.setProperty("rate",180)
engine.setProperty('volume', 0.9)


engine.say("Simulation started")

# Object avoidance Params
CRUISE_SPEED = 3.0   
TURN_SPEED = 0.5     
OBSTACLE_DIST = 0.8 
    
robot_state = 'FORWARD'
state_timer = 0

last_detected_obj = "None"
seen_objects = set()

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    '''    
    # Setup Arrows to move robot (↑, ↓, ←, →) 
    key=keyboard.getKey()
    
    left_wheel_speed = 0.0
    right_wheel_speed = 0.0
    speed = 2.5
    
    if key == Keyboard.UP:
        left_wheel_speed = speed
        right_wheel_speed = speed
        
    elif key == Keyboard.DOWN:
         left_wheel_speed = -speed
         right_wheel_speed = -speed
         
    elif key == Keyboard.RIGHT:
         left_wheel_speed = -speed
         right_wheel_speed = speed
         
    elif key == Keyboard.LEFT:
         left_wheel_speed = speed
         right_wheel_speed = -speed
    
    left_motor.setVelocity(left_wheel_speed)
    right_motor.setVelocity(right_wheel_speed)
'''
    left_wheel_speed = 0.0
    right_wheel_speed = 0.0
    speed = 2.5
    
    lidar_data = lidar.getRangeImage()
    if not lidar_data: 
        continue

    front_idx = len(lidar_data) // 2
    start_idx = max(0, front_idx - 40)
    end_idx = min(len(lidar_data), front_idx + 40)
    front_sector = lidar_data[start_idx : end_idx]
    valid_readings = [d for d in front_sector if d != float('inf')]
    min_front = min(valid_readings) if valid_readings else 10.0
   
    if robot_state == 'FORWARD':
        if min_front < OBSTACLE_DIST:
            robot_state = 'REVERSE'
            state_timer = 60 
        else:
            left_motor.setVelocity(CRUISE_SPEED)
            right_motor.setVelocity(CRUISE_SPEED)
    elif robot_state == 'REVERSE':
        if state_timer > 0:
            left_motor.setVelocity(-1.0)
            right_motor.setVelocity(-1.0)
            state_timer -= 1
        else:
            robot_state = 'TURN'
            state_timer = 60 
            direction = 1 if random.random() > 0.5 else -1
            current_turn_velocity = TURN_SPEED * direction
    elif robot_state == 'TURN':
        if state_timer > 0:
            left_motor.setVelocity(-current_turn_velocity)
            right_motor.setVelocity(current_turn_velocity)
            state_timer -= 1
        else:
            if min_front > OBSTACLE_DIST + 0.1:
                robot_state = 'FORWARD'
            else:
                state_timer = 30        
         
    

            # Step 7: Integrate with the TTS module 
            # Audio Feedback and Retrieving Information on Detected Objects
            boxes = r.boxes
            for box in boxes:
                cls_id = int(box.cls[0]) # Get category ID
                cls_name = model.names[cls_id] # Get category names
                
                # Trigger the TTS module
                # such: print(f"find: {cls_name}")
                # Then call: tts_module.warn(cls_name)
                 
                # Save last object detected
                last_detected_obj = cls_name 
                 
                # TTS Voice output for new object
                if float(box.conf[0]) > 0.5 and cls_name not in seen_objects:
                    engine.say(f"{cls_name} ahead avoid hitting it")
                    engine.runAndWait()
                    seen_objects.add(cls_name)
                    
# Step 8: Wall Inspection
        # Reading distance
        distance = distance_sensor.getValue()

        # Debug
        #print("distance:", distance)

        # Distance less than 0.5 and YOLO fails to detect objects → Classified as a wall
        if distance < 0.5 and len(detected_objects) == 0:
            print("Wall")
            engine.say("Wall ahead avoid hitting it")
            engine.runAndWait()
        
