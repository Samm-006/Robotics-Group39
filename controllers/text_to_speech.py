"""text_to_speech controller."""

from controller import Robot, Keyboard 
import pyttsx3 # For audio feedback

# create the Robot instance.
robot = Robot()

# Obtain time step (64ms from the code)
timestep = 64 # int(robot.getBasicTimeStep())

left_motor= robot.getDevice("left wheel motor")
right_motor= robot.getDevice("right wheel motor")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))


# Initialisation of keyboard for movement 
keyboard = Keyboard()
keyboard.enable(timestep)

# Initialisation TTS module
engine = pyttsx3.init()
engine.setProperty("rate",180)
engine.setProperty('volume', 0.9)

def speak(audio):
    engine.say(audio)
    engine.runAndWait()
   
   
def tts_from_yolo(detected_obj):
    if not detected_obj:
        return
        
    label = ", ".join(detected_obj)
    feedback = f"{label} ahead, avoid hitting it."
    speak(feedback)


engine.say("Simulation started")


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    
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
    
    

            # Step 7: Integrate with the TTS module 
            # Audio Feedback and Retrieving Information on Detected Objects
            boxes = r.boxes
            for box in boxes:
                cls_id = int(box.cls[0]) # Get category ID
                cls_name = model.names[cls_id] # Get category names
                
                # Trigger the TTS module
                # such: print(f"find: {cls_name}")
                # Then call: tts_module.warn(cls_name)
                # If statment used to not allow overlapping + repatitive warning
                tts_from_yolo(detected_objects)

    # Step8 Wall Inspection
        # Reading distance
        distance = distance_sensor.getValue()

        # Debug
        #print("distance:", distance)

        # Distance less than 0.5 and YOLO fails to detect objects → Classified as a wall
        if distance < 0.5 and len(detected_objects) == 0:
           print("Wall ")
           engine.say("Wall ahead avoid hitting it")
         
         
