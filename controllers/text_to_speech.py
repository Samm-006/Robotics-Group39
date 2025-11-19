"""audio_spot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import pyttsx3

# create the Robot instance.
robot = Robot()

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

last_feedback_obj= None
last_wall_warning= False

engine.say("Simulation started")

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    sensor_value = 42  # replace with your logic
    if sensor_value == 42:
        engine.say("Sensor value reached 42")
        engine.runAndWait()
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.


#code ADDED into yolo controller to send audio feedback when object + wall is detected 
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
                current_obj = ", ".join(detected_objects)
                
                if current_obj != last_feedback_obj:
                    tts_from_yolo(detected_objects)
                    last_feedback_obj = current_obj
                    last_wall_warning = False 

        # Step8 Wall Inspection
        # Reading distance
        distance = distance_sensor.getValue()
        # Debug
        #print("distance:", distance)

        # Distance less than 0.5 and YOLO fails to detect objects → Classified as a wall
        if distance < 0.5 and len(detected_objects) == 0:
            if not last_wall_warning:
                print("Wall ")
                engine.say("Wall ahead avoid hitting it")
                last_wall_warning = True
                last_feedback_obj = None
