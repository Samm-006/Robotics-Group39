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

speak("Simulation started")

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
