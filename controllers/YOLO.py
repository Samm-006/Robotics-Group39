
# Import the libraries required for Webots and YOLO
from controller import Robot, Keyboard
import numpy as np  # For image conversion
import cv2          # For image processing and display
from ultralytics import YOLO # YOLO
import math
import time
# Step 1: Initialisataion 

# Create a robot instance
robot = Robot()

# Obtain time step (64ms from the code)
timestep = 64 # int(robot.getBasicTimeStep())


#Motors
left_motor= robot.getDevice("left wheel motor")
right_motor= robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

#Encoders 
left_encoder= robot.getDevice("left wheel sensor")
right_encoder= robot.getDevice("right wheel sensor")
left_encoder.enable(timestep)
right_encoder.enable(timestep)


#IR Sensors
ir_names = ["ds0", "ds1", "ds2", "ds3", "ds4", "ds5", "ds6", "ds7"]
ir_sensors = []
for name in ir_names:
    sensor = robot.getDevice(name)
    if sensor:
        sensor.enable(timestep)
    ir_sensors.append(sensor)
 

#Keyboard 
keyboard = Keyboard()
keyboard.enable(timestep)


# Initialise the camera
# Robot/ children/ camera
camera = robot.getDevice("camera")
camera.enable(timestep)
# Obtain the camera's width and height (for image transformation)
width = camera.getWidth()
height = camera.getHeight()

# Initialise the distance
# Robot/ children/ distancesensor
distance_sensor = robot.getDevice("distance sensor")
distance_sensor.enable(timestep)


# Step 2: Load the YOLO model 
# 'yolov8n.pt' It is a lightweight, fast pre-trained model that can subsequently be used with other models (yolov8s.pt, yolov8m.pt) Improve accuracy
print("Loading YOLOv8n model...")
model = YOLO('yolov8n.pt') 
print("Model loaded。")

# Create a resizable window
WINDOW_NAME = "Webots Camera - YOLOv8 Detections"
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

#SLAM params
R = 0.021
L = 0.052

RES= 0.02
SIZE= 4.0
N = int(SIZE / RES)
grid= np.zeros((N, N), dtype=float)


x = 0.0
y = 0.0
theta = 0.0  
prevL = 0.0
prevR= 0.0

IR_ANGLES = [+0.5, -0.5, 0.0, 3.14159, +1.57, -1.57, 0.0, 0.0]

def w2m(px, py):
    mx = int((px+ SIZE/2.0)/RES)
    my = int((py+ SIZE/2.0)/RES)
    if 0 <= mx < N and 0 <= my < N :
        return mx,my
    return None
    
def ir_dist(v):
    if v < 80.0: 
        return None
    d = 5.0/(v - 60.0)
    d = max(0.02, min (0.3, d))
    return d 

# Step 3: Main Loop
while robot.step(timestep) != -1:
    #KEYBOARD MOVEMENT
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
    
    #SLAM ODOMETRY 
    print("Running SLAM :" , x, y, theta)
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue() 
    Ls = left_encoder.getValue()
    Rs = right_encoder.getValue()
    
    #to avoid bad encoders 
    if math.isnan(Ls) or math.isnan(Rs):
        continue
        
    dL= (Ls - prevL)* R
    dR= (Rs - prevR)* R
    prevL, prevR = Ls, Rs
        
    dC = (dL + dR)/2.0
    dT = (dR - dL)/L
        
    x += dC * math.cos(theta+ dT/2.0)
    y += dC * math.sin (theta+ dT/2.0)
    theta = (theta + dT +math.pi)%(2.0*math.pi) - math.pi
     
    
    #Mapping
    cell = w2m(x, y)
    if cell: 
        grid [cell[0]][cell[1]]= 0
     
    for i, s in enumerate(ir_sensors):
        if s is None:
            continue
         
        d= ir_dist (s.getValue())
        if d is None:
            continue
       
        ang = theta + IR_ANGLES[i]    
        hx= x + d * math.cos(ang)
        hy= y + d * math.sin(ang)
            
        c = w2m(hx, hy)
        if c:
            grid [c[0]][c[1]]= 1
            
    
    # Step 4: Acquire and convert the image
    # 1. Retrieve raw image data from Webots (of type bytes)
    img_bytes = camera.getImage()
    
    if img_bytes:
        # 2. Convert bytes to a NumPy array
        # Webots Cameras typically provide the BGRA format.
        image = np.frombuffer(img_bytes, dtype=np.uint8).reshape((height, width, 4))
        
        # 3. Convert BGRA to BGR 
        image_bgr = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

        #Step 5: Run YOLO inference
        
        # Pass the BGR image to the model
        # 'stream=True' Applicable to video streams
        results = model(image_bgr, stream=True, verbose=False)

        # Step 6: Process and display the results 
        
        annotated_frame = image_bgr.copy() # Copy a frame for drawing

        detected_objects = set() # Use sets to avoid printing the same object multiple times within a single frame.
        
        for r in results:
            # Use YOLO's built-in drawing tools to annotate images.
            annotated_frame = r.plot()
            
            # Check all the checkboxes
            boxes = r.boxes
            for box in boxes:
                cls_id = int(box.cls[0])           # Get category ID
                confidence = box.conf[0].item()    # Obtain Confidence
                
                # Get category name (such as "chair", "person")
                cls_name = model.names[cls_id] 
                
                # Only add objects with high confidence to the set.
                if confidence > 0.5: # Only process detections with a confidence level greater than 50%.
                    detected_objects.add(cls_name)

        #  print objects
        if detected_objects:

            output_message = ", ".join(detected_objects)
            print(f"Obstacle detected: {output_message} ")
           
            
            # Step 7: Integrate with the TTS module 
            # Audio Feedback and Retrieving Information on Detected Objects
            boxes = r.boxes
            for box in boxes:
                cls_id = int(box.cls[0]) # Get category ID
                cls_name = model.names[cls_id] # Get category names
                
                # Trigger the TTS module
                # such: print(f"find: {cls_name}")
                # Then call: tts_module.warn(cls_name)
        
        # Step8 Wall Inspection
       

        # Reading distance
        distance = distance_sensor.getValue()

        # Debug
        #print("distance:", distance)

        # Distance less than 0.5 and YOLO fails to detect objects → Classified as a wall
        if distance < 0.5 and len(detected_objects) == 0:
            print("Wall ")
        
                
        # Displaying images with detection boxes using OpenCV
        cv2.imshow("Webots Camera - YOLOv8 Detections", annotated_frame)
        cv2.waitKey(1)

    else:
        print("Unable to obtain camera images")

# End of cycle
cv2.destroyAllWindows()
