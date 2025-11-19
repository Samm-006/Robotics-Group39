
# Import the libraries required for Webots and YOLO
from controller import Robot
import numpy as np  # For image conversion
import cv2          # For image processing and display
from ultralytics import YOLO # YOLO

# Step 1: Initialisation 

# Create a robot instance
robot = Robot()

# Obtain time step (64ms from the code)
timestep = 64 # int(robot.getBasicTimeStep())

# Initialise the camera
# DEF HEAD_SHAPES Pose/ children/ camera
camera = robot.getDevice("camera")
camera.enable(timestep)
# Initialise the distance
# DEF HEAD_SHAPES Pose/ children/ distancesensor
distance_sensor = robot.getDevice("distance_sensor")
distance_sensor.enable(timestep)

# Obtain the camera's width and height (for image transformation)
width = camera.getWidth()
height = camera.getHeight()

# Step 2: Load the YOLO model 
# 'yolov8n.pt' It is a lightweight, fast pre-trained model that can subsequently be used with other models (yolov8s.pt, yolov8m.pt) Improve accuracy
print("Loading YOLOv8n model...")
model = YOLO('yolov8n.pt') 
print("Model loaded。")

# Create a resizable window
WINDOW_NAME = "Webots Camera - YOLOv8 Detections"
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)


# Step 3: Main Loop
while robot.step(timestep) != -1:
    
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
