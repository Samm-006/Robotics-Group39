# Import the libraries required for Webots and YOLO
from controller import Robot
import numpy as np  # For image conversion
import cv2          # For image processing and display
from ultralytics import YOLO # YOLO
import math
import time
import random 
import pyttsx3 # For audio feedback
import copy
import heapdict #For priority queue



# Step 1: Initialisataion 
# Create a robot instance
robot = Robot()

# Obtain time step (64ms from the code)
timestep = 64 # int(robot.getBasicTimeStep())

# Initialisation TTS module
engine = pyttsx3.init()
engine.setProperty("rate",180)
engine.setProperty('volume', 0.9)

# Speak "Moving" every 400 steps (approx 12 seconds)
TTS_INTERVAL = 400 
last_tts_time = 0 
tts_cooldown = 10

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

#Lidar
lidar = robot.getDevice('LDS-01')
lidar.enable(timestep)
lidar.enablePointCloud()

# Step 2: Load the YOLO model 
# 'yolov8n.pt' It is a lightweight, fast pre-trained model that can subsequently be used with other models (yolov8s.pt, yolov8m.pt) Improve accuracy
print("Loading YOLOv8n model...")
model = YOLO('yolov8n.pt') 
print("Model loaded。")

# Create a resizable window
WINDOW_NAME = "Webots Camera - YOLOv8 Detections"
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

#SLAM params
R = 0.019
L = 0.09

RES= 0.02
SIZE= 20
N = int(SIZE / RES)
grid= np.zeros((N, N), dtype=float)


x = 0.0
y = 0.0
theta = 0.0  
prevL = 0.0
prevR= 0.0

step_count = 0

IR_ANGLES = [+0.5, -0.5, 0.0, 3.14159, +1.57, -1.57, 0.0, 0.0]


# Object avoidance Params
CRUISE_SPEED = 6.28   
TURN_SPEED = 0.5    
OBSTACLE_DIST = 0.3 
    
robot_state = 'FORWARD'
state_timer = 0

def w2m(px, py):
    mx = int((px+ SIZE/2.0)/RES)
    my = int((SIZE/2.0-py)/RES) 
    if 0 <= mx < N and 0 <= my < N :
        return (my,mx)
    return None

      
def ir_dist(v):
    if v < 2.0: 
        return None
    d = 5.0/(v - 60.0)
    d = max(0.02, min (0.3, d))

    return d 
    


state="SLAM"
    

#A STAR parameters
order=[]
initial_turn=True
initial_forward=True


# Step 3: Main Loop
while robot.step(timestep) != -1:
    if state=="SLAM":
                
        left_wheel_speed = 0.0
        right_wheel_speed = 0.0
        speed = 2.5
        
        # Object avoidance using LIDAR (stop → reverse → turn → continue)
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
            if step_count % TTS_INTERVAL == 0:
                engine.say("Moving forward")
                engine.runAndWait()
    
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
            # To Say "Turning" once before starting turn ---
                print("Status: Turning...")
                engine.say("Turning now")
                engine.runAndWait()
                    
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
               
        #SLAM ODOMETRY 
        print("Running SLAM :" , x, y, theta)
        # Read the sensors:
        #  val = ds.getValue() 
        Ls= left_encoder.getValue()
        Rs= right_encoder.getValue()
    
        dL= (Ls - prevL)* R
        dR= (Rs - prevR)* R
        prevL, prevR = Ls, Rs
            
        dC = (dL + dR)/2.0
        dT = (dR - dL)/L
            
        x += dC * math.cos(theta+ dT/2.0)
        y += dC * math.sin (theta+ dT/2.0)
        theta = (theta + dT +math.pi)%(2.0*math.pi) - math.pi
    
        
        print("Running SLAM :", round(x, 3), round(y, 3), round(theta, 3))
        
        # Read the sensors:
        cell = w2m(x, y)
        if cell: 
            grid [cell[0]][cell[1]] -= 0.05
            
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
                grid [c[0]][c[1]]+= 0.2 
              
              
        grid = np.clip(grid, -5.0, 5.0)
        
        step_count += 1
        
        #binary for A* (1= free space, 0= obstacle)
        binary_grid = (grid > 0.1).astype(int)
        
        binary_grid = binary_grid.tolist()
        
        #Threshold to terminate
        
        count=0
        total=N*N
        for i in range(len(binary_grid)):
            for j in range(len(binary_grid)):
                if binary_grid[i][j]==1:
                    count=count+1
        if count/total>0.0001:
           
            state="A_STAR"
        
        print("Percentage of 1's:" ,100*count/total) 
        
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
                     
                    # Save last object detected
                    last_detected_obj = cls_name 
                    current_time = time.time()
                     
                    # TTS Voice output for new object
                    if float(box.conf[0]) > 0.5:
                        if current_time - last_tts_time > tts_cooldown:
                            engine.say(f"{cls_name} ahead avoid hitting it")
                            engine.runAndWait()
                            last_tts_time = current_time
                
            
            # Step 8: Wall Inspection
            # Reading distance
            distance = distance_sensor.getValue()
        
                # Debug
                #print("distance:", distance)
        
            current_time = time.time()
            # Distance less than 0.5 and YOLO fails to detect objects → Classified as a wall
            if distance < 0.5 and len(detected_objects) == 0:
                print("Wall")
                if current_time - last_tts_time > tts_cooldown:
                    engine.say("Wall ahead avoid hitting it")
                    engine.runAndWait()
                    last_tts_time = current_time
        
                      
            # Displaying images with detection boxes using OpenCV
            cv2.imshow("Webots Camera - YOLOv8 Detections", annotated_frame)
            cv2.waitKey(1)
    
        else:
            print("Unable to obtain camera images")
    elif state=="A_STAR":
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        #Get position of robot relative to the grid
        start=w2m(x,y)
        start=list(start)
        print("Start cell: ",start)
        grid=binary_grid
        def create_goal():
            #Get goal by picking a random part of the grid which is 1
            picked=True
            end=[]
            while picked==True:
                rand1=random.randint(0, N-1)
                rand2=random.randint(0, N-1)
                if grid[rand1][rand2]==1:
                    end.append(rand1)
                    end.append(rand2)
                    picked=False 
            return end
            
        #Initialise end node
        end=create_goal()
        print("End cell: ",end)
        
        def hc(grid,start,end):
            #Using manhattan distance as heuristic h(n) as only 4 directions
            heuristic=copy.deepcopy(grid)
            for i in range(len(grid)):
                for j in range(len(grid)):
                        heuristic[i][j]=abs(j-end[1])+abs(i-end[0])
            return heuristic
        
        
        def test1():
            #Test grid 1 where there exists multiple paths from start to end
            grid1 = [
            [1, 1, 1, 1, 1, 1, 1, 1, 1],
            [1, 0, 0, 0, 0, 0, 0, 1, 1],
            [1, 1, 1, 1, 1, 1, 1, 1, 1],
            [1, 0, 1, 0, 1, 0, 1, 0, 1],
            [1, 0, 1, 0, 1, 0, 1, 0, 1],
            [1, 0, 1, 0, 1, 0, 1, 0, 1],
            [1, 1, 1, 1, 0, 1, 1, 1, 1],
            [1, 1, 0, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 1, 1]]
            return grid1
        
        def test2():
            #Test grid 2 where there exists one path from start to end
            grid2 = [
            [1, 1, 1, 1, 1, 1, 1, 1, 1],
            [0, 0, 0, 0, 0, 0, 0, 1, 1],
            [1, 0, 1, 1, 1, 1, 1, 0, 1],
            [1, 0, 1, 0, 1, 0, 1, 0, 1],
            [1, 0, 1, 0, 1, 0, 1, 0, 1],
            [1, 0, 1, 0, 1, 0, 1, 0, 1],
            [1, 1, 1, 1, 0, 1, 1, 1, 1],
            [1, 1, 0, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 1, 1, 1, 0, 1, 1]]
            return grid2
            
        def test3():
            #Test grid 3 where there exists no paths from start to end
            grid3 = [
            [1, 1, 1, 1, 1, 1, 1, 1, 1],
            [0, 0, 0, 0, 0, 0, 0, 1, 1],
            [1, 0, 1, 1, 1, 1, 1, 0, 1],
            [1, 0, 1, 0, 1, 0, 1, 0, 1],
            [1, 0, 1, 0, 1, 0, 1, 0, 1],
            [1, 0, 1, 0, 1, 0, 1, 0, 1],
            [1, 1, 1, 1, 0, 1, 1, 0, 1],
            [1, 1, 0, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 1, 1, 1, 0, 1, 1]]
            return grid3
        
        
        #Uncomment to test a grid    
        #grid=test1()
        #grid=test2()
        #grid=test3()
        
        #Uncomment to test the grid with the Robot's position and goal 
        #start=[1,7]
        #end=[7,1]
        
        
        heuristic=hc(grid,start,end)
        
        
        
        def A_star_search(grid,start,end,heuristic,theta):
            #Main A* algorithm
            
            #Initialises visited nodes,step cost, gn, current node, fn and path connection for two nodes
            visited=[]
            step=1
            gn={tuple(start):0}
            current=start.copy()
            fn=heapdict.heapdict()
            path={}
            while end not in visited:
                #Until a path is created, checks if neighbouring nodes is available to search and not already visited
                #For each neighbouring node possible update gn value, fn value and path connection between two nodes
                
                #Looks at node below
                if (current[0]+1)<len(grid):
                    if grid[current[0]+1][current[1]]==1 and ([current[0]+1,current[1]] not in visited):
                        if (current[0]+1,current[1]) in gn:
                            if gn[(current[0]+1,current[1])]>gn[(current[0],current[1])]+step:
                                gn[(current[0]+1,current[1])]=gn[(current[0],current[1])]+step
                        else:
                             gn[(current[0]+1,current[1])]=gn[(current[0],current[1])]+step
                        fn[(current[0]+1,current[1])]=(gn[(current[0]+1,current[1])]+heuristic[current[0]+1][current[1]])
                        path[(current[0]+1,current[1])] = (current[0], current[1])
                #Looks at node right
                if (current[1]+1)<len(grid):
                    if grid[current[0]][current[1]+1]==1 and ([current[0],current[1]+1] not in visited):
                        if (current[0],current[1]+1) in gn:
                            if gn[(current[0],current[1]+1)]>gn[(current[0],current[1])]+step:
                                gn[(current[0],current[1]+1)]=gn[(current[0],current[1])]+step
                        else:
                            gn[(current[0],current[1]+1)]=gn[(current[0],current[1])]+step
                        fn[(current[0],current[1]+1)]=(gn[(current[0],current[1]+1)]+heuristic[current[0]][current[1]+1])
                        path[(current[0],current[1]+1)] = (current[0], current[1])
                #Looks at node above
                if (current[0]-1)>=0:
                    if grid[current[0]-1][current[1]]==1 and ([current[0]-1,current[1]] not in visited):
                        if (current[0]-1,current[1]) in gn:
                            if gn[(current[0]-1,current[1])]>gn[(current[0],current[1])]+step:
                                gn[(current[0]-1,current[1])]=gn[(current[0],current[1])]+step
                        else: 
                            gn[(current[0]-1,current[1])]=gn[(current[0],current[1])]+step
                        fn[(current[0]-1,current[1])]=(gn[(current[0]-1,current[1])]+heuristic[current[0]-1][current[1]])
                        path[(current[0]-1,current[1])] = (current[0], current[1])      
                #Looks at node left
                if (current[1]-1)>=0:
                    if grid[current[0]][current[1]-1]==1 and ([current[0],current[1]-1] not in visited):
                        if (current[0],current[1]-1) in gn:            
                            if gn[(current[0],current[1]-1)]>gn[(current[0],current[1])]+step:
                                gn[(current[0],current[1]-1)]=gn[(current[0],current[1])]+step
                        else: 
                            gn[(current[0],current[1]-1)]=gn[(current[0],current[1])]+step
                        fn[(current[0],current[1]-1)]=(gn[(current[0],current[1]-1)]+heuristic[current[0]][current[1]-1])
                        path[(current[0],current[1]-1)] = (current[0], current[1])
               
                
                #Stops code if no possible path exists
                if len(fn)==0:
                    print("No path exists")
                    exit()
                
                #Finds next node to expand  
                   
                values=fn.popitem()
                
                visited.append(current.copy()) 
                
        
                current=list(values[0])
                
             
            #Uses dictionary to trace from the end node to start node to create the full path    
            full_path=[]
            current=end
            full_path.append(tuple(current))
            while (current[0],current[1]) != (start[0],start[1]):
                current=path[(current[0],current[1])]
                full_path.append(current)
                
            full_path.reverse()
            print("Order of nodes:", full_path)
            
            #Using the full path, prints the directions to go
            
            order=[]
            i=0
            for i in range(len(full_path)-1):
                before=full_path[i]
                after=full_path[i+1]
                if ((before[0]-after[0])==-1)and((before[1]-after[1])==0):
                    order.append("down")
                if ((before[0]-after[0])==1)and((before[1]-after[1])==0):
                    order.append("up")            
                if ((before[0]-after[0])==0)and((before[1]-after[1])==-1):
                    order.append("right")
                if ((before[0]-after[0])==0)and((before[1]-after[1])==1):
                    order.append("left")
                    
            print("Directions to end node:",order)
            
            return order
            
        order=A_star_search(grid,start,end,heuristic,theta) 
        #Through trial and error
        angle=theta+0.876
        state="TURN"
        j=0
        #seen_objects=set()
    elif (state=="TURN" or state=="FORWARD"):
        #Use YOLO and TTS while robot is moving the path
        
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
                     
                    # Save last object detected
                    last_detected_obj = cls_name 
                    current_time = time.time()
                     
                    # TTS Voice output for new object
                    if float(box.conf[0]) > 0.5:
                        if current_time - last_tts_time > tts_cooldown:
                            engine.say(f"{cls_name} ahead avoid hitting it")
                            engine.runAndWait()
                            last_tts_time = current_time
                            
            # Step 8: Wall Inspection
            # Reading distance
            distance = distance_sensor.getValue()
        
                # Debug
                #print("distance:", distance)
        
            current_time = time.time()
            # Distance less than 0.5 and YOLO fails to detect objects → Classified as a wall
            if distance < 0.5 and len(detected_objects) == 0:
                print("Wall")
                if current_time - last_tts_time > tts_cooldown:
                    engine.say("Wall ahead avoid hitting it")
                    engine.runAndWait()
                    last_tts_time = current_time          
                            
            # Displaying images with detection boxes using OpenCV
            cv2.imshow("Webots Camera - YOLOv8 Detections", annotated_frame)
            cv2.waitKey(1)
    
        else:
            print("Unable to obtain camera images")
        
        
        if state=="TURN":
               
        #To move the robot to the correct orientation    
            if len(order)>j:
                #To get the correct angle orientation
                if order[j]=="right":
                    direction=0
                if order[j]=="left":
                    direction=math.pi      
                if order[j]=="up":
                    direction=((math.pi)/2)
                if order[j]=="down":
                    direction=-((math.pi)/2)
     
                
                if initial_turn==True:
                    #Initialise the angle that the robot would turn, rotation angle and encoders
                    
                    angle=direction-angle
                    
                    #to get in -pi to pi using website https://stackoverflow.com/questions/27093704/converge-values-to-range-pi-pi-in-matlab-not-using-wraptopi
                    angle=angle - 2*math.pi*math.floor( (angle+math.pi)/(2*math.pi) );
                    
                    
                    before_l=left_encoder.getValue()
                    before_r=right_encoder.getValue()
                    
                    
                    #Based on expanding the Proto node
                    wheel_radius= 0.019
                    axle_length= 0.045+0.045
                    
                    #Based on the equation from website https://www.roboticsbook.org/S52_diffdrive_actions.html
                   
                    rotation=(axle_length*abs(angle))/(2*wheel_radius)
                    
                    #Move the robot on the spot left or right depending on angle
                    
                    if angle<0:
                        left_motor.setVelocity(1)
                        right_motor.setVelocity(-1)
                    else:
                        left_motor.setVelocity(-1)
                        right_motor.setVelocity(1) 
                    
                    
                        
                    initial_turn=False
                  
                #Keeps on turning until the average encoder value reach the rotation angle and switch state to forward  
                average_turn=(abs(left_encoder.getValue()-before_l)+abs(right_encoder.getValue()-before_r))/2  
                if average_turn>=rotation:
                    left_motor.setVelocity(0.0)
                    right_motor.setVelocity(0.0)      
                    state="FORWARD"  
                    initial_forward=True
                    angle=direction
                    
                       
            else:
                #Once entered end node, the state is Idle
                state="IDLE"
           
        elif state=="FORWARD":
            #To move a robot a certain distance
            
            if initial_forward==True:
                
                #Initialise the angle the robot needs to travel and encoders
                before_l=left_encoder.getValue()
                before_r=right_encoder.getValue()
                
                
                #Based on expanding the Proto node
                wheel_radius= 0.019
                
                #Working in radians 
                distance_angle=RES/wheel_radius
                
                left_motor.setVelocity(3)
                right_motor.setVelocity(3)
                initial_forward=False

            
            else:
                #Keeps on going forward until average angle is reached and then swtich state to Turn
                average_distance_angle=(abs(left_encoder.getValue()-before_l)+abs(right_encoder.getValue()-before_r))/2  
                if average_distance_angle>=distance_angle: 
                    left_motor.setVelocity(0.0)
                    right_motor.setVelocity(0.0)
                    j=j+1  
                    state="TURN"
                    initial_turn=True

           
            
    elif state=="IDLE":
        #Finished running
        print("Directions to end node:",order)
        print("Finished")
        exit() 
    

         
# End of cycle
cv2.destroyAllWindows()
