# Import the libraries required for Webots and YOLO
from controller import Robot, Keyboard
import numpy as np  # For image conversion
import math
import cv2
import time

# Step 1: Initialisataion 

# Create a robot instance
robot = Robot()

timestep = int(robot.getBasicTimeStep())


left_motor= robot.getDevice("left wheel motor")
right_motor= robot.getDevice("right wheel motor")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)


left_encoder= robot.getDevice("left wheel sensor")
right_encoder= robot.getDevice("right wheel sensor")
left_encoder.enable(timestep)
right_encoder.enable(timestep)

ir_names = ["ds0", "ds1", "ds2", "ds3", "ds4", "ds5", "ds6", "ds7"]
ir_sensors = []
for name in ir_names:
    sensor = robot.getDevice(name)
    if sensor:
        sensor.enable(timestep)
    ir_sensors.append(sensor)

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

step_count=0

def w2m(px, py):
    mx = int((px+ SIZE/2.0)/RES)
    my = int((py+ SIZE/2.0)/RES)
    if 0 <= mx < N and 0 <= my < N :
        return (mx,my)
    return None
    
IR_ANGLES = [+0.5, -0.5, 0.0, 3.14159, +1.57, -1.57, 0.0, 0.0]

def ir_dist(v):
    if v < 80.0: 
        return None
    d = 5.0/(v - 60.0)
    d = max(0.02, min (0.3, d))

    return d 

# Main loop:
while robot.step(timestep) != -1:
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
        grid [cell[0]][cell[1]] -= 1
        
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
            grid [c[0]][c[1]]+= 0
          
          
    grid = np.clip(grid, -5.0, 5.0)
    
    step_count += 1
    
    #binary for A* (1= no obstacle, 0= obstacle)
    binary_grid = (grid > 0.3).astype(int)
    
    if step_count % 10 ==0:
        print("Grid")
        print(binary_grid)    

    #Motion
    front_val =ir_sensors[1].getValue() if ir_sensors[1] else 0
      
        
    if front_val > 200.0:
        left_motor.setVelocity(-2.0)
        right_motor.setVelocity(2.0)
    else:
        left_motor.setVelocity(4.0)
        right_motor.setVelocity(4.0)
