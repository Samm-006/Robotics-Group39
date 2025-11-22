"""SLAM controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math
import numpy as np

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
left_motor= robot.getDevice("left wheel motor")
right_motor= robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0)
right_motor.setVelocity(0)

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

R= 0.021
L= 0.053
STEPS= 1000
STEP_RAD= 2*math.pi/STEPS

x = y = theta = 0
prevL = 0 #check
prevR= 0  #check

RES= 0.02
SIZE= 4.0
N= int(SIZE/RES)
grid= np.zeros((N, N))

def w2m(px, py):
    mx= int ((px+ SIZE/2)/RES)
    my= int ((py+ SIZE/2)/RES)
    return(mx, my) if 0 <= mx < N and 0 <= my < N else None

IR_ANGLES = [+0.5, -0.5, 0.0, 3.14159, +1.57, -1.57, 0.0, 0.0]

def ir_dist(v):
    if v < 80: return None
    return max (0.02, min (0.3, 5/ (v-60)))
    
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    print("Running SLAM :" , x, y, theta)
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    Ls= left_encoder.getValue()
    Rs= right_encoder.getValue()
    dL= (Ls - prevL)*STEP_RAD*R
    dR= (Rs - prevR)*STEP_RAD*R
    prevL, prevR = Ls, Rs
    
    dC = (dL + dR)/2
    dT = (dR - dL)/L
    
    x += dC*math.cos(theta+ dT/2)
    y += dC*math.sin (theta+ dT/2)
    theta = (theta + dT +math.pi)%(2*math.pi) - math.pi
    
    #Mapping
    
    cell= w2m(x, y)
    if cell: grid [cell[0]][cell[1]] -= 0.05
    
    for i, s in enumerate(ir_sensors):
        d= ir_dist (s.getValue())
        if not d: continue 
        ang= theta + IR_ANGLES[i]
        hx= x+ d*math.cos(ang)
        hy= y+ d*math.sin(ang)
        c= w2m(hx, hy)
        if c:
            grid [c[0]][c[1]]+= 0.2
        
    #Motion
    front= ir_sensors [2].getValue() if ir_sensors [2] is not None else 0
    if front > 200:
        left_motor.setVelocity(-2)
        right_motor.setVelocity(2)
    else:
        left_motor.setVelocity(4)
        right_motor.setVelocity(4)
        

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
