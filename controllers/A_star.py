import math
import copy
import heapdict

def hc(grid,start,end):
    #Using euclidean distance as heuristic h(n)
    heuristic=copy.deepcopy(grid)
    for i in range(len(grid)):
        for j in range(len(grid)):
                heuristic[i][j]=math.sqrt((j-end[1])**2+(i-end[0])**2)
    return heuristic


#Robot's position and goal 
start=[1,7]
end=[7,1]


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


#uncomment to test a grid    
grid=test1()
#grid=test2()
#grid=test3()



heuristic=hc(grid,start,end)



def A_star_search(grid,start,end,heuristic):
    #Main A* algorithm
    
    #Initialises visited nodes, open nodes, step cost, gn, current node, fn and path connection two nodes
    visited=[]
    step=1
    gn={tuple(start):0}
    current=start.copy()
    fn=heapdict.heapdict()
    path={}
    while end not in visited:
        #Until a path is created, checks if neighbouring nodes is available to search and not already visited
        #For each neighbouring node possible, update open node list, gn value, fn value and path connection between two nodes
        
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
    
    
    
A_star_search(grid,start,end,heuristic)


#Code developed from here onwards is mine but does not work unless integrated in SLAM
#Working code is in Group_39_Integrated_System controller
#this code is just to show what I have done
''' 
order=[]
initial_turn=True
initial_forward=True
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

'''
