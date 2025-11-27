import math
import copy
from queue import PriorityQueue

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
#grid=test1()
grid=test2()
#grid=test3()



heuristic=hc(grid,start,end)



def A_star_search(grid,start,end,heuristic):
    #Main A* algorithm
    
    #Initialises visited nodes, open nodes, step cost, gn, current node, fn and path connection two nodes
    visited=[]
    open_nodes=[]
    step=1
    gn={tuple(start):0}
    current=start.copy()
    f=PriorityQueue()
    path={}
    while end not in visited:
        #Until a path is created, checks if neighbouring nodes is available to search and not already visited
        #For each neighbouring node possible, update open node list, gn value, fn value and path connection between two nodes
        
        #Looks at node below
        if (current[0]+2)<=len(grid):
            if grid[current[0]+1][current[1]]==1 and ([current[0]+1,current[1]] not in visited):
                open_nodes.append([current[0]+1,current[1]])
                if (current[0]+1,current[1]) in gn:
                    if gn[(current[0]+1,current[1])]>gn[(current[0],current[1])]+step:
                        gn[(current[0]+1,current[1])]=gn[(current[0],current[1])]+step
                else:
                     gn[(current[0]+1,current[1])]=gn[(current[0],current[1])]+step
                f.put((gn[(current[0]+1,current[1])]+heuristic[current[0]+1][current[1]],([current[0]+1,current[1]])))
                path[(current[0]+1,current[1])] = (current[0], current[1])
        #Looks at node right
        if (current[1]+2)<=len(grid):
            if grid[current[0]][current[1]+1]==1 and ([current[0],current[1]+1] not in visited):
                open_nodes.append([current[0],current[1]+1])
                if (current[0],current[1]+1) in gn:
                    if gn[(current[0],current[1]+1)]>gn[(current[0],current[1])]+step:
                        gn[(current[0],current[1]+1)]=gn[(current[0],current[1])]+step
                else:
                    gn[(current[0],current[1]+1)]=gn[(current[0],current[1])]+step
                f.put((gn[(current[0],current[1]+1)]+heuristic[current[0]][current[1]+1],([current[0],current[1]+1])))
                path[(current[0],current[1]+1)] = (current[0], current[1])
        #Looks at node above
        if (current[0]-1)>=0:
            if grid[current[0]-1][current[1]]==1 and ([current[0]-1,current[1]] not in visited):
                open_nodes.append([current[0]-1,current[1]])
                if (current[0]-1,current[1]) in gn:
                    if gn[(current[0]-1,current[1])]>gn[(current[0],current[1])]+step:
                        gn[(current[0]-1,current[1])]=gn[(current[0],current[1])]+step
                else: 
                    gn[(current[0]-1,current[1])]=gn[(current[0],current[1])]+step
                f.put((gn[(current[0]-1,current[1])]+heuristic[current[0]-1][current[1]],([current[0]-1,current[1]])))
                path[(current[0]-1,current[1])] = (current[0], current[1])      
        #Looks at node left
        if (current[1]-1)>=0:
            if grid[current[0]][current[1]-1]==1 and ([current[0],current[1]-1] not in visited):
                open_nodes.append([current[0],current[1]-1])
                if (current[0],current[1]-1) in gn:            
                    if gn[(current[0],current[1]-1)]>gn[(current[0],current[1])]+step:
                        gn[(current[0],current[1]-1)]=gn[(current[0],current[1])]+step
                else: 
                    gn[(current[0],current[1]-1)]=gn[(current[0],current[1])]+step
                f.put((gn[(current[0],current[1]-1)]+heuristic[current[0]][current[1]-1],([current[0],current[1]-1])))
                path[(current[0],current[1]-1)] = (current[0], current[1])
       
        
        #Stops code if no possible path exists
        if len(open_nodes)==0:
            print("No path exists")
            exit()
        
        #Finds next node to expand      
        values=f.get()
   
        visited.append(current.copy())

        current=list(values[1])

        open_nodes.remove(current)
     
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
