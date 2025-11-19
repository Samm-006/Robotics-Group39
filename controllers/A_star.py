import math
import copy

def hc(grid,start,end):
    #Using euclidean distance as heuristic h(n)
    heuristic=copy.deepcopy(grid)
    for i in range(len(grid)):
        for j in range(len(grid)):
            heuristic[i][j]=math.sqrt((j-end[1])**2+(i-end[0])**2)
    return heuristic


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
    #Every step cost 1 g(n)
    visited=[]
    open_nodes=[]
    step=1
    gn={tuple(start):0}
    current=start.copy()
    fn=[]
    path={}
    while end not in visited:
        if (current[0]+2)<=len(grid):
            if grid[current[0]+1][current[1]]==1 and ([current[0]+1,current[1]] not in visited):
                #Looks at node below
                open_nodes.append([current[0]+1,current[1]])
                if (current[0]+1,current[1]) in gn:
                    if gn[(current[0]+1,current[1])]>gn[(current[0],current[1])]+step:
                        gn[(current[0]+1,current[1])]=gn[(current[0],current[1])]+step
                else:
                     gn[(current[0]+1,current[1])]=gn[(current[0],current[1])]+step
                fn.append(gn[(current[0]+1,current[1])]+heuristic[current[0]+1][current[1]])
                path[(current[0]+1,current[1])] = (current[0], current[1])
        if (current[1]+2)<=len(grid):
            if grid[current[0]][current[1]+1]==1 and ([current[0],current[1]+1] not in visited):
                #Looks at node right
                open_nodes.append([current[0],current[1]+1])
                if (current[0],current[1]+1) in gn:
                    if gn[(current[0],current[1]+1)]>gn[(current[0],current[1])]+step:
                        gn[(current[0],current[1]+1)]=gn[(current[0],current[1])]+step
                else:
                    gn[(current[0],current[1]+1)]=gn[(current[0],current[1])]+step
                fn.append(gn[(current[0],current[1]+1)]+heuristic[current[0]][current[1]+1])
                path[(current[0],current[1]+1)] = (current[0], current[1])
        if (current[0]-1)>=0:
            if grid[current[0]-1][current[1]]==1 and ([current[0]-1,current[1]] not in visited):
                #Looks at node above
                open_nodes.append([current[0]-1,current[1]])
                if (current[0]-1,current[1]) in gn:
                    if gn[(current[0]-1,current[1])]>gn[(current[0],current[1])]+step:
                        gn[(current[0]-1,current[1])]=gn[(current[0],current[1])]+step
                else: 
                    gn[(current[0]-1,current[1])]=gn[(current[0],current[1])]+step
                fn.append(gn[(current[0]-1,current[1])]+heuristic[current[0]-1][current[1]])
                path[(current[0]-1,current[1])] = (current[0], current[1])
        if (current[1]-1)>=0:
            if grid[current[0]][current[1]-1]==1 and ([current[0],current[1]-1] not in visited):
                #Looks at the nodel left
                open_nodes.append([current[0],current[1]-1])
                if (current[0],current[1]-1) in gn:            
                    if gn[(current[0],current[1]-1)]>gn[(current[0],current[1])]+step:
                        gn[(current[0],current[1]-1)]=gn[(current[0],current[1])]+step
                else: 
                    gn[(current[0],current[1]-1)]=gn[(current[0],current[1])]+step
                fn.append(gn[(current[0],current[1]-1)]+heuristic[current[0]][current[1]-1])
                path[(current[0],current[1]-1)] = (current[0], current[1])
       
        
        if len(open_nodes)==0:
            print("No path exists")
            exit()
        
        mini=min(fn)
        
        for i in range(len(fn)):
            if mini==fn[i]:
                position=i
        
        visited.append(current.copy())

        current=open_nodes[position]

        open_nodes.remove(current)
        fn.pop(position)
        
    full_path=[]
    current=end
    full_path.append(tuple(current))
    while (current[0],current[1]) != (start[0],start[1]):
        current=path[(current[0],current[1])]
        full_path.append(current)
        
    full_path.reverse()
    print("Order of nodes:", full_path)
    
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
