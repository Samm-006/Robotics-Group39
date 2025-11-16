import math
import copy

def hc(grid,start,end):
    #Using euclidean distance as heuristic h(n)
    heuristic=copy.deepcopy(grid)
    for i in range(len(grid)):
        for j in range(len(grid)):
            heuristic[i][j]=math.sqrt((j-end[1])**2+(i-end[0])**2)
    print(heuristic)
    return heuristic


#Output of SLAM     
grid = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1],
    [0, 0, 0, 0, 0, 0, 0, 1, 1],
    [1, 0, 1, 1, 1, 1, 1, 0, 1],
    [1, 0, 1, 0, 1, 0, 1, 0, 1],
    [1, 0, 1, 0, 1, 0, 1, 0, 1],
    [1, 0, 1, 0, 1, 0, 1, 0, 1],
    [1, 1, 1, 1, 0, 1, 1, 1, 1],
    [1, 1, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 0, 1, 1]
]


start=[0,8]
end=[7,1]


heuristic=hc(grid,start,end)



#Needs to keep track of all open nodes
#each open node will have fn

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
                #fn[0]=gn+step+heuristic[current[0]+1][current[1]]
                print("down")
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
                #fn[1]=gn+step+heuristic[current[0]][current[1]+1]
                print("right")
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
                #fn[2]=gn+step+heuristic[current[0]-1][current[1]]
                print("up")
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
                #fn[3]=gn+step+heuristic[current[0]][current[1]-1]
                print("left")
                open_nodes.append([current[0],current[1]-1])
                if (current[0],current[1]-1) in gn:            
                    if gn[(current[0],current[1]-1)]>gn[(current[0],current[1])]+step:
                        gn[(current[0],current[1]-1)]=gn[(current[0],current[1])]+step
                else: 
                    gn[(current[0],current[1]-1)]=gn[(current[0],current[1])]+step
                fn.append(gn[(current[0],current[1]-1)]+heuristic[current[0]][current[1]-1])
                path[(current[0],current[1]-1)] = (current[0], current[1])
        print(fn)
        
        if len(open_nodes)==0:
            print("No path exists")
            exit()
        
        mini=min(fn)
        print(mini)
        
        for i in range(len(fn)):
            if mini==fn[i]:
                position=i
        print(position)
        
        print(path)
        
        visited.append(current.copy())
        
        
        
        
        current=open_nodes[position]
        
        

        open_nodes.remove(current)
        fn.pop(position)
        
            

        print(current)
        #print(visited)
        print(open_nodes)
            #print(gn)
            
        
    full_path=[]
    current=end
    full_path.append(current)
    while (current[0],current[1]) != (start[0],start[1]):
        current=path[(current[0],current[1])]
        full_path.append(current)
        print(current)
        
    full_path.reverse()
    print(full_path)
        
A_star_search(grid,start,end,heuristic)
