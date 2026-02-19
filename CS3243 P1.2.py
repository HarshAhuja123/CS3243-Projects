from typing import List, Tuple
import collections
import heapq
import copy
# to recap:
#    Nuke: must search systematically within a certain radius and clear out ALL creep data in valid areas, update environment and push into new node, then push new node with new action but same parent and action as last one. 
print(sum([1/k**2 for k in range(1,20000)]))
        
def backtrack_actions(curr,start):
    now=curr
    acts=collections.deque()
    while now!=start:
        acts.append(now["action"][0])
        now=now["parent"]
    final=[]
    while len(acts)!=0:
        final.append(acts.pop())
    return final

def h(curr,obj):
    poss=[]
    for i in obj:
        poss.append((abs(curr[1]-i[1])+abs(curr[0]-i[0])))
    return 4*min(poss) # average distance travelled more representative and a smaller underestimation.

def TM(parent,action): # only purpose is to move the pointer. All skill based special treatment will occur within the main algorithm loop
    if action==0:
        return (parent[0]-1,parent[1])
    if action==1:
        return (parent[0]+1,parent[1])
    if action==2:
        return (parent[0],parent[1]-1)
    if action==3:
        return (parent[0],parent[1]+1)

def creepcost(centers,dict,state):
    for i in centers:
            if abs(state[0]-i[0])+abs(state[1]-i[1])<=10:
                return 0
    return dict.get(state,0)



#WE ARE INTRODUCING THE ENVIRONMENT ATTRIBUTE TO THE NODE AS ENVIRONMENT IS ALSO CHANGING
def search(dct) -> list[int]:
    BH=[]
    dict=dct
    obst=set(map(tuple,dct["obstacles"]))
    obj=set(map(tuple,dct["goals"]))
    creepdict={}
    for i in dict["creeps"]:
        creepdict[(i[0],i[1])]=i[2] # hashing to improve efficiency
        
    # IMPORTANT   
    # we will NEVER need to ADD creep data in, so we can copy paste the same dictionary without needing to reHASH
    
    
    
    count=0
    dict["creeps"]=set() # EASIER FOR ENVIRONMENT EDITS
    dict.pop("obstacles")
    dict.pop("goals")
    cols=dict["cols"]
    rows=dict["rows"]
    #start=(0,Node(state=tuple(dict["start"]),parent=0,action="",cost=0,F=False,env=dict))
    start=((h(tuple(dict["start"]),obj)),{"state":tuple(dict["start"]),"parent":{},"action":None,"cost":0,"F":False,"env":dict})
    heapq.heappush(BH,(start[0],count,start[1]))
    cost={}
    reached={(start[1]["state"],dict["num_flash_left"],dict["num_nuke_left"],frozenset(dict["creeps"]))}
    cost[(start[1]["state"],dict["num_flash_left"],dict["num_nuke_left"],frozenset(dict["creeps"]))]=0
    poss=[0,1,2,3,4,5]
    dirns=[(dx,dy) for dx in range(-10,11) for dy in range(-10,11) if abs(dx)+abs(dy)<=10] # storing all offsets within manhattan distance of 10
    while len(BH)>0:
        curr=heapq.heappop(BH)
        if curr[2]["state"] in obj:
            return backtrack_actions(curr[2],start[1]) # must redefine this function to output the ACTION and not the path
        
        for i in poss:
            topush=[]
            ### IF FLASH INVOKED
            if i==4 and curr[2]["env"]["num_flash_left"]>0 and curr[2]["F"]!=True:
                newenv=copy.deepcopy(curr[2]["env"])
                newenv["num_flash_left"]=max(0,newenv["num_flash_left"]-1)
                topush.append((curr[2]["cost"]+10+h(curr[2]["state"],obj)/2,{"state":curr[2]["state"],"parent":curr[2],"action":(i,False),"cost":curr[2]["cost"]+10,"F":True,"env":newenv}))
                # cost incrementation and the flash signal set for child to know            
            elif i==5 and curr[2]["env"]["num_nuke_left"]>0:
                # remove creep data for all within 10 manhattan distance tiles
                now=curr[2]["state"]
                # we edit the creeps key of this environment
                 ## COORDINATE GENERATION ABOVE for all manhattan distances. given any point, there is scope to reverse the order of the numbers too, and this multiple direction state is reflected within the loop below
                node=(curr[2]["cost"]+50+h(curr[2]["state"],obj),{"state":curr[2]["state"],"parent":curr[2],"action":(5,False),"cost":curr[2]["cost"]+50,"F":False,"env":{"num_nuke_left":max(0,curr[2]["env"]["num_nuke_left"]-1),"num_flash_left":curr[2]["env"]["num_flash_left"]}})

                node[1]["env"]["creeps"]=copy.deepcopy(curr[2]["env"]["creeps"])
                node[1]["env"]["creeps"].add(now)
                #for coord in dirns:
                 #   ref=(now[0]+coord[0],now[1]+coord[1])
                  #  if  ref[0]>=0 and ref[0]<rows and ref[1]>=0 and ref[1]<cols and ref not in obst:
                   #     node[1]["env"]["creeps"].add(ref) 
                topush.append(node)
 ## +h(now["state"],newenv) not needed for the last line here, as no movement change, only creep done.
 # consider that a flash can result in you reaching from somewhere to someplace else. flash invocation is one action. all following actions are also affected by the flash
 # you can visit A from B using either standard movements, or a flash
            ##### IF PARENT INVOKES FLASH --- EDIT TO ADD IN HEURISTIC
            elif curr[2]["F"]==True and i!=4 and i!=5: 
                now=curr[2]["state"]
                env=curr[2]["env"]
                costc=curr[2]["cost"]
                last=now
                while True:
                    now=TM(now,i)
                    if now in obst or now[0]<0 or now[0]>=rows or now[1]<0 or now[1]>=cols:
                        break
                    last=now
                    costc+=2+creepcost(curr[2]["env"]["creeps"],creepdict,last)
                if last!=curr[2]["state"]:
                    topush.append((costc+h(last,obj),{"state":last,"cost":costc,"parent":curr[2],"action":(i,True),"env":env,"F":False})) # true flag is specifically for flashes
                # this loop will handle the append operation till the first obstacle is encountered.
            if i in poss[0:4]:
                k=TM(curr[2]["state"],i) # standard state of no FLASH invocation
                if k not in obst and k[0]>=0 and k[0]<rows and k[1]>=0 and k[1]<cols: # obstacle check for standard case as well
                    topush.append((curr[2]["cost"]+4+creepcost(curr[2]["env"]["creeps"],creepdict,k)+h(k,obj),{"state":k,"parent":curr[2],"action":(i,False),"cost":curr[2]["cost"]+4+creepcost(curr[2]["env"]["creeps"],creepdict,k),"F":False,"env":curr[2]["env"]}))
            # deciding what does and does not get admitted into the main binary heap
            for pt in topush:
                if (pt[1]["state"],pt[1]["env"]["num_flash_left"],pt[1]["env"]["num_nuke_left"],frozenset(pt[1]["env"]["creeps"])) not in reached or cost.get((pt[1]["state"],pt[1]["env"]["num_flash_left"],pt[1]["env"]["num_nuke_left"],frozenset(pt[1]["env"]["creeps"])),pt[0]+1)>pt[0]:
                    reached.add((pt[1]["state"],pt[1]["env"]["num_flash_left"],pt[1]["env"]["num_nuke_left"],frozenset(pt[1]["env"]["creeps"])))
                    cost[(pt[1]["state"],pt[1]["env"]["num_flash_left"],pt[1]["env"]["num_nuke_left"],frozenset(pt[1]["env"]["creeps"]))]=pt[0]
                    heapq.heappush(BH,(pt[0],count,pt[1]))
                    count+=1
    return []
