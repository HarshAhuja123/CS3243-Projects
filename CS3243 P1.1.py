#UNINFORMED SEARCH MAZE SOLVING IMPLEMENTATIONS
### DFS 
from typing import List, Tuple
import collections
import heapq
def dfs_search(dct) -> List[Tuple[int, int]]:
    S=collections.deque()
    obst=set(map(tuple,dct["obstacles"]))
    obj=set(map(tuple,dct["goals"]))
    start=tuple(dct["start"])
    reached=set()
    parent={start:None}
    poss=[(-1,0),(1,0),(0,1),(0,-1)]
    if start not in obst:
        reached.add(start)
        S.append(start)
    else: return []
    while S:
        curr=S.pop()
        if curr in obj:
            final=[]
            now=curr
            while now!=None:
                final.append(now)
                now=parent[now]
            return final[::-1]
        for i in poss:
            ref=(curr[0]+i[0],curr[1]+i[1])
            if ref not in obst and ref[0]<dct["rows"] and ref[0]>=0 and ref[1]>=0 and ref[1]<dct["cols"] and ref not in reached:
                S.append(ref)
                parent[ref]=curr
                reached.add(ref)
    return []


#   BFS

def bfs_search(dct) -> List[Tuple[int, int]]:
    S=collections.deque()
    obst=set(map(tuple,dct["obstacles"]))
    obj=set(map(tuple,dct["goals"]))
    start=tuple(dct["start"])
    reached=set()
    parent={start:None}
    poss=[(-1,0),(1,0),(0,1),(0,-1)]
    if start not in obst:
        reached.add(start)
        S.appendleft(start)
    else: return []
    while S:
        curr=S.pop()
        if curr in obj:
            final=[]
            now=curr
            while now!=None:
                final.append(now)
                now=parent[now]
            return final[::-1]
        for i in poss:
            ref=(curr[0]+i[0],curr[1]+i[1])
            if ref not in obst and ref[0]<dct["rows"] and ref[0]>=0 and ref[1]>=0 and ref[1]<dct["cols"] and ref not in reached:
                S.append(ref)
                parent[ref]=curr
                reached.add(ref)
    return []


#   UCS

def ucs_search(dct) -> List[Tuple[int, int]]:
    S=[]
    obst=set(map(tuple,dct["obstacles"]))
    obj=set(map(tuple,dct["goals"]))
    start=(0,tuple(dct["start"]))
    reached=set()
    # our scheme is such that costs wll be updated in separate places, but it will also be stored in the dictionary to allow ordering
    parent={start[1]:None} # stored as a parent dictionary to store parent info
    poss=[(-1,0),(1,0),(0,1),(0,-1)] # for navigation, as before
    if start[1] not in obst:
        reached.add(start[1])
        heapq.heappush(S,start)
    else: return []
    cost={}
    cost[start[1]]=0
    while S:
        curr=heapq.heappop(S)
        if curr[1] in obj:
            final=[]
            now=curr[1]
            while now!=None:
                final.append(now)
                now=parent[now]
            return final[::-1]
        for i in poss:
            ref=(curr[0]+1,(curr[1][0]+i[0],curr[1][1]+i[1]))
           
            if ref[1] not in obst and ref[1][0]<dct["rows"] and ref[1][0]>=0 and ref[1][1]>=0 and ref[1][1]<dct["cols"]:
                if ref[1] not in reached or ref[0]<cost.get(ref[1],0):
                    cost[ref]=ref[0]
                    heapq.heappush(S,ref)
                    parent[ref[1]]=curr[1]
                    reached.add(ref[1])
                
    return []
