#### RANDOM RESTART HILL CLIMBING ALGORITHM WITH HEURISTIC (Local Search)
######## Designed for a use case where a set of integers must be assigned into a given number of bins without number repetition within bins, and with each bin summing up
######## up to an integer allowably between provided parameters minSum and maxSum. 



import random
import heapq
from copy import deepcopy
def hvs(state,testCase):
    # we revamp this to generate specifically the highest value successor. in order to minimise cost we basically need to find the biggest disparity 
    # biggest violation from the "sums" sublist within state
    # we then need to find a value that either closes this gap or reduces it. we must find a value in the entire list of values outside of the list we are at that can be moved to this list to close the gap without causing violations in other areas. 
    maxelim=[]
    new=deepcopy(state)
    maxSum=testCase["maxSum"]
    minSum=testCase["minSum"]
    over=[]
    under=[]
    for i in range(testCase["count"]):
        if state["sums"][i]<testCase["minSum"]:
            under.append((i,-testCase["minSum"]+state["sums"][i]))
        if state["sums"][i]>testCase["maxSum"]:
            over.append((i,state["sums"][i]-testCase["maxSum"]))
    over.sort(key=lambda a: -a[1])
    under.sort(key=lambda b: b[1])
    smallestCostChange=0
    maxinfo=()
    for i in over:
        refover=state["bins"][i[0]]
        oversum=state["sums"][i[0]]
        for j in under:
            refunder=state["bins"][j[0]]
            undersum=state["sums"][j[0]]
            for a in refover:
                if a in refunder: 
                    continue # cannot allow for repeat additions, which are clear violations
                newunderviol=0
                newoverviol=0
                if undersum + a > maxSum:
                    newunderviol = undersum + a - maxSum
                if oversum - a < minSum:
                    newoverviol = minSum - oversum + a
                newovercost=oversum-maxSum
                if oversum-a>maxSum:
                    newovercost=a
                newundercost=minSum-undersum
                if undersum+a<minSum:
                    newundercost=a
                # the violations will push the heuristic up and cost changes will push it down
                if  newunderviol + newoverviol - newundercost - newovercost < smallestCostChange:
                    smallestCostChange = newunderviol + newoverviol - newundercost - newovercost
                    maxinfo=(i[0],j[0],a)
    if maxinfo==():
        return state
    new["bins"][maxinfo[1]].append(maxinfo[-1])
    new["bins"][maxinfo[0]].remove(maxinfo[-1])
    new["sums"][maxinfo[1]]+=maxinfo[-1]
    new["sums"][maxinfo[0]]-=maxinfo[-1]
    # implement all state modifications and return this new neighbour, should be the highest value
    return new 
       # if diff<0: # we are dealing with a MINSUM VIOLATION
        #    for k in range(len(state["bins"])):
         #       for val in state["bins"][k]:
          #          if state["sums"][k]-val<minSum or refsum+k>maxSum:
           #             continue # if removing the value causes a violation, we skip it
            #        # remember moving a value will cause changes for both the list we are trying to amend and other lists
             #       effoncurr=state["sums"][k]-val
              #      if min(val,abs(val-minSum))+min(minSum-effoncurr,0)>maxcostdrop:
               #         maxcostdrop=min(val,abs(val-minSum))+min(effoncurr,0) # this is as the effect of adding a larger value than the  minsum difference will not cause a cost rise of the same quantity
                #        maxinfo=(k,val,pos,diff) # stores all information about which value to move from which bin to which bin. we perform the transfer at the end of sweep
        #elif diff>0: 
         #   for k in state["bins"][pos]:
          #      for j in len(state["bins"]):
           #         if state["sums"]+k>maxSum or refsum-k<minSum:
            #            continue # once again checking for illegal values if we are dealing with a minSum case
             #       if k
#def hvs(state,testCase):
    #highest value successor is that with the lowest heuristic function value: 
    # crude method: iterate throughout the values remaining,
 #   done=False
  #  ref=testCase["values"]
   # tested=[]
#    new=deepcopy(state)
 #   length=testCase["count"]
  #  maxS=testCase["maxSum"]
   # minS=testCase["minSum"]
    #while True:
     #   val=ref.pop()
      #  for i in range(length):
       #     if maxS-testCase["rmdr"][i]>=val and val not in new[i]:
        #        new[i].append(val)
         #       testCase["rmdr"][i]+=val
          #      new[-1]+=val
           #     for f in tested:
            #        heapq.heappush(ref,f)
             #   return new
#        tested.append(val)
 #   for g in tested: 
  #      heapq.heappush(ref,g)
   # return new
def initialise(testCase):
    ret=[[] for k in range(testCase["count"])]
    vals=testCase["values"]
    for i in vals:
        random.shuffle(ret)
        placed=False
        for k in ret:
            if i not in k:
                k.append(i)
                placed=True
                break
        if placed==False:
            return None
    return ret
def HC(testCase,max_iter,init):
    # should we create a remaining dictionary to update counts?
    curr=init
    fail=False
    minSum=testCase["minSum"]
    maxSum=testCase["maxSum"]
    for i in range(len(init["bins"])):
        if init["sums"][i]<minSum or init["sums"][i]>maxSum:
            fail=True
    if fail==False:
        return init
    iters=0
    total=sum(testCase["values"])
    while True:
        if iters>=max_iter:
            return None
        neighb=hvs(curr,testCase)
        if h(neighb,testCase,total)>=h(curr,testCase,total):
            return curr
        curr=neighb
        iters+=1
def local_search(testCase: dict, max_restarts: int = 1000, max_iterations: int = 10000, debug: bool = False) -> list:
    # we change the values list into a heapq so that we always start testing for the largest value first, seeing as by our heuristic that is what will always yield the highest value successor first (or at the very least is expected to)
    rep=set([k for k in testCase["values"] if testCase["values"].count(k)>1])
    vals=testCase["values"]
    minSum=testCase["minSum"]
    maxSum=testCase["maxSum"]
    for i in range(max_restarts):
        random.shuffle(vals)
        curr=None
        while curr==None:
            curr=initialise(testCase)
        state={}
        state["bins"]=curr
        state["sums"]=[sum(state["bins"][i]) for i in range(testCase["count"])]
        res=HC(testCase,max_iterations,state)
        if res!=None:
            restart=False
            for i in range(testCase["count"]):
                if res["sums"][i]<minSum or res["sums"][i]>maxSum:
                    restart=True
            if restart==False:
                return res["bins"]
            
    """
    Performs a random-restart hill climbing search for the CSP defined in testCase.
    Args:
        testCase (dict): A dictionary containing:
            - "count": Number of partitions needed.
            - "minSum": List of values to be partitioned.
            - "maxSum": The size of each partition.
        max_restarts (int): Max number of random restarts allowed.
        max_iterations (int): Max total hill climbing iterations allowed.
        debug (bool): If True, prints debug output.
    Returns:
        list: A list of partition solutions (one representative element per subset).
    """
    pass
def h(state,testCase,total):
    cost=0
    for i in range(len(state["bins"])):
        if state["sums"][i]>testCase["maxSum"]:
            cost+=state["sums"][i]-testCase["maxSum"]
        if state["sums"][i]<testCase["minSum"]:
            cost+=testCase["minSum"]-state["sums"][i]
    return cost # these values must be popped for each assignment and evidently copies need to be made
