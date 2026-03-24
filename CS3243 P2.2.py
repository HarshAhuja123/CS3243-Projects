from collections import deque
ref=[]
def compare(a,b):
  return not (a==1 and b==1)
  # we are lucky because the constraint is NOT directional, so no constraint preprocessing needs to be done. it can be tested on everything
def init(inp,cell):
  obst=inp["obstacles"]
  sizes=[(1,cell[0],cell[1])]
  k=1
  while k<=inp["max_square_size"]:
    done=False
    for i in range(0,k):
      for j in range(0,k):
        if cell[1]+j>=inp["cols"] or cell[0]+i>=inp["rows"] or [cell[0]+i,cell[1]+j] in obst:
          return sizes
    sizes.append((k,cell[0],cell[1]))
    k+=1
  return sizes
def overlap(a, b):
    sa,ra,ca = a
    sb,rb,cb = b
    return not (
        ra + sa <= rb or   # a above b
        rb + sb <= ra or   # b above a
        ca + sa <= cb or   # a left of b
        cb + sb <= ca      # b left of a
    )
def lcvorder(var,inp,assignm):
  vals=inp["vars"][var][0]
  if len(vals)==1:
    return vals
  temp=0
  constrs=inp["constraints"][var]
  domsum=0
  for i in constrs:
    if i in assignm:
      continue
    for poss in inp["vars"][i][0]:
      if compare(poss,0)==False:
        domsum+=1
  temp=1
  domsumnew=0
  for j in constrs:
    if j in assignm:
      continue
    for poss in inp["vars"][j][0]:
      if compare(poss,1)==False:
        domsumnew+=1
  if domsumnew>domsum:
    return [1,0]
  return [0,1]
def selvar(assignm, inp):
  for i in ref:
    if i in assignm:
      continue
    return i
    minvar = None
    best = (float('inf'), float('-inf'))
    for v in inp["vars"]:
        if v in assignm:
            continue
        domain = inp["vars"][v][0]
        domain_size = len(domain)
        # largest square available for this variable
        max_square = max(d for d in domain)
        # MRV first, then prefer variables that allow larger squares
        score = (domain_size, -max_square)
        if score[1] > best[1]:
            best = score
            minvar = v
    return minvar
    # we must prune illegal values. i.e. we run this against everything within max_square size and which is not an obstacle
    
def forward(assignm,var,inp):
  for i in inp["constraints"][var]:
    if i in assignm: 
      continue
    remove=[]
    count=0
    if assignm[var]==1:
      if 1 in inp["vars"][i][0]:
        inp["vars"][i][0].remove(1)
        inp["vars"][i][1].append(1)
        count+=1
        
        if inp["vars"][i][0]==[]:
          return False
    inp["vars"][i][2].append(count)
  return True
def backtrack(assignm,inp):
  if assignm[0] > inp["rows"]*inp["cols"] - len(inp["obstacles"]) or sum(assignm.values())-assignm[0]>inp["square_count"]:
    return False
  if (sum(assignm.values())-assignm[0]==inp["square_count"] and assignm[0]==inp["rows"]*inp["cols"]-len(inp["obstacles"])):
      full = set()
      for var in assignm:
          if var == 0:
              continue
          if assignm[var] == 1:
              s, i, j = var
              for r in range(i, i+s):
                  for c in range(j, j+s):
                      if (r,c) not in inp["obstacles"]:
                          full.add((r,c))
      for r in range(inp["rows"]):
          for c in range(inp["cols"]):
              if [r,c] not in inp["obstacles"] and (r,c) not in full:
                  return False
      return [i for i in assignm if assignm[i]==1]
      
  var=selvar(assignm,inp)
  if var==None:
    return False
  for val in sorted(inp["vars"][var][0], key=lambda x: -x):
    # we now perform a consistency check specifically for this value
    cons=True
    for neighbor in inp["constraints"][var]:
      if neighbor in assignm:
          if not compare(val, assignm[neighbor]):
              cons = False
              break
    if cons==True:
      assignm[var]=val # addition is the same as value assignment 1
      assignm[0]+=(var[0]**2)*val
      inference=forward(assignm,var,inp)
      if inference!=False:
        res=backtrack(assignm,inp)
        if res!=False:
          return res
        for w in inp["constraints"][var]:
          refpush=inp["vars"][w][2].pop() if len(inp["vars"][w][2])!=0 else 0
          for p in range(refpush):
            inp["vars"][w][0].append(inp["vars"][w][1].pop())
        
      # now we deal with the fact that the inference does turn out to be FALSE
      del assignm[var]
      assignm[0]-=(var[0]**2)*val
  return False
def solve_CSP(inp):
    global ref
    # we must first redefine the problem to work well with a CSP configuration : requires us to use 
    # variables: perhaps square of size
    # our constraints are that squares cannot overlap. # the only binary constraints we have is for squares that we already know about
    inp["vars"]={}
    # we go to every cell and do a breadth first search expanding down and to the right. we stop the moment we hit an obstacle
    for i in range(inp["rows"]):
      for j in range(inp["cols"]):
        if [i,j] in inp["obstacles"]:
          continue
        for g in init(inp,[i,j]): # init returns possible values from [i,j]
          inp["vars"][g]=[[0,1],deque(),deque()]
    inp["constraints"]={}
    ref=list(inp["vars"].keys())
    for k in inp["vars"]:
      inp["constraints"][k]=set()
    for k in range(len(inp["vars"])):
      for f in range(k+1,len(inp["vars"])):
        a=ref[k]
        b=ref[f]
        if overlap(a,b):
          inp["constraints"][a].add(b)
          inp["constraints"][b].add(a)
          
           # can store as set since ordering doesn't matter? @ also NO need to specify unique constraints for all of them seeing as the necessary check is constant across all of them
    ref=sorted(list(inp["vars"].keys()),key=lambda a : -a[0])
    return backtrack({0:0},inp)
