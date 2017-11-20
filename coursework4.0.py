import numpy as np
import scipy as sp
import math as ma
import sys

# calculate the shortest path.
def Dijkst(ist,isp,wei):
    # Dijkstra algorithm for shortest path in a graph
    #    ist: index of starting node
    #    isp: index of stopping node
    #    wei: weight matrix

    # exception handling (start = stop)
    if (ist == isp):
        shpath = [ist]
        return shpath

    # initialization
    N         =  len(wei)
    Inf       =  sys.maxint
    UnVisited =  np.ones(N,int)
    cost      =  np.ones(N)*1.e6
    par       = -np.ones(N,int)*Inf

    # set the source point and get its (unvisited) neighbors
    jj            = ist
    cost[jj]      = 0
    UnVisited[jj] = 0
    tmp           = UnVisited*wei[jj,:]
    ineigh        = np.array(tmp.nonzero()).flatten()
    L             = np.array(UnVisited.nonzero()).flatten().size

    # start Dijkstra algorithm
    while (L != 0):
        # step 1: update cost of unvisited neighbors,
        #         compare and (maybe) update
        for k in ineigh:
            newcost = cost[jj] + wei[jj,k]
            if ( newcost < cost[k] ):
                cost[k] = newcost
                par[k]  = jj

        # step 2: determine minimum-cost point among UnVisited
        #         vertices and make this point the new point
        icnsdr     = np.array(UnVisited.nonzero()).flatten()
        cmin,icmin = cost[icnsdr].min(0),cost[icnsdr].argmin(0)
        jj         = icnsdr[icmin]

        # step 3: update "visited"-status and determine neighbors of new point
        UnVisited[jj] = 0
        tmp           = UnVisited*wei[jj,:]
        ineigh        = np.array(tmp.nonzero()).flatten()
        L             = np.array(UnVisited.nonzero()).flatten().size

    # determine the shortest path
    shpath = [isp]
    while par[isp] != ist:
        shpath.append(par[isp])
        isp = par[isp]
    shpath.append(ist)

    return shpath[::-1]

def calcWei(RX,RY,RA,RB,RV):
    # calculate the weight matrix between the points

    n    = len(RX)
    wei = np.zeros((n,n),dtype=float)
    m    = len(RA)
    for i in range(m):
        xa = RX[RA[i]-1]
        ya = RY[RA[i]-1]
        xb = RX[RB[i]-1]
        yb = RY[RB[i]-1]
        dd = ma.sqrt((xb-xa)**2 + (yb-ya)**2)
        tt = dd/RV[i]
        wei[RA[i]-1,RB[i]-1] = tt
    return wei

#since the weights change at every iteration,
#define a new weight function. 
def NewWeight(M):    
    #calculate the updated weights.
    for k in range(len(wei)):
       for l in range(len(wei)): 
          if wei[k,l]>0: #only update for non-zero weights
                M[k,l]=wei[k,l]+0.5*eta*(CurrentCars[k] \ #formula
                +CurrentCars[l])
    return M

if __name__ == '__main__':

    import numpy as np
    import scipy as sp
    import csv
#Project 1 
    RomeX = np.empty(0,dtype=float) # import data
    RomeY = np.empty(0,dtype=float)
    with open('RomeVertices','r') as file:
        AAA = csv.reader(file)
        for row in AAA:
            RomeX = np.concatenate((RomeX,[float(row[1])]))
            RomeY = np.concatenate((RomeY,[float(row[2])]))
    file.close() 

    RomeA = np.empty(0,dtype=int)
    RomeB = np.empty(0,dtype=int)
    RomeV = np.empty(0,dtype=float)
    with open('RomeEdges','r') as file:
        AAA = csv.reader(file)
        for row in AAA:
            RomeA = np.concatenate((RomeA,[int(row[0])]))
            RomeB = np.concatenate((RomeB,[int(row[1])]))
            RomeV = np.concatenate((RomeV,[float(row[2])]))
    file.close()
    
    wei = calcWei(RomeX,RomeY,RomeA,RomeB,RomeV) #calculate the weights 
    WeiN=np.zeros([len(wei),len(wei)]) #to store new weights
    NofCars=np.zeros([len(wei),200]) #store #ofcars after each iteration
    CarsOut=np.zeros(200) #record #of cars left for verification
    CurrentCars=np.zeros(len(wei)) #copy of NofCars at iteration-1
    eta=0.01 #eta value for weight calculation 
    Edges=zip(RomeA,RomeB) #all edges to be removed once it's utilised  
    #start iterations 
    #for minute=0 
    #step 1: inject car
    CurrentCars[12]+=20
    #step 2: update weights 
    WeiN=NewWeight(WeiN)
    #step 3: run Dijkstra using new weights
    shpath=Dijkst(12,51,WeiN)
    #step 4: calculate the car movements at the node 
    #calculate the outbound car first
    #deduct another one to get the remaining cars 
    NofCars[shpath[1],0]=0.7*(CurrentCars[shpath[0]])  
    NofCars[shpath[0],0]=CurrentCars[shpath[0]]-\
                      0.7*(CurrentCars[shpath[0]]) 
   #start the loop
   #cars are injected for first 180 iterations only 
   #step 1 to 4 are repeated 
    for minute in range(1,200):
   #update the flowing vector.
        CurrentCars=NofCars[:,minute-1].copy() #number of cars at the moment 
        if minute in range(1,180): #inject cars
            CurrentCars[12]+=20
        else:
            pass 
        WeiN=NewWeight(WeiN) #update the weights
        for car in range(len(NofCars)):
            if CurrentCars[car]>0: #only run dijkstra for nodes with cars
                shpath=Dijkst(car,51,WeiN) #remember to use new weights
               #calculate the car movements
                if car != 51: #separate working for node 51 
                   NofCars[shpath[1],minute]+=round(0.7* \
                                    (CurrentCars[shpath[0]]))
                   NofCars[shpath[0],minute]+=CurrentCars[shpath[0]]-\
                        round(0.7*(CurrentCars[shpath[0]])) 
                   if (shpath[0]+1,shpath[1]+1) in Edges: 
                      #gather utilised edges & remove them 
                      Edges.remove((shpath[0]+1,shpath[1]+1))
                else :  #for node 51 particularly 
                   CarsOut[minute]+=CurrentCars[shpath[0]]-\
                            round(CurrentCars[shpath[0]]*0.6)           
                   NofCars[shpath[0],minute]+=round(CurrentCars[shpath[0]]\
                                            *0.6)
            else: 
                pass 
    print NofCars #show all the number of cars at every iteration
    print 'Maximum number of cars at each node is', NofCars.max(axis=1)
    print 'There are', len(Edges), 'nodes that are not utilised:', Edges 

   #check that number of cars is consistent
    CarStayed= np.sum(NofCars,axis=0) #sum by columns
    TotalCars=np.zeros(len(CarStayed)) #store the total #ofcars
    for i in range(len(CarStayed)):
       TotalCars[i]+=CarStayed[i]+np.sum(CarsOut[:i+1])
    print 'The total number of cars in the iteration is',TotalCars 
    
    #calculate the most congested nodes
    CongNodes=[] #store congested nodes
    Max=list(NofCars.max(axis=1)) #make it a list
    #sort out the top five
    SortedList= sorted(NofCars.max(axis=1), reverse=True)[:5]  
    for i in SortedList: 
       for v,val in enumerate(Max):
          if val==i:  #find the index in max list 
             if v+1 not in CongNodes:
                CongNodes.append(v+1)
    print 'Five most congested nodes are', CongNodes, 
    print 'with the corresponding maximum number of', SortedList  
    
    # eta=0
    # maximum numbers go is at node 52
    # many not utilized 
    
    # Node 30 dead. 
    wei = calcWei(RomeX,RomeY,RomeA,RomeB,RomeV) #calculate the weights 
    wei[:,29]=0 #routes connect node 30 =0 
    wei[29,:]=0 
    WeiN=np.zeros([len(wei),len(wei)]) #to store new weights
    NofCars=np.zeros([len(wei),200]) #store #ofcars after each iteration
    CarsOut=np.zeros(200) #record #of cars left for verification
    CurrentCars=np.zeros(len(wei)) #all car numbers are taken here
    eta=0.01 #eta value for weight calculation 
    Edges=zip(RomeA,RomeB) #all edges to be removed once it's utilised  
    #Start iterations 
    #For minute=0 
    NofCars[12,0]+=20
    CurrentCars[12]+=20
    WeiN=NewWeight(WeiN)
    shpath=Dijkst(12,51,WeiN)
    NofCars[shpath[1],0]=0.7*(CurrentCars[shpath[0]])  
    NofCars[shpath[0],0]=CurrentCars[shpath[0]]-0.7*(CurrentCars[shpath[0]]) 
    for minute in range(1,200):
        CurrentCars=NofCars[:,minute-1].copy() #number of cars at the moment 
        if minute in range(1,180): #inject cars for the first 180 iterations.
            CurrentCars[12]+=20
        else:
            pass 
        WeiN=NewWeight(WeiN) #update the weights
        for car in range(len(NofCars)):
            if CurrentCars[car]>0: 
                shpath=Dijkst(car,51,WeiN)
               #calculate the number of cars left and stayed. 
                if car != 51:
                   NofCars[shpath[1],minute]+=round(0.7* \
                                    (CurrentCars[shpath[0]]))
                   NofCars[shpath[0],minute]+=CurrentCars[shpath[0]]-\
                                round(0.7*(CurrentCars[shpath[0]])) 
                   if (shpath[0]+1,shpath[1]+1) in Edges: 
                      Edges=filter(lambda a:a!=((shpath[0]+1,shpath[1]+1))\
                                   ,Edges)
                else : 
                   CarsOut[minute]+=CurrentCars[shpath[0]]-\
                            round(CurrentCars[shpath[0]]*0.6)           
                   NofCars[shpath[0],minute]+=round(CurrentCars[shpath[0]]\
                                            *0.6)
            else: 
                pass 
    print 'Maximum number of cars at each node is', NofCars.max(axis=1)
    print 'There are', len(Edges), 'edges that are not utilised:', Edges
    #calculate the congested nodes 
    CongNodeswh=[] #store the congested nodes
    Maxwh=list(NofCars.max(axis=1))
    #sort out the top five with maximum number
    SortedListwh= sorted(NofCars.max(axis=1), reverse=True)[:5]  
    for i in SortedListwh: 
       for v,val in enumerate(Maxwh):
          if val==i:
             if v+1 not in CongNodeswh:
                CongNodeswh.append(v+1)
    print 'Five most congested nodes are', CongNodeswh
    print 'with the corresponding maximum number of', SortedListwh 
    #calculate difference 
    Maxwh=NofCars.max(axis=1) #max of new NofCars
    Difference=Maxwh-Max #get the difference between the before and after
    print 'Node with the peak value'
    print 'increase the most is node', Difference.argmax()+1,
    print 'with', Difference.max()
    Difference=list(Difference)
    Sorted= sorted(Difference, reverse=True)[-2] #get the second one
    print 'Node with the peak value'
    print 'decrease the most is node', Difference.index(Sorted)+1,
    print 'with', -Sorted
    
       
 
        
            
            
     