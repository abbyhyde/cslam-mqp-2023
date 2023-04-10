"""
Map-Merging Algorithm for ROS mapping
Based off Merging Occupancy Grid Maps From Multiple Robots (2006) by Birk and Carpin
Doesn't fully work, functionally runs but doesn't find the best transformation to complete the merge
"""

import numpy as np

n = 0 # size of matrix in X direction
m = 0 # size of matrix in Y direction

# takes in a matrix and a c value, returns the associated dmap
def dmap(m1,c):
    # set up array in same dimensions as m
    new_m1 = np.zeros([n,m])
    max = n*m + 1
    # convert m into 0 and max
    for x in range(n):
        for y in range(m):
            if m1[x][y] == c:
                new_m1[x][y] = 0
            else:
                new_m1[x][y] = max

    # first pass: update cell based on val and left and upper neighbors
    h = 0
    for x in range(-1,n-1):
        for y in range(-1,m-1):
            # h = min of left+1 and upper+1
            h = min(new_m1[x-1][y]+1, new_m1[x][y-1]+1)
            # dmap at location: min current and h
            new_m1[x][y] = min(h, new_m1[x][y])
    
    # second pass: update cell based on val and right and lower neighbors, start from bottom right corner
    # probably right in indices but we'll see
    for x in range(n-2,-2,-1):
        for y in range(m-2,-2,-1):
            # h = min of right+1 and lower+1
            h = min(new_m1[x+1][y]+1, new_m1[x][y+1]+1)
            # dmap at location: min current and h
            new_m1[x][y] = min(h, new_m1[x][y])
    return new_m1

# calculates d for two matrices and a c value
def d(m1, m2, c):
    # compute dmap for m2 c
    m2_dmap = dmap(m2,c)
    ret = 0
    count = 0
    for x in range(-1,n):
        for y in range(-1,m):
            if m1[x][y] == c:
                count += 1
                ret += m2_dmap[x][y]

    # ret = ret / num of c instances in m1
    if (count > 0):
        ret = ret / count
    return ret

# calculates the similarity between two matrices
def similarity(m1, m2):
    # sum of for all possible values of c
    total = 0
    for c in [0,1]:
        # calculate d(m1, m2, c)
        d1 = d(m1, m2, c)
        # calculate d(m2, m1, c)
        d2 = d(m2, m1, c)

        total += d1+d2
    return total

# finds the number of cells in the two matrices that agree (excltying unknown values)
def agr(m1, m2):
    # num of cells in m1,m2 that agree when directly overlapping
    total = 0
    for x in range(n):
        for y in range(m):
            # print(x,y,m1[x][y],m2[x][y])
            if m1[x][y] == m2[x][y] and m1[x][y] != -1:
                total += 1
    return total

# finds the number of cells in the two matrices that disagree (excltying unknown values)
def dis(m1, m2):
    # num of cells in m1,m2 that agree when directly overlapping
    total = 0
    for x in range(n):
        for y in range(m):
            # print(x,y,m1[x][y],m2[x][y])
            if m1[x][y] != m2[x][y] and m1[x][y] != -1 and m2[x][y] != -1:
                total += 1
    return total

# calculates the heuristic for a specific transformation
def delta(m1,m2):
    # similarity + clock(dis-agr)
    clock = 0.1
    sec_half = clock * (dis(m1,m2) - agr(m1,m2))
    total = similarity(m1,m2) + sec_half
    # print(similarity(m1,m2), sec_half, total)
    return total

# calculates the acceptance indicator
def ai(m1,m2):
    # anything above 98% is considered good
    agr_c = agr(m1,m2)
    dis_c = dis(m1,m2)
    ai = 1-(agr_c / (agr_c+dis_c))
    return ai

# generates the next guess based on previous guess, translation left/right, and translation up/down
def nextMatrix(prev, tx, ty, theta):
    # do transformation matrix as shown in article
    trans_matrix = np.array([[np.cos(theta),-np.sin(theta),tx],
                    [np.sin(theta),np.cos(theta),ty],
                    [0,0,1]])
    # print(trans_matrix)
    
    # result = np.ones([n,m]) * -1 # should these be -1s or what
    result = np.zeros([n,m])
    for x in range(n):
        for y in range(m):
            # print(x,y)
            a = np.array([[x], [y], [1]])
            r = trans_matrix.dot(a)
            # result[x][y] = r * prev[x][y]
            # print(r)
            new_x, xd = int(r[0]),r[0]-int(r[0])
            new_y, yd = int(r[1]),r[1]-int(r[1])
            # print(new_x, new_y)
            if (new_x < n and new_y < m and new_x > -1 and new_y > -1): 
                result[new_x][new_y] = ((xd*prev[x-1][y]) + ((1-xd)*prev[x][y]) + (yd*prev[x][y-1]) + ((1-yd)*prev[x][y]))/2
                result[new_x][new_y] = round(result[new_x][new_y])
                # print(new_x, new_y, result[new_x][new_y])
            # print("------")
    # problem: a bunch of the values get cut off cause they turn negative. have to figure out some way to shift the matrix over so it's normal
    # 
    return result

"""
input: two maps/occupancy grids of any size
output: the two maps combined (provided that there is some overlap), None otherwise
"""
def map_merging(m1, m2):
    # determine n and m based on biggest dimensions 
    global n,m
    if (len(m1) > n): 
        n = len(m1)
    if (len(m2) > n): 
        n = len(m2)
    if (len(m1[0]) > m): 
        m = len(m1[0])
    if (len(m2[0]) > m): 
        m = len(m2[0])
    print(n,m)

    # convert the 

    # perform random walk algorithm
    maxNumSteps = 20
    steps = 0
    maxNumTries = 1000
    tries = 0
    prev_m2 = m2
    prev_score = 0
    mean = [0,1,1] # need actual random values cause otherwise it won't go anywhere
    cov = [[1,0,0],[0,1,0],[0,0,1]]
    cov = np.array(cov)
    h = 3 # num of prev samples to be used for recalc covariance and mean
    # rs_threshold = 0.98
    samples = np.zeros([maxNumSteps,5])
    while (tries < maxNumTries and steps < maxNumSteps):
        tries += 1
        # print(cov)
        # generate new transformation -> prev_m2 + random new thing
        [theta, tx, ty] = np.random.multivariate_normal(mean,cov)
        tx = round(tx)
        ty = round(ty)
        theta = round(theta,2)
        # print(theta, tx, ty)
        s = nextMatrix(prev_m2, tx, ty, theta)

        # calc new score based on heuristic delta
        new_score = round(delta(m1,s))
        acceptance = round(ai(m1,s),2)
        # print(new_score)
        
        # random keep thing is here but makes the acceptance indicator worse :(
        # rs_val = np.random.random_sample()
        # print(rs_val, rs_threshold, " <------")
        if (new_score > prev_score):
            print("success!", new_score, prev_score)
            print(s)
            prev_m2 = s
            prev_score = new_score

            samples[steps] = [theta, tx, ty, new_score, acceptance]
            steps += 1
            # print(samples)

            # update covariance and mean
            # should h always be 10 or should it build up to 10? 
            # index has to also be taken into account when indexing into samples
            # either first three samples or steps-3:steps
            if (steps < h):
                last_h_vals = samples[:h]
            else:
                last_h_vals = samples[steps-h:steps]
            cov = np.cov(last_h_vals)
            print(cov)
            mean = last_h_vals.mean(1)
            # cov is going from being a 1x3 array to being a 10x10 array
            # because we're giving it 10 samples so it's returning a 10x10 array
    print("--------")
    print(steps, tries)
    print(samples)
    print(prev_m2)
            
    # at this point prev_m2 should be the best

    # then actually merge the maps together... lol. 
    # using transform from best option, transform m2 again but don't cut off any edges that get moved around

    # then let's run through each coordinate
    # if either is -1 and the other is a value set to value
    # take average of two values if both are real
    # if either is 0 or 1 set to that? 




    pass

m1 = [[0,1,1,-1,5,10],
      [0,2,2,30,4,5],
      [0,3,3,10,-1,6],
      [0,4,4,-1,-1,-1],
      [0,5,5,0,0,0],
      [0,6,6,0,0,0]]

m2 = [[0,0,1,1,-1,5],
      [0,0,2,2,30,4],
      [0,0,3,3,10,-1],
      [0,0,4,4,-1,-1],
      [0,0,5,5,0,0],
      [0,0,6,6,0,0]]

# m1 = [[0,0,0,0,0,100],
#       [0,0,0,0,0,100],
#       [0,0,0,0,0,100],
#       [0,0,0,0,0,100],
#       [0,0,0,0,0,100],
#       [0,0,0,0,0,100]]

# m2 = [[100,0,0,0,0,0],
#       [100,0,0,0,0,0],
#       [100,0,0,0,0,0],
#       [100,0,0,0,0,0],
#       [100,0,0,0,0,0],
#       [100,0,0,0,0,0]]

map_merging(m1,m2)

# print(agr(m1,m2))
# print(dis(m1,m2))
# print(ai(m1,m2))
# print(dmap(m1,1))
# print(dmap(m2,0))
# print(d(m1,m2,0))
# print(d(m1,m2,1))
# print(similarity(m1,m1))
# print(delta(m1,m1))