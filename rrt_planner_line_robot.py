import time
import random
import drawSample
import math
import sys
import imageToRects
import utils
import numpy



def redraw(canvas):
    canvas.clear()
    canvas.markit( tx, ty, r=SMALLSTEP )
    drawGraph(G, canvas)
    for o in obstacles: canvas.showRect(o, outline='blue', fill='blue')
    canvas.delete("debug")


def drawGraph(G, canvas):
    global vertices,nodes,edges
    if not visualize: return
    for i in G[edges]:
       canvas.polyline(  [vertices[i[0]], vertices[i[1]] ]  )


def genPoint():
    if args.rrt_sampling_policy == "uniform":
        # Uniform distribution
        x = random.random()*XMAX
        y = random.random()*YMAX
    elif args.rrt_sampling_policy == "gaussian":
        # Gaussian with mean at the goal
        x = random.gauss(tx, sigmax_for_randgen)
        y = random.gauss(ty, sigmay_for_randgen)
    else:
        print ("Not yet implemented")
        quit(1)

    bad = 1
    while bad:
        bad = 0
        if args.rrt_sampling_policy == "uniform":
            # Uniform distribution
            x = random.random()*XMAX
            y = random.random()*YMAX
        elif args.rrt_sampling_policy == "gaussian":
            # Gaussian with mean at the goal
            x = random.gauss(tx, sigmax_for_randgen)
            y = random.gauss(ty, sigmay_for_randgen)
        else:
            print ("Not yet implemented")
            quit(1)
        # range check for gaussian
        if x<0: bad = 1
        if y<0: bad = 1
        if x>XMAX: bad = 1
        if y>YMAX: bad = 1
    return [x,y,tt]

def returnParent(k, canvas):
    """ Return parent note for input node k. """
    for e in G[edges]:
        if e[1]==k:
            canvas.polyline(  [vertices[e[0]], vertices[e[1]] ], style=3  )
            return e[0]
     
     

def genvertex():
    vertices.append( genPoint() )
    return len(vertices)-1

def pointToVertex(p):
    vertices.append( p )
    return len(vertices)-1

def pickvertex():
    return random.choice( range(len(vertices) ))


def lineFromPoints(p1,p2):
    
    x1 = p1[0]
    x2 = p2[0]
    y1 = p1[1]
    y2 = p2[1]
    if (x2 - x1) == 0:
        run = None #infinite line
    else:
        run = (x2 - x1) #calculate slope
        
    if run is not None:
        rise = (y2 - y1)
        
    else:
        rise = None
    
    
    return (rise, run) #returns a tuple contianing the rise and run of the slope of a line

def pointPointDistance(p1,p2):
    return (((p1[0] - p2[0])**2) + ((p1[1] - p2[1])**2)) ** 0.5 #returns the Euclidian distance between points

def closestPointToPoint(G,p2):
    cp = None
    min_value = float('inf') #initialize to a large value
    
    for vertex in range(len(G[nodes])):
        d = pointPointDistance(vertices[(G[nodes][vertex])], p2)
        if d < min_value: #gradually decrease the size of the closest point
            min_value = d
            cp = G[nodes][vertex]
            
    #return vertex index
    return cp


def crossProd(a, b):
    result = [0] * len(a)

    for i in range(len(a)):
        j = (i + 1) % len(a)
        k = (i + 2) % len(a)
        result[i] = a[j] * b[k] - a[k] * b[j]

    return result

def are_arrays_equal(arr1, arr2):

    # Compare each element of the arrays
    for i in range(len(arr1)):
        if arr1[i] != arr2[i]:
            return False

    return True


def signCheck(arr): #computes sign of each element in an array
    result = [0] * len(arr)
    for i in range(len(arr)):
        if arr[i] < 0:
            result[i] = -1
        
        elif arr[i] > 0:
            result[i] = 1
    
    return result



def lineHitsRect(p1,p2,r):
    robot_vect = [(p2[0] - p1[0]), (p2[1] - p1[1])]
    ob_vert = [(r[0], r[1]), (r[0], r[3]), (r[2], r[1]), (r[2], r[3])]
    ob_edges = [(ob_vert[0], ob_vert[1]), (ob_vert[1], ob_vert[3]), (ob_vert[0], ob_vert[2]), (ob_vert[2], ob_vert[3])]
    
    for edge in ob_edges:
        edge_vector = [(edge[1][0] - edge[0][0]), (edge[1][1] - edge[0][1])]
        vect1_a = [(edge[0][0] - p1[0]), (edge[0][1] - p1[1])]
        vect1_b = [(edge[1][0] - p1[0]), (edge[1][1] - p1[1])]
        cross_robot_vect1a = crossProd(robot_vect, vect1_a) #compute cross product between the robot position vector and upper/lower left side coords of an obstacle for p1
        cross_robot_vect1b = crossProd(robot_vect, vect1_b) #computes cross product between the robot position vector and upper/lower right side coords of an obstacle for p1
        
        vect2_a = [(p2[0] - edge[0][0]), (p2[1] - edge[0][1])]
        vect2_b = [(p1[0] - edge[0][0]), (p1[1] - edge[0][1])]
        
        cross_edge_vect2a = crossProd(edge_vector, vect2_a) #computes the cross product of endpoint and edge of an obstacle
        cross_edge_vect2b = crossProd(edge_vector, vect2_b) #computes the cross product of start point and edge of an obstacle
        
        if (not(are_arrays_equal(signCheck(cross_robot_vect1a), signCheck(cross_robot_vect1b))) and (not(are_arrays_equal(signCheck(cross_edge_vect2a), signCheck(cross_edge_vect2b))))):
            return True
    
    return False

def inRect(p,rect,dilation):
    if (rect[0] - dilation) <= p[0] <= (rect[2] + dilation) and (rect[1] - dilation) <= p[1] <= (rect[3] + dilation):
        return True
    else:
        return False


def addNewPoint(p1, p2, stepsize):
    closest_point_location = vertices[p1] #find location of the new closest point
    
    cpx = closest_point_location[0] #x and y values of closest
    cpy = closest_point_location[1]
    
    unit_vector_between_points = lineFromPoints(closest_point_location, p2) #calculate unit vector
    run = unit_vector_between_points[1]
    rise = unit_vector_between_points[0]
    
    distance_between = pointPointDistance(closest_point_location, p2)
    
    angle = math.atan2(unit_vector_between_points[1], unit_vector_between_points[0])
    
    if distance_between != 0:
        run = (run/distance_between) * stepsize
        rise = (rise/distance_between) * stepsize
    
    new_p = ((cpx + run), (cpy + rise), angle) #calculate new point by increasing the slope by stepsize
    return new_p

def getEndPoint(p,length):
    return [p[0] + math.cos(p[2])*length,p[1] + math.sin(p[2])*length]


def outOfBounds(point):
    if point[0] < 0 or point[1] < 0 or point[0] > XMAX or point[1] > YMAX:
        return True
    return False
    
    


def rrt_search(G, tx, ty, canvas):
    # Please carefully read the comments to get clues on where to start
    # TODO
    # Fill this function as needed to work ...
    global sigmax_for_randgen, sigmay_for_randgen
    n = 0
    nsteps = 0
    while 1:  # Main loop
        # This generates a point in form of [x,y] from either the normal dist or the Gaussian dist
        p = genPoint()

        # This function must be defined by you to find the closest point in the existing graph to the guiding point
        cp = closestPointToPoint(G, p)
        v = addNewPoint(cp, p, SMALLSTEP)
        
        endpoint1 = getEndPoint(v, (ROBOTSIZE/2))
        endpoint2 = getEndPoint(v, (-1*ROBOTSIZE/2))

        if visualize:
            # if nsteps%500 == 0: redraw()  # erase generated points now and then or it gets too cluttered
            n = n + 1
            if n > 10:
                canvas.events()
                n = 0
                
        reject_point = False

        for o in obstacles:
            # The following function defined by you must handle the occlusion cases
            if lineHitsRect(vertices[cp], v, o) or inRect(v, o, 1) or lineHitsRect(endpoint1,endpoint2,o) or outOfBounds(v) or outOfBounds(endpoint2) or outOfBounds(endpoint1) or inRect(endpoint1, o, 1) or inRect(endpoint2, o, 1):
                reject_point = True
                break
        
        if reject_point:
            continue

        k = pointToVertex(v)  # is the new vertex ID
        G[nodes].append(k)
        G[edges].append((cp, k))
        if visualize:
            canvas.polyline([vertices[cp], vertices[k]], 5)
            canvas.polyline([endpoint1, endpoint2], 1)

        if pointPointDistance(endpoint2, [tx, ty]) < SMALLSTEP or pointPointDistance(endpoint1, [tx, ty]) < SMALLSTEP:
            print("Target achieved.", len(G[nodes]), "nodes in entire tree")
            if visualize:
                t = pointToVertex([tx, ty])  # is the new vertex ID
                G[edges].append((k, t))
                if visualize:
                    canvas.polyline([p, vertices[t]], 2)
                # while 1:
                #     # backtrace and show the solution ...
                #     canvas.events()
                nsteps = 0
                totaldist = 0
                while 1:
                    oldp = vertices[k]  # remember point to compute distance
                    k = returnParent(k, canvas)  # follow links back to root.
                    canvas.events()
                    if k <= 1: break  # have we arrived?
                    nsteps = nsteps + 1  # count steps
                    totaldist = totaldist + pointPointDistance(vertices[k], oldp)  # sum lengths
                print("Path length", totaldist, "using", nsteps, "nodes.")

                global prompt_before_next
                if prompt_before_next:
                    canvas.events()
                    print("More [c,q,g,Y]>")
                    d = sys.stdin.readline().strip().lstrip()
                    print("[" + d + "]")
                    if d == "c": canvas.delete()
                    if d == "q": return
                    if d == "g": prompt_before_next = 0
                break


def main():
    #seed
    random.seed(args.seed)
    if visualize:
        canvas = drawSample.SelectRect(xmin=0,ymin=0,xmax=XMAX ,ymax=YMAX, nrects=0, keepcontrol=0)#, rescale=800/1800.)
        for o in obstacles: canvas.showRect(o, outline='red', fill='blue')
    while 1:
        redraw(canvas)
        G[edges].append( (0,1) )
        G[nodes].append(1)
        if visualize: canvas.markit( tx, ty, r=SMALLSTEP )

        drawGraph(G, canvas)
        rrt_search(G, tx, ty, canvas)

    if visualize:
        canvas.mainloop()


if __name__ == '__main__':
    # display = drawSample.SelectRect(imfile=im2Small,keepcontrol=0,quitLabel="")
    args = utils.get_args()
    visualize = utils.get_args()
    drawInterval = 100  # 10 is good for normal real-time drawing

    prompt_before_next = 1  # ask before re-running sonce solved
    SMALLSTEP = args.step_size  # what our "local planner" can handle.

    SMALLTHETA = 5  # degrees
    
    ROBOTSIZE = args.robot_length

    map_size, obstacles = imageToRects.imageToRects(args.world)
    # Note the obstacles are the two corner points of a rectangle
    # Each obstacle is (x1,y1), (x2,y2), making for 4 points
    XMAX = map_size[0]
    YMAX = map_size[1]

    G = [[0], []]  # nodes, edges
    vertices = [[args.start_pos_x, args.start_pos_y], [args.start_pos_x, args.start_pos_y + 10]]

    # goal/target
    tx = args.target_pos_x
    ty = args.target_pos_y
    tt = args.target_pos_theta

    # start
    sigmax_for_randgen = XMAX / 2.0
    sigmay_for_randgen = YMAX / 2.0
    nodes = 0
    edges = 1
    main()
