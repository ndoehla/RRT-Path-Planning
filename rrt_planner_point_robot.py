'''
Hello and welcome to the first assignment :)
Please try to enjoy implementing algorithms you learned from the course; in this regard, we also have tried
to provide a good starting point for you to develop your own code. In this script, you may find different functions
that should be defined by you to perform specific tasks. A good suggestion would be to begin with the primary
function for the RRT algo named as "rrt_search". Following this function line by line you may realize how to define
each function to make it work!
Note, you may use the following commands to run this script with the required arguments:
python3 rrt_planner_point_robot.py --arg1_name your_input1 --arg2_name your_input2 e.g.
python3 rrt_planner_point_robot.py --world="shot.png"
To see the list of arguments, take a look at utils.py
Also note:
G is a list containing two groups: the first group includes the nodes IDs (0,1,2,...), while the second holds all pairs of nodes creating an edge
Vertices is a list of nodes locations in the map; it corresponds to the nodes' IDs mentioned earlier
GOOD LUCK!
'''

import random
import drawSample
import sys
import imageToRects
import utils
import math


def redraw(canvas):
    canvas.clear()
    canvas.markit(tx, ty, r=SMALLSTEP)
    drawGraph(G, canvas)
    for o in obstacles: canvas.showRect(o, outline='blue', fill='blue')
    canvas.delete("debug")


def drawGraph(G, canvas):
    global vertices,nodes,edges
    if not visualize: return
    for i in G[edges]:
        # e.g. vertices: [[10, 270], [10, 280]]
        canvas.polyline(  [vertices[i[0]], vertices[i[1]] ]  )


# Use this function to generate points randomly for the RRT algo
def genPoint():
    # if args.rrt_sampling_policy == "uniform":
    #     # Uniform distribution
    #     x = random.random()*XMAX
    #     y = random.random()*YMAX
    # elif args.rrt_sampling_policy == "gaussian":
    #     # Gaussian with mean at the goal
    #     x = random.gauss(tx, sigmax_for_randgen)
    #     y = random.gauss(ty, sigmay_for_randgen)
    # else:
    #     print ("Not yet implemented")
    #     quit(1)

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
    return [x,y]

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
    vertices.append(p)
    return len(vertices)-1

def pickvertex():
    return random.choice( range(len(vertices) ))

def lineFromPoints(p1,p2):
    
    x1, y1 = p1
    x2, y2 = p2
    
    if (x2 - x1) == 0:
        run = None #infinite line
    else:
        run = (x2 - x1) #calculate slope
        
    if run is not None:
        rise = (y2 - y1)
        
    else:
        rise = None
    
    
    return (rise, run) #returns a tuple contianing the rise and run of the slope of a line


def pointPointDistance(p1,p2): #done yay
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


def lineHitsRect(p1,p2,r): #inprogress
    
    rise, run = lineFromPoints(p1, p2)
    slope = rise / run
    yInt = p1[1] - (slope * p1[0])
    
    
    if r[0] <= ((r[1]-yInt)/slope) <= r[2]: #line intersects top of obstacle
        return True
    
    if r[0] <= ((r[3]-yInt)/slope) <= r[2]: #line intersects bottom of obstacle
        return True
    
    
    if r[1] <= ((slope * r[0]) + yInt) <= r[3]: #line intsersects left side of obstacle
        return True
    
    if r[1] <= ((slope * r[2]) + yInt) <= r[3]:
        return True
    
    
    return False

def inRect(p,rect,dilation):
    """ Return 1 in p is inside rect, dilated by dilation (for edge cases). """
    #TODO
    px = p[0]
    py = p[1]
    
    x1 = rect[0] - dilation
    x2 = rect[2] + dilation
    y1 = rect[1] - dilation
    y2 = rect[3] + dilation
    
    if x1 <= px <= x2 and y1 <= py <= y2:
        return True
    return False

def addNewPoint(p1,p2,stepsize):
    closest_point_location = vertices[p1] #find location of the new closest point
    
    cpx = closest_point_location[0] #x and y values of closest
    cpy = closest_point_location[1]
    
    unit_vector_between_points = lineFromPoints(closest_point_location, p2) #calculate unit vector
    run = unit_vector_between_points[1]
    rise = unit_vector_between_points[0]
    
    distance_between = pointPointDistance(closest_point_location, p2)
    
    if distance_between != 0:
        run = (run/distance_between) * stepsize
        rise = (rise/distance_between) * stepsize
    
    new_p = ((cpx + run), (cpy + rise)) #calculate new point by increasing the slope by stepsize
    return new_p

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

        if visualize:
            # if nsteps%500 == 0: redraw()  # erase generated points now and then or it gets too cluttered
            n = n + 1
            if n > 10:
                canvas.events()
                n = 0
                
        reject_point = False

        for o in obstacles:
            # The following function defined by you must handle the occlusion cases
            if lineHitsRect(vertices[cp], v, o) or inRect(v, o, 1):
                reject_point = True
                break
                # ... reject
        
        if reject_point:
            continue

        k = pointToVertex(v)  # is the new vertex ID
        G[nodes].append(k)
        G[edges].append((cp, k))
        if visualize:
            canvas.polyline([vertices[cp], vertices[k]])

        if pointPointDistance(v, [tx, ty]) < SMALLSTEP:
            print("Target achieved.", len(G[nodes]), "nodes in entire tree")
            if visualize:
                t = pointToVertex([tx, ty])  # is the new vertex ID
                G[edges].append((k, t))
                if visualize:
                    canvas.polyline([p, vertices[t]], 1)
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
        # graph G
        redraw(canvas)
        G[edges].append( (0,1) )
        G[nodes].append(1)
        if visualize: canvas.markit( tx, ty, r=SMALLSTEP )
        drawGraph(G, canvas)
        args.rrt_sampling_policy = "gaussian"
        rrt_search(G, tx, ty, canvas)

    if visualize:
        canvas.mainloop()

if __name__ == '__main__':

    args = utils.get_args()
    visualize = utils.get_args()
    drawInterval = 10  # 10 is good for normal real-time drawing

    prompt_before_next = 1  # ask before re-running sonce solved
    SMALLSTEP = 20  # what our "local planner" can handle.
    map_size, obstacles = imageToRects.imageToRects(args.world)
    # Note the obstacles are the two corner points of a rectangle (left-top, right-bottom)
    # Each obstacle is (x1,y1), (x2,y2), making for 4 points

    XMAX = map_size[0]
    YMAX = map_size[1]
    # The boundaries of the world are (0,0) and (XMAX,YMAX)

    G = [[0], []]  # nodes, edges
    vertices = [[args.start_pos_x, args.start_pos_y], [args.start_pos_x, args.start_pos_y + 10]]

    # goal/target
    tx = args.target_pos_x
    ty = args.target_pos_y

    # start
    sigmax_for_randgen = XMAX / 2.0
    sigmay_for_randgen = YMAX / 2.0
    nodes = 0
    edges = 1

    main()
