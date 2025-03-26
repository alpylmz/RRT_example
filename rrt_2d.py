import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.collections import LineCollection
from rrt_base import GoalBiasedGreedySteerKNeighborhoodRRTStarBase

# CIRCLE_OBSTACLES = [(center_x,center_y,radius),...]
CIRCLE_OBSTACLES = [(-1,0.5,0.5),(1.0,1.0,0.5),(-0.8,-0.8,1),(1,0,0.6),(1,-1,1)]
XY_MIN = -2
XY_MAX = 2
SEED = 42
DOF = 2



class RRTStar2D(GoalBiasedGreedySteerKNeighborhoodRRTStarBase):
    def __init__(self,seed):
        GoalBiasedGreedySteerKNeighborhoodRRTStarBase.__init__(self,seed)

    # Sample a random point in the free space if the goal is reachable
    def sample(self, p):
        if not self.is_goal_reachable() and self.random.random_sample() < p:
            return self.goal_coordinates
        else:
            return self.random.uniform(low=XY_MIN, high=XY_MAX, size=(DOF,))

    def distance(self, c1, c2):
        return np.sqrt(np.sum((c1-c2)**2))

    # If the point is valid or not
    def valid(self, c):
        for obs in CIRCLE_OBSTACLES:
            if np.sqrt((c[0]-obs[0])**2+(c[1]-obs[1])**2) < obs[2]:
                return False
        return True

    def allclose(self,c1, c2):
        return np.allclose(c1,c2)
    
    # Steer from c0 to c in steps of step_size
    # At each step, check if the edge is collision free
    # This allows us to have a direct path from c0 to c if there is no collision along the way
    def steer(self, c0, c, step_size):
        
        dist = self.distance(c0,c)
        is_stepped = False
        tot_step = 0.0
        curr_step = c0


        while True:
            # arrived at target
            if self.allclose(c,c0):
                break
                
            if tot_step >= 1.0:
                curr_step = c #in final target
                break
            
            new_step = curr_step + (c-c0) * (step_size / dist)

            if self.distance(new_step,c) < step_size: #arrived at target
                if not self.valid(c):
                    break
            else:
                if not self.collision_free(c0,new_step, step_size):
                    break

            # we made a step
            is_stepped = True
            curr_step = new_step

            tot_step += step_size

        if is_stepped:
            return curr_step
        return None
    
    # Check if the new edge is collision free
    # It does that by setting up the equation for the distance from the circle to the line
    # \delta = B^2 - A(Ca - r^2)
    def collision_free(self, c1, c2, step_size):
        if self.distance(c1, c2) < step_size: # this speeds up the computation, but is not necessary
            return self.valid(c2)
        for obstacle in CIRCLE_OBSTACLES:
            start = c1
            vec = c2 - c1
            center = np.array([obstacle[0], obstacle[1]])
            radius = obstacle[2]

            B = (np.dot(vec, start - center))
            A = (np.dot(vec, vec))
            Ca = np.dot(start - center, start - center)
            delta = B**2 - A * (Ca - radius**2)
            if self.intersect(A, B, delta):
                return False

        return True

    # Check if the edge intersects with the circle
    def intersect(self, A, B, delta):
        
        if delta < 0:
            return False
        t1 = (-B + np.sqrt(delta))
        t2 = (-B - np.sqrt(delta))

        t1 /= A
        t2 /= A

        if (1.0 >= t1 >= 0.0) or (1.0 >= t2 >= 0.0):
            return True


        return False



if __name__ == "__main__":
    c_init = np.array([0,0])
    c_goal = np.array([0,-1.5])
    p = 0.5
    k = 20
    step_size = 0.1

    rrt = RRTStar2D(SEED)
    rrt.init_rrt(c_init, c_goal)

    fig, ax = plt.subplots()

    for obs in CIRCLE_OBSTACLES:
        ax.add_patch(plt.Circle((obs[0],obs[1]), radius=obs[2], fc="y"))

    edge_lines = LineCollection([],colors=[(0,0,1)],linewidth=0.5)
    goal_path = LineCollection([],colors=[(1,0,0)])
    simplified_goal_path = LineCollection([],colors=[(0,1,0)])

    ax.add_collection(edge_lines)
    ax.add_collection(goal_path)
    ax.add_collection(simplified_goal_path)
    ax.set_aspect("equal")
    ax.set_xlim(XY_MIN, XY_MAX)
    ax.set_ylim(XY_MIN, XY_MAX)
    ax.scatter(c_init[0], c_init[1], facecolor="r")
    ax.scatter(c_goal[0], c_goal[1], facecolor="g")
    title = ax.text(0.5,0.95, "", bbox={'facecolor':'w', 'alpha':0.5, 'pad':3},
                transform=ax.transAxes, ha="center")
    paused = False
    def animate(frame_no):
        rrt.add_node(p,k,step_size)
        if rrt.is_goal_reachable():
            goal_edges = rrt.get_path_to_goal()
            goal_path.set_segments(goal_edges)
            simplified_goal_path.set_segments(rrt.simplify_path(goal_edges, step_size))
        edge_lines.set_segments(rrt.get_all_edges())
        title.set_text("p=%s, k=%s, step_size=%s, nodes=%s goal_cost=%.2f"%(p,k,step_size,frame_no+1,rrt.get_goal_cost()))
        return edge_lines,goal_path,title,simplified_goal_path
    #if you want to animate, uncomment (disable offline calculation and plot below first)
    #anim = animation.FuncAnimation(fig, animate,frames=3000, interval=1, blit=True)

    def toggle_pause(self):
        global paused
        paused = not paused
        if paused:
            anim.event_source.stop()
        else:
            anim.event_source.start()

    #fig.canvas.mpl_connect('button_press_event', toggle_pause) if you want to pause in certain situations

    #calculate offline and plot after:
    NUM_NODES = 1000
    for i in range(NUM_NODES):
        rrt.add_node(p,k,step_size)

    edge_lines.set_segments(rrt.get_all_edges())
    goal_edges = rrt.get_path_to_goal()
    goal_path.set_segments(goal_edges)
    simplified_goal_path.set_segments(rrt.simplify_path(goal_edges, step_size))
    title.set_text("p=%s, k=%s, step_size=%s, nodes=%s, goal_cost=%.2f"%(p,k,step_size,NUM_NODES,rrt.get_goal_cost()))
    fig.set_size_inches(8,8)
    plt.show()