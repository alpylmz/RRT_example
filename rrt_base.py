import numpy as np


class Node:
    def __init__(self,_parent,_coordinates,_cost): # _parent = 0 means root
        self.parent = _parent
        self.children = []
        self.coordinates = _coordinates
        self.cost = _cost
        return


    def addChildren(self,_children):
        self.children.append(_children)
        return

    def getChildren(self):
        return self.children

    def removeChildren(self, _conf):
        i=0
        while i < len(self.children):
            if np.allclose(self.children[i].coordinates,_conf):
                self.children.pop(i)
            i+=1

    def setParent(self, _parent):
        self.parent = _parent

    def setCost(self,_cost):
        self.cost = _cost
        



class GoalBiasedGreedySteerKNeighborhoodRRTStarBase:
    def __init__(self, seed):
        '''Constructor with seed, to be supplied to the np.random.RandomState object held inside. Feel free to add things.'''
        self.random = np.random.RandomState(seed)
        # this holds 0 if empty, Node if exists
        self.root = 0
        # this holds 0 if uninitialized, c_goal if exists
        self.goal_coordinates = 0

    def distance(self, c1, c2):
        '''returns the distance between two configurations c1, c2.'''
        pass

    def steer(self, c0, c, step_size):
        '''starting from the configuration c0, gets closer to
        c in discrete steps of size step_size in terms of distance(c0, c). returns the last no-collision
        configuration. If no collisions are hit during steering, returns c2. If
        steering is not possible at all, returns None.'''
        pass
    def allclose(self, c1, c2):
        '''returns True if two configurations are numerically very close (just use np.allclose default values).'''
        pass

    def sample(self, p):
        '''either returns goal configuration with some goal bias probability p or returns
        a no collision configuration sample with 1-p. The goal bias becomes 0 as soon as goal node is found.'''
        pass

    def returnsecond(self,e):
        return e[1]

    def neighborhood(self, c, k): 
        '''returns a list of k closest nodes to configuration c in terms of distance(q.value, c)'''
        # traverse all nodes 
        if self.root == 0: # If root does not exist
            return []

        shortest_ones = [(self.root,self.distance(self.root.coordinates,c))]
        queue = [self.root]

        while queue:
            node = queue.pop()
            queue += node.getChildren()
            dist = self.distance(c,node.coordinates)
            if((dist) < (shortest_ones[-1][1])):
                if len(shortest_ones) == k:
                    shortest_ones.pop()
                    shortest_ones.append((node,dist))
                    shortest_ones.sort(key=self.returnsecond)
                else:
                    shortest_ones.append((node,dist))
                    shortest_ones.sort(key=self.returnsecond)
        
        # I need to return the nodes, not the distances
        answer = []
        for i in shortest_ones:
            answer.append(i[0])
        
        
        return answer
                



        

    def init_rrt(self, c_init, c_goal): 
        '''initializes/resets rrt with the root node holding c_init and goal configuration c_goal.'''
        #initialize the root
        self.root = Node(0,c_init,0)
        self.goal_coordinates = c_goal


    def valid(self, c):
        '''returns True if configuration c is non-colliding.'''
        pass
    def collision_free(self, c1, c2, step_size):
        '''returns True if the linear trajectory between c1 and c2 are collision free.'''
        pass



    def add_node(self, p, k, step_size): 
        '''adds a node to the rrt with goal bias probability p, near function with k closest neighbors,
        and step_size for greedy steering. returns the new configuration that is now part of the tree.'''
        new_c = self.sample(p)

        x_nearest = self.neighborhood(new_c, 1)
        # new node configuration
        new_node_c = self.steer(x_nearest[0].coordinates, new_c, step_size)

        if new_node_c is None:
            # that means steering is not possible
            # in this case we will sample a new point
            return self.add_node(p,k,step_size)

            # try return None here and see what happens!
            # return None

        if self.collision_free(new_node_c, x_nearest[0].coordinates,step_size):
            x_min = x_nearest[0]
        
            c_min = self.Cost(x_nearest[0])+ self.distance(x_nearest[0].coordinates, new_node_c) #line 9

            # k near neighbors
            near_ones = self.neighborhood(new_node_c,k) # line 7
        
            x_min = None
            c_min = 999999

            #connect along a minimum cost path
            for near in near_ones:
                if self.collision_free(near.coordinates,new_node_c,step_size) and (self.Cost(near) + self.distance(near.coordinates,new_node_c) < c_min):
                    x_min = near
                    c_min = self.Cost(near) + self.distance(near.coordinates,new_node_c)

            if x_min is None:
                return None
            
            new_node = Node(x_min,new_node_c,x_min.cost + self.distance(new_node_c, x_min.coordinates))
            x_min.addChildren(new_node) #line 13

            near_ones = self.neighborhood(new_node_c,k) # line 7
            #rewire the tree
            for near in near_ones:
                #print(c_min,self.distance(near.coordinates, new_node_c),self.Cost(near.coordinates) )
                if self.collision_free(new_node_c,near.coordinates,step_size) and c_min + self.distance(near.coordinates, new_node_c) < self.Cost(near):
                    cur_parent = near.parent
                    if cur_parent != 0:
                        cur_parent.removeChildren(near.coordinates)
                    
                    near.setParent(new_node)
                    new_node.addChildren(near)
                    near.setCost(c_min + self.distance(near.coordinates, new_node_c))
            

        # returns the new configuration
        return new_node_c

    def Cost(self,node):
        return node.cost


    def get_path_to_goal(self): 
        '''returns the path to goal node as a list of tuples of configurations[(c_init, c1),(c1, c2),...,(cn,c_goal)].
        If the goal is not reachable yet, returns None.'''
        path = self.look_for_goal(self.root)
        return path
        
    def look_for_goal(self,node):
        children = node.getChildren()

        for i in children:
            #print(i.coordinates)
            if np.allclose(i.coordinates,self.goal_coordinates):
                #print("found")
                return [(node.coordinates,i.coordinates)]
            
            x = self.look_for_goal(i)
            if x:
                return [(node.coordinates,i.coordinates)] + x #if goal found, return the path
        # path not found
        return None


    def is_goal_reachable(self): 
        '''returns True if goal configuration is reachable.'''
        if self.get_path_to_goal() == None:
            return False
        return True


    def simplify_path(self, path, step_size): 
        '''greedily removes redundant edges from a configuration path represented as a list of tuples
        of configurations [(c_init,c1),(c1,c2),(c2,c3)...(cn,c_goal)], as described
        Principles of Robot Motion, Theory, Algorithms and Implementations (2005), p. 213,
        Figure 7.7.(use the default version, always try to connect to c_goal, not the other way around'''
        
        i = 1

        while i < len(path):
            to = None
            j = 0
            while j <= (len(path) - i):
                if self.collision_free(path[(len(path) - i)][1],path[j][0],step_size):
                    to = j
                    break
                j+=1
            
            if not to:
                path[to] = (path[to][0], path[(len(path) - i)][1]) # reconnected these
                # now, need to delete nodes between them
                del path[to+1:(len(path) - i)+1]

            i+=1   
            
        return path

    
    

    def get_all_edges(self): 
        '''returns all of the edges in the tree as a list of tuples of configurations [(c1,c2),(c3,c4)...]. The
        order of the edges, The order of edges in the list and their direction is not important.'''
        answer = []
        queue = [self.root]
        while queue:
            node = queue.pop()
            children_of_node = node.getChildren()
            queue += children_of_node
            #first, put the edges between the node and its children
            for i in children_of_node:
                answer += [(node.coordinates, i.coordinates)]
            #last, pop a value from list, and continue from there
            

        return answer


    def get_goal_cost(self): 
        '''returns the non-simplified goal path length in terms of distance. Returns np.Inf if goal is not reachable.'''
        path = self.get_path_to_goal()
        if not path:
            return np.Inf
        
        cost = 0
        for i in path:
            cost += self.distance(i[0], i[1])
        return cost