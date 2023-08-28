import random
import math

def rrt(start, goal):
    max_iter=10
    nodes = [(start[0], start[1], None)]
    
    for _ in range(max_iter):
        x_rand, y_rand = get_random_point()
        nearest_node_idx = find_nearest_node(nodes, x_rand, y_rand)
        nearest_node = nodes[nearest_node_idx]
        new_node = extend(nearest_node, x_rand, y_rand)
        
        if new_node is not None:
            new_node = (new_node[0], new_node[1], nearest_node_idx)
            nodes.append(new_node)
            if distance(new_node, goal) <= 1.0:
                goal_parent_idx = len(nodes) - 1
                goal = (goal[0], goal[1], goal_parent_idx)
                return get_path(nodes, goal)
        
    return None

def get_random_point():
    x_rand = round(random.uniform(-5, 5), 1)
    y_rand = round(random.uniform(-5, 5), 1)
    return x_rand, y_rand

def find_nearest_node(nodes, x_rand, y_rand):
    distances = [(node[0] - x_rand) ** 2 + (node[1] - y_rand) ** 2 for node in nodes]
    nearest_node_idx = min(range(len(distances)), key=distances.__getitem__)
    return nearest_node_idx

def extend(node, x, y):
    max_distance = 0.15
    theta = math.atan2(y - node[1], x - node[0])
    x_new = node[0] + max_distance * math.cos(theta)
    y_new = node[1] + max_distance * math.sin(theta)
    
    new_node = (x_new, y_new)
    return new_node

def distance(node1, node2):
    dx = node2[0] - node1[0]
    dy = node2[1] - node1[1]
    return math.sqrt(dx**2 + dy**2)

def get_path(nodes, goal):
    path = []
    node_idx = goal[2]
    while node_idx is not None:
        node = nodes[int(node_idx)]
        path.append((node[0], node[1]))
        node_idx = node[2]
    path.reverse()
    #path.append(goal[:-1])
    return path[1:]