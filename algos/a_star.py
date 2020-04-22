from queue import PriorityQueue
grid = None
# w x h
grid_dimensions = None
node_dict = {}

class Node:
    def __init__(self, coordinates, parent, path_cost, priority):
        self.coordinates = coordinates
        self.parent = parent
        self.path_cost = path_cost
        self.priority = priority
        node_dict[coordinates] = self
    # def __cmp__(self, object):
    #     return cmp(self.cost, object.cost)

    def __lt__(self, other):
        return self.priority < getattr(other, 'priority', other)

    def __eq__(self, other):
        return self.priority == getattr(other, 'priority', other)

def backtrack(node, path=[]):
    if node == None:
        return path
    else:
        path = [node.coordinates] + path
        return backtrack(node.parent, path)

def check_if_valid(coordinates):
    #will have to adjust in the final code 
    x_valid = False
    y_valid = False
    if(coordinates[0] >= 0 and coordinates[0] <= grid_dimensions[1]-1):
        x_valid = True
    
    if(coordinates[1] >= 0 and coordinates[1] <= grid_dimensions[0]-1):
        y_valid = True

    if x_valid and y_valid and grid[coordinates[0]][coordinates[1]] != 1:
        return True
    else:
        return False
        
def find_four_way_successors(coordinates):
    successors = []
    s1 = (coordinates[0]+1, coordinates[1])
    if check_if_valid(s1):
        successors.append(s1)

    s2 = (coordinates[0]-1, coordinates[1])
    if check_if_valid(s2):
        successors.append(s2)

    s3 = (coordinates[0], coordinates[1]+1)
    if check_if_valid(s3):
        successors.append(s3)

    s4 = (coordinates[0], coordinates[1]-1)
    if check_if_valid(s4):
        successors.append(s4)

    return successors

def find_eight_way_successors(coordinates):
    successors = []
    s1 = (coordinates[0]+1, coordinates[1])
    if check_if_valid(s1):
        successors.append(s1)

    s2 = (coordinates[0]-1, coordinates[1])
    if check_if_valid(s2):
        successors.append(s2)

    s3 = (coordinates[0], coordinates[1]+1)
    if check_if_valid(s3):
        successors.append(s3)

    s4 = (coordinates[0], coordinates[1]-1)
    if check_if_valid(s4):
        successors.append(s4)
    
    s5 = (coordinates[0]+1, coordinates[1]+1)
    if check_if_valid(s5):
        successors.append(s5)

    s6 = (coordinates[0]+1, coordinates[1]-1)
    if check_if_valid(s6):
        successors.append(s6)

    s7 = (coordinates[0]-1, coordinates[1]-1)
    if check_if_valid(s7):
        successors.append(s7)

    s8 = (coordinates[0]-1, coordinates[1]+1)
    if check_if_valid(s8):
        successors.append(s8)

    return successors

# def find_cost_of_successor(current, succesor):
#     x1, y1 = current.coordinates
#     x2, y2 = succesor
#     heuristic = abs(x1-x2) + abs(y1-y2)
#     weight = ((x1-x2)**2 + (y1-y2)**2)**0.5
#     cost = current.cost + weight + heuristic
#     return cost 

def manhattan_distance(point_1, point_2):
    x1, y1 = point_1
    x2, y2 = point_2
    distance = abs(x1-x2) + abs(y1-y2)
    return distance

def euclidean_distance(point_1, point_2):
    x1, y1 = point_1
    x2, y2 = point_2
    distance = ((x1-x2)**2 + (y1-y2)**2)**0.5
    return distance

def a_star(source, destination):
    #append the source to queue
    source_node = Node(source, None, 0, 0)
    closed = []
    queue = PriorityQueue()
    queue.put((0, source_node))
    #pop from queue and use as current
    while not queue.empty():
        # map(lambda node: node.coordinates, queue)
        current_node = queue.get()
        current_node = current_node[1]
        if current_node.coordinates in closed:
            continue
        closed.append(current_node.coordinates)
        #if current same as destination append current to path and return
        if current_node.coordinates == destination:
            return backtrack(current_node)
        #generate successors of current and add to queue
        succesors = find_eight_way_successors(current_node.coordinates)
        for succesor in succesors: 
            # calculate successor information
            step_cost = int(euclidean_distance(current_node.coordinates, succesor) * 10)
            heuristic = manhattan_distance(succesor, destination) * 10
            # heuristic = 0
            path_cost = current_node.path_cost + step_cost
            priority = path_cost + heuristic
            if succesor not in node_dict:
                succesor_node = Node(succesor, current_node, path_cost, priority)
                queue.put((priority, succesor_node))
            else:
                #compare the path cost and update path cost and parent
                succesor_node = node_dict[succesor]
                if succesor not in closed and path_cost < succesor_node.path_cost:
                    succesor_node.parent = current_node
                    succesor_node.path_cost = path_cost
                    succesor_node.priority = priority 

        #repeat till destination found or queue empty

def visualize_path(path):
    for coordinate in path:
        x, y = coordinate
        grid[x][y] = 5

if __name__ == "__main__":
    #read the grid file
    grid_file = open('../grids/grid_3.txt', 'r')
    lines = []
    for line in grid_file:
        lines.append([int(x) for x in line.split()])

    grid_dimensions = lines[0]
    grid = lines[1:]
        
    #apply the a_star algorithm to find path
    source = (0,28)
    # destination = (4,3)
    destination = (16,10)
    path = a_star(source, destination)
    print(path)
    visualize_path(path)
    for line in grid:
        print(line)