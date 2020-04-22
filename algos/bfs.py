grid = None
# w x h
grid_dimensions = None

class Node:
    def __init__(self, coordinates, parent):
        self.coordinates = coordinates
        self.parent = parent


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

def bfs(source, destination):
    #append the source to queue
    source_node = Node(source, None)
    queue = []
    visited = []
    queue.append(source_node)
    #pop from queue and use as current
    while len(queue) > 0:
        map_output = map(lambda node: node.coordinates, queue)
        # print(list(map_output))
        current_node = queue.pop(0)
        #if current same as destination append current to path and return
        if current_node.coordinates == destination:
            return backtrack(current_node)
        #generate successors of current and add to queue
        # succesors = find_four_way_successors(current_node.coordinates)
        succesors = find_eight_way_successors(current_node.coordinates)
        for succesor in succesors:
            if succesor not in visited:
                visited.append(succesor)
                succesor_node = Node(succesor, current_node)
                queue.append(succesor_node)
        #repeat till destination found or queue empty

def visualize_path(path):
    for coordinate in path:
        x, y = coordinate
        grid[x][y] = 5

if __name__ == "__main__":
    #read the grid file
    grid_file = open('../grids/grid_1.txt', 'r')
    lines = []
    for line in grid_file:
        lines.append([int(x) for x in line.split()])


    grid_dimensions = lines[0]
    grid = lines[1:]
        
    #apply the bfs algorithm to find path
    source = (0,0)
    destination = (4,3)
    path = bfs(source, destination)
    print(path)
    visualize_path(path)
    for line in grid:
        print(line)