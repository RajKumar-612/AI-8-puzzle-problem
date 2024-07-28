import collections
import heapq
import sys
from queue import Queue
from datetime import datetime

# fringe = collections.deque([])
# fringe = []


class Node:
    def __init__(self, parent, state, step, cost, depth, heuristic_cost):
        self.parent = parent
        self.state = state
        self.step = step
        self.cost = cost
        self.depth = depth
        self.key = ''.join(str(item)
                           for innerlist in state for item in innerlist)
        self.heuristic_cost = heuristic_cost

    def __lt__(p, q):
        if p.heuristic_cost == q.heuristic_cost:
            return p.cost < q.cost
        return p.heuristic_cost < q.heuristic_cost

    # def __gt__(self, other):
    #     return self.cost > other.cost


# def insertNode(parent, state, step, cost, depth):
#     global nodes_generated, max_fringe_size
#     fringe.append(Node(parent, state, step, cost, depth))
#     nodes_generated = nodes_generated+1
#     max_fringe_size = max(max_fringe_size, len(fringe))

def calculateHeuristics(l1):
    sum = 0
    for i in range(3):
        for j in range(3):
            found = False
            for x in range(3):
                for y in range(3):
                    if l1[i][j] == goal[x][y]:
                        sum = sum + (abs(i-x)+abs(j-y))*l1[i][j]
                        # if (l1[i][j] == 0):
                        #     sum = sum+(abs(i-x)+abs(j-y))
                        found = True
                        break
                if found:
                    break
    return sum


def searchTrace(nodeexpanded, successorsCount, fringe, closet=set()):
    content = "\nExpanding Node:"
    content = content + str(nodeexpanded.state)+" "+"Action="+str(nodeexpanded.step)+" depth:"+str(
        nodeexpanded.depth) + " g(n):"+str(nodeexpanded.cost)+" F(n):"+str(nodeexpanded.heuristic_cost)
    content = content + "\nSuccessors Generated:"+str(successorsCount)
    content = content + "\nFringe: size:"+str(len(fringe))+" [\n"
    for x in fringe:
        content = content + "{ "+str(x.state)+" "+" depth:"+str(
            x.depth) + " g(n):"+str(x.cost)+" F(n):"+str(x.heuristic_cost)+" }"
    content = content + "\n]\n"
    content = content + "\nClosed: [\n"
    for x in closet:
        content = content + x+" "
    content = content + "\n]"
    traceFile.write(content)


def generateSuccessors(curNode, calc_heuristic=False):

    global nodes_generated
    found = False
    for i in range(3):
        if found:
            i = i-1
            break
        for j in range(3):
            if curNode.state[i][j] == 0:
                found = True
                break

    movesPossible = []
    if j-1 >= 0:
        s1 = [x[:] for x in curNode.state]
        s1[i][j], s1[i][j-1] = s1[i][j-1], s1[i][j]

        if calc_heuristic:
            heuristic = calculateHeuristics(s1)
        else:
            heuristic = 0

        movesPossible.append(Node(curNode, s1, "MOVE "+str(s1[i][j])+" RIGHT", curNode.cost +
                                  s1[i][j], curNode.depth+1, heuristic))
        nodes_generated = nodes_generated+1

    if j+1 < 3:
        s2 = [x[:] for x in curNode.state]
        s2[i][j], s2[i][j+1] = s2[i][j+1], s2[i][j]

        if calc_heuristic:
            heuristic = calculateHeuristics(s2)
        else:
            heuristic = 0

        movesPossible.append(Node(curNode, s2, "MOVE " +
                                  str(s2[i][j])+" LEFT", curNode.cost+s2[i][j], curNode.depth+1, heuristic))
        nodes_generated = nodes_generated+1

    if i-1 >= 0:
        s3 = [x[:] for x in curNode.state]
        s3[i][j], s3[i-1][j] = s3[i-1][j], s3[i][j]

        if calc_heuristic:
            heuristic = calculateHeuristics(s3)
        else:
            heuristic = 0

        movesPossible.append(Node(curNode, s3, "MOVE " +
                                  str(s3[i][j])+" DOWN", curNode.cost+s3[i][j], curNode.depth+1, heuristic))
        nodes_generated = nodes_generated+1

    if i+1 < 3:
        s4 = [x[:] for x in curNode.state]
        s4[i][j], s4[i+1][j] = s4[i+1][j], s4[i][j]

        if calc_heuristic:
            heuristic = calculateHeuristics(s4)
        else:
            heuristic = 0

        movesPossible.append(Node(curNode, s4, "MOVE " +
                                  str(s4[i][j])+" UP", curNode.cost+s4[i][j], curNode.depth+1, heuristic))
        nodes_generated = nodes_generated+1

    return movesPossible


def bfs():
    fringe = collections.deque([])
    closet = set()
    fringe.append(Node(None, start, "Start", 0, 0, 0))
    global nodes_popped, nodes_expanded, nodes_generated, max_fringe_size
    nodes_generated = nodes_generated+1
    while fringe:
        max_fringe_size = max(max_fringe_size, len(fringe))
        curNode = fringe.popleft()
        nodes_popped = nodes_popped+1
        if curNode.key not in closet:
            if curNode.state == goal:
                printGoalInformation(curNode)
                break
            successors = generateSuccessors(curNode)
            fringe.extend(successors)
            nodes_expanded = nodes_expanded+1
            closet.add(curNode.key)
            if dump:
                searchTrace(curNode, len(successors), fringe, closet)


def ucs():
    fringe = []
    closet = set()
    global nodes_popped, nodes_expanded, nodes_generated, max_fringe_size
    fringe.append(Node(None, start, "Start", 0, 0, 0))
    nodes_generated = nodes_generated+1
    while len(fringe) > 0:
        max_fringe_size = max(max_fringe_size, len(fringe))
        curNode = fringe[0]
        fringe = fringe[1:]
        nodes_popped = nodes_popped+1
        if curNode.key not in closet:
            if curNode.state == goal:
                printGoalInformation(curNode)
                break
            successors = generateSuccessors(curNode)
            fringe.extend(successors)
            heapq.heapify(fringe)
            # for x in fringe:
            #     print(x.cost, end=' ')
            # print()
            # time.sleep(2)
            nodes_expanded = nodes_expanded+1
            closet.add(curNode.key)
            if dump:
                searchTrace(curNode, len(successors), fringe, closet)


def dfs():
    fringe = collections.deque([])
    closet = set()
    fringe.append(Node(None, start, "Start", 0, 0, 0))
    global nodes_popped, nodes_expanded, nodes_generated, max_fringe_size
    nodes_generated = nodes_generated+1
    while fringe:
        max_fringe_size = max(max_fringe_size, len(fringe))
        curNode = fringe.pop()
        nodes_popped = nodes_popped+1
        if curNode.key not in closet:
            if curNode.state == goal:
                printGoalInformation(curNode)
                break
            closet.add(curNode.key)
            successors = generateSuccessors(curNode)
            fringe.extend(successors)
            if dump:
                searchTrace(curNode, len(successors), fringe, closet)
            nodes_expanded = nodes_expanded+1


def dls(depth):

    fringe = collections.deque([])
    fringe.append(Node(None, start, "Start", 0, 0, 0))
    global nodes_popped, nodes_expanded, nodes_generated, max_fringe_size
    nodes_generated = nodes_generated+1
    while fringe:
        max_fringe_size = max(max_fringe_size, len(fringe))
        curNode = fringe.pop()
        nodes_popped = nodes_popped+1
        if curNode.state == goal:
            printGoalInformation(curNode)
            return 1
        if curNode.depth < depth:
            successors = generateSuccessors(curNode)
            fringe.extend(successors)
            nodes_expanded = nodes_expanded+1
            if dump:
                searchTrace(curNode, len(successors), fringe)

    return 0


def ids():
    depth = 0
    while (not dls(depth)):
        depth = depth+1


def greedy():
    fringe = []
    closet = set()
    global nodes_popped, nodes_expanded, nodes_generated, max_fringe_size
    nodes_generated = nodes_generated+1
    fringe.append(Node(None, start, "Start", 0, 0, 0))
    while len(fringe) > 0:
        max_fringe_size = max(max_fringe_size, len(fringe))
        curNode = fringe[0]
        fringe = fringe[1:]
        nodes_popped = nodes_popped+1
        if curNode.key not in closet:
            if curNode.state == goal:
                printGoalInformation(curNode)
                break
            successors = generateSuccessors(curNode, True)
            fringe.extend(successors)
            heapq.heapify(fringe)
            # for x in fringe:
            #     print(x.heuristic_cost, end=' ')
            # print()
            # time.sleep(2)
            nodes_expanded = nodes_expanded+1
            closet.add(curNode.key)
            if dump:
                searchTrace(curNode, len(successors), fringe, closet)


def AStar():
    fringe = []
    closet = set()
    global nodes_popped, nodes_expanded, nodes_generated, max_fringe_size
    fringe.append(Node(None, start, "Start", 0, 0, 0))
    nodes_generated = nodes_generated+1
    while len(fringe) > 0:
        max_fringe_size = max(max_fringe_size, len(fringe))
        curNode = fringe[0]
        fringe = fringe[1:]
        nodes_popped = nodes_popped+1
        if curNode.key not in closet:
            if curNode.state == goal:
                print(len(closet))
                printGoalInformation(curNode)
                break
            successors = generateSuccessors(curNode, True)
            for x in successors:
                x.heuristic_cost = x.heuristic_cost+x.cost
                fringe.append(x)
            heapq.heapify(fringe)
            # for x in fringe:
            #     print("h:"+str(x.heuristic_cost)+"g:"+str(x.cost), end=' ')
            # print()
            # time.sleep(2)
            nodes_expanded = nodes_expanded+1
            closet.add(curNode.key)
            if dump:
                searchTrace(curNode, len(successors), fringe, closet)


def readFile(file):
    i = j = 0
    l = [[0]*3 for _ in range(3)]
    with open(file) as f:
        for line in f:
            line = line.split()
            if i == 3:
                break
            for _ in range(len(line)):
                l[i][j] = int(line[_])
                j = j+1
                if j == 3:
                    break
            i, j = i+1, 0
    return l


def printGoalInformation(node):
    print("Nodes Popped", nodes_popped)
    print("Nodes Generated", nodes_generated)
    print("Nodes Expanded", nodes_expanded)
    print("Max Fringe Size:", max_fringe_size)
    print("Solution found at depth "+str(node.depth)+" with cost "+str(node.cost))
    moves = []
    while not (node.state == start and node.cost == 0):
        moves.append(node.step)
        node = node.parent
    moves = moves[::-1]
    print("Steps to reach Goal:")
    print(*moves, sep="\n")


if __name__ == "__main__":
    n = len(sys.argv)
    startFile = sys.argv[1]
    goalFile = sys.argv[2]
    start = readFile(startFile)
    goal = readFile(goalFile)
    h = calculateHeuristics(start)
    method, dump = "a*", False

    if n >= 4:
        if (sys.argv[3].lower() == 'true' or sys.argv[3].lower() == 'false'):
            method = "a*"
            dump = sys.argv[3].lower()
        else:
            method = sys.argv[3].lower()
            if n > 4:
                dump = sys.argv[4].lower()

    dump = True if dump == 'true' else False
    nodes_popped = nodes_expanded = nodes_generated = max_fringe_size = 0
    filename = "trace-"+datetime.now().strftime("%d_%m_%Y-%I_%M_%S_%p")+".txt"
    if dump:
        traceFile = open(filename, "w")

    if method == "bfs":
        bfs()
    elif method == 'dfs':
        dfs()
    elif method == 'ucs':
        ucs()
    elif method == 'dls':
        dls(int(input("Enter depth:")))
    elif method == 'ids':
        ids()
    elif method == 'greedy':
        greedy()
    elif method == 'a*':
        AStar()
    if dump:
        traceFile.write("\nNodes Popped:"+str(nodes_popped))
        traceFile.write("\nNodes Expanded:"+str(nodes_expanded))
        traceFile.write("\nNodes Generated:"+str(nodes_generated))
        traceFile.write("\nMax Fringe Size:"+str(max_fringe_size))
        traceFile.close()
