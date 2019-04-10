
import math

MIN_DISTANCE = 1.0

class Position:

	def __init__(self,angle,x,y):
		self.ang = angle
		self.xPos = x
		self.zPos = y


class Node():
	"""A node class for A* Pathfinding"""

	def __init__(self, parent=None, position=None):
		self.parent = parent
		self.position = position

		self.g = 0
		self.h = 0
		self.f = 0

	def __eq__(self, other):
		return self.position == other.position


def calcDistance(pos1,pos2):
	Dist = math.sqrt((pos1.xPos-pos2.xPos)**2 + (pos1.zPos-pos2.zPos)**2)

	if Dist < MIN_DISTANCE:
		print "Pos: ", pos1.xPos ," ",pos1.zPos
		print "Obsticle: " , pos2.xPos," ", pos2.zPos
		print "Dist: ", Dist
		print "----------"
	return Dist

def checkCollision(pos,xArr,yArr):
	for i in range(0,len(xArr)):
		if (calcDistance(pos,Position(0.0,xArr[i],yArr[i])) < MIN_DISTANCE):
			return True
				

def astar(mazeX,mazeY, start, end):
	"""Returns a list of tuples as a path from the given start to the given end in the given maze"""

	# Create start and end node
	start_node = Node(None, start)
	start_node.g = start_node.h = start_node.f = 0
	end_node = Node(None, end)
	end_node.g = end_node.h = end_node.f = 0

	# Initialize both open and closed list
	open_list = []
	closed_list = []

	# Add the start node
	open_list.append(start_node)

	# Loop until you find the end
	while len(open_list) > 0:

		
		# Get the current node
		current_node = open_list[0]
		current_index = 0
		for index, item in enumerate(open_list):
			if item.f < current_node.f:
				current_node = item
				current_index = index

		# Pop current off open list, add to closed list
		open_list.pop(current_index)
		closed_list.append(current_node)

		print "==============================="
		print current_node.position.xPos , "  ", current_node.position.zPos
		print "==============================="
		# Found the goal
		if current_node.position.xPos == end_node.position.xPos and current_node.position.zPos == end_node.position.zPos:
			path = []
			current = current_node
			while current is not None:
				path.append((current.position.xPos,current.position.zPos))
				current = current.parent
			return path[::-1] # Return reversed path

		# Generate children
		children = []
		for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares
			# Get node position
			node_position = Position(0,current_node.position.xPos + new_position[0]*MIN_DISTANCE, current_node.position.zPos + new_position[1]*MIN_DISTANCE)

			# Make sure within range
			if node_position.xPos > 10 or node_position.xPos < 0 or node_position.zPos > 10 or node_position.zPos < 0:
				continue

			# Make sure walkable terrain
			if checkCollision(node_position,mazeX,mazeY):
				continue
			
			# Create new node
			new_node = Node(current_node, node_position)

			# Append
			children.append(new_node)

		# Loop through children
		for child in children:

			# Child is on the closed list
			for closed_child in closed_list:
				if calcDistance(child.position,closed_child.position) < MIN_DISTANCE:
					continue

			# Create the f, g, and h values
			child.g = current_node.g + 1
			child.h = ((child.position.xPos - end_node.position.xPos) ** 2) + ((child.position.zPos - end_node.position.zPos) ** 2)
			child.f = child.g + child.h

			# Child is already in the open list
			for open_node in open_list:
				if calcDistance(child.position,open_node.position) < MIN_DISTANCE and child.g > open_node.g:
					continue

			# Add the child to the open list
			open_list.append(child)


def main():


	mazeX = [2.0,2.0,2.0,2.0,2.0]#[4,4,4,4,4,4,4,4,4,5,5,6,6]
	mazeY = [0.0,1.0,2.0,3.0,4.0]#[0,1,2,3,5,6,7,8,9,3,5,3,4]

	start = Position(0,0,0)
	end = Position(0,4,0)

	path = astar(mazeX,mazeY, start, end)

	print(path)


if __name__ == '__main__':
    main()
