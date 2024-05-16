from math import inf


class Node:
    def __init__(self, key, val, time):
        self.key = key  # Key of Node
        self.val = val  # Value of Node
        self.timeCreated = time  # Time created
        self.status = 1  # Alive or dead: 1 or 0
        self.parent = None  # Parent of Node
        self.pointers = [
            [time, 1, "left", None],
            [time, 1, "right", None],
        ]  # list of pointers of form [timeCreated, status, "left or right", node]
        self.pointerLimit = 3  # Each node can have at most this many pointers
        self.color = {time: 1}  # Red Node as new node is always inserted as Red Node


class persistentRBTree:
    def __init__(self):
        # root node can be copied idk how to handle that yet
        self.root = None

    # find and return item in tree with greatest key value <= to key at time. Need to save the node at which we last went right
    def access(self, node, key, time, lastRight):
        # If none we return the node where we went right for the last time
        if node is None:
            return lastRight

        # First check correct time should always be true though
        if node.timeCreated <= time < node.timeDeleted:

            # found
            if node.key == key:
                return node

            # go left
            if node.key < key:
                leftPointers = []  # for dealing with multiple left pointers
                for pointer in node.pointers:
                    if (
                        pointer[0] <= time and pointer[1] == 1
                    ):  # if pointer is alive and created before out time
                        if pointer[2] == "left" and pointer[3] is not None:
                            leftPointers.append(pointer)
                if len(leftPointers) == 0:
                    return self.access(self, None, key, time, lastRight)
                else:
                    # among multiple left pointers we follow the one with greatest time created
                    leftPointers.sort(key=lambda x: x[0], reverse=True)
                    correctPointer = leftPointers[0]
                    return self.access(self, correctPointer[3], key, time, lastRight)

            # go right
            if node.key > key:
                rightPointers = []  # for dealing with multiple right pointers
                for pointer in node.pointers:
                    if (
                        pointer[0] <= time and pointer[1] == 1
                    ):  # if pointer is alive and created before out time
                        if pointer[2] == "right" and pointer[3] is not None:
                            rightPointers.append(pointer)
                if len(rightPointers) == 0:
                    return self.access(self, None, key, time, lastRight)
                else:
                    # among multiple right pointers we follow the one with greatest time created
                    rightPointers.sort(key=lambda x: x[0], reverse=True)
                    correctPointer = rightPointers[0]
                    return self.access(self, correctPointer[3], key, time, lastRight)

        else:
            print("error: wrong time")
            return lastRight

    # Similar to access, instead returns the bottom node instead of last right. Finds parent for insertion
    def findParent(self, node, key, time, lastNode):
        # If none we return the last node
        if node is None:
            return lastNode

        # First check correct time should always be true though
        if node.timeCreated <= time < node.timeDeleted:

            # go left
            if node.key < key:
                leftPointers = []  # for dealing with multiple left pointers
                for pointer in node.pointers:
                    if (
                        pointer[0] <= time and pointer[1] == 1
                    ):  # if pointer is alive and created before out time
                        if pointer[2] == "left" and pointer[3] is not None:
                            leftPointers.append(pointer)
                if len(leftPointers) == 0:
                    return node  # If all pointers are none just return the node
                else:
                    # among multiple left pointers we follow the one with greatest time created
                    leftPointers.sort(key=lambda x: x[0], reverse=True)
                    correctPointer = leftPointers[0]
                    return self.access(self, correctPointer[3], key, time, lastNode)

            # go right
            if node.key > key:
                rightPointers = []  # for dealing with multiple right pointers
                for pointer in node.pointers:
                    if (
                        pointer[0] <= time and pointer[1] == 1
                    ):  # if pointer is alive and created before out time
                        if pointer[2] == "right" and pointer[3] is not None:
                            rightPointers.append(pointer)
                if len(rightPointers) == 0:
                    return node  # If all pointers are none just return the node
                else:
                    # among multiple right pointers we follow the one with greatest time created
                    rightPointers.sort(key=lambda x: x[0], reverse=True)
                    correctPointer = rightPointers[0]
                    return self.access(self, correctPointer[3], key, time, lastNode)

        else:
            print("error: wrong time")
            return lastNode

    # Creates a copy of a node and potentialy its parents
    def copyNode(self, node, time):
        copyNode = Node(node.key, node.val, time)
        copyNode.parent = node.parent

    # insert item at time
    def insert(self, root, key, val, time):
        # create new Node
        node = Node(key, val, time)

        # find parent for node
        parent = root.findParent(root, key, time, None)

        # node.parent = parent  # Set parent of Node
        if parent is None:  # If parent is none then it is root node
            node.parent = parent
            root = node

        # If the pointers are full we need to copy the node
        elif len(parent.pointers) == parent.pointerLimit:
            # Copied nodes are pronounced dead
            parent.status = 0
            copyNode = Node(parent.key, parent.val, time)
            copyNode.self

        elif (
            node.val < parent.val
        ):  # Check if it is right Node or Left Node by checking the value
            parent.left = node
        else:
            parent.right = node

        return "TODO"

    # insert item at time
    def delete(self, node, item, time):
        return "TODO"
