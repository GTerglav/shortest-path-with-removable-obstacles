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
        if node.timeCreated <= time and node.status == 1:

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
    def copyParentOfNode(self, node, time):
        parentNode = node.parent
        while (
            parentNode is not None
            and len(parentNode.pointers) >= parentNode.pointerLimit
        ):
            copyParent = Node(
                parentNode.key, parentNode.val, time
            )  # create another node that is like parent of node

            # The copied node dies
            parentNode.status = 0

            if parentNode.parent is not None:
                copyParent.parent = parentNode.parent  # Has the same parent
            else:
                self.root = copyParent  # Now the copy is the root of tree

            if node.key < parentNode.key:  # add pointer from copyParent to node
                copyParent.pointers = [[time, 1, "left", node]]

                # copyParent alose has latest pointer to right
                # Reuse code from access
                rightPointers = []
                for pointer in parentNode.pointers:
                    if (
                        pointer[0] <= time and pointer[1] == 1
                    ):  # if pointer is alive and created before out time
                        if pointer[2] == "right" and pointer[3] is not None:
                            rightPointers.append(pointer)
                if len(rightPointers) == 0:
                    copyParent.pointers.append([time, 1, "right", None])
                else:
                    # among multiple right pointers we follow the one with greatest time created
                    rightPointers.sort(key=lambda x: x[0], reverse=True)
                    correctPointer = rightPointers[0]
                    copyParent.pointers.append(correctPointer)

            else:
                copyParent.pointers = [[time, 1, "right", node]]

                # again for left
                leftPointers = []
                for pointer in parentNode.pointers:
                    if (
                        pointer[0] <= time and pointer[1] == 1
                    ):  # if pointer is alive and created before out time
                        if pointer[2] == "left" and pointer[3] is not None:
                            leftPointers.append(pointer)
                if len(leftPointers) == 0:
                    copyParent.pointers.append([time, 1, "left", None])
                else:
                    # among multiple left pointers we follow the one with greatest time created
                    leftPointers.sort(key=lambda x: x[0], reverse=True)
                    correctPointer = leftPointers[0]
                    copyParent.pointers.append(correctPointer)

            # Update the parent of out node
            node.parent = copyParent

            # But if i copy the parent i should also update the pointers of grandparent not just parents pointers
            # TODO later PLSS

            # we send node copying up the tree
            if parentNode.parent is None:
                break
            else:
                parentNode = parentNode.parent

    # insert item at time
    def insert(self, key, val, time):
        # create new Node
        node = Node(key, val, time)

        # find parent for node
        parent = self.findParent(self.root, key, time, None)

        # node.parent = parent  # Set parent of Node
        if parent is None:  # If parent is none then it is root node
            node.parent = parent
            self.root = node

        # If the pointers are full we need to copy the node
        elif len(parent.pointers) == parent.pointerLimit:
            # Copied nodes are pronounced dead
            self.copyParentOfNode(node, time)
        elif (
            node.val < parent.val
        ):  # Check if it is right Node or Left Node by checking the value
            parent.pointers.append([time, 1, "left", node])
        else:
            parent.pointers.append([time, 1, "right", node])

        if node.parent == None:  # Root node is always Black
            node.color[time] = 0
            return

        if node.parent.parent == None:  # If parent of node is Root Node
            return

        self.fixInsert(node)

    # below code provided by chatgpt###################
    # Code for left rotate
    def leftRotate(self, node, time):
        # Node 'y' will be the new root of the rotated subtree
        rightPointers = [
            pointer
            for pointer in node.pointers
            if pointer[2] == "right" and pointer[1] == 1
        ]
        if not rightPointers:
            return  # Can't rotate if there is no right child

        rightPointers.sort(key=lambda x: x[0], reverse=True)
        y = rightPointers[0][3]  # The latest right child

        # Create a copy of y since we will modify it
        yNew = Node(y.key, y.val, time)
        yNew.color = y.color.copy()
        yNew.color[time] = y.color[time]

        # Update pointers for yNew
        yNew.pointers = [[time, 1, "left", node]]

        # If y has a left child, update its parent to node
        leftPointers = [
            pointer
            for pointer in y.pointers
            if pointer[2] == "left" and pointer[1] == 1
        ]
        if leftPointers:
            leftPointers.sort(key=lambda x: x[0], reverse=True)
            yNewLeftChild = leftPointers[0][3]
            if yNewLeftChild:
                yNewLeftChild.parent = node
            yNew.pointers.append([time, 1, "left", yNewLeftChild])

        # Update right child of node
        node.pointers.append(
            [time, 1, "right", yNewLeftChild if leftPointers else None]
        )

        # Handle the parent pointers
        yNew.parent = node.parent
        if node.parent is None:
            self.root = yNew
        else:
            if node.key < node.parent.key:
                parentPointers = [
                    pointer
                    for pointer in node.parent.pointers
                    if pointer[2] == "left" and pointer[1] == 1
                ]
            else:
                parentPointers = [
                    pointer
                    for pointer in node.parent.pointers
                    if pointer[2] == "right" and pointer[1] == 1
                ]

            if parentPointers:
                parentPointers.sort(key=lambda x: x[0], reverse=True)
                parentPointer = parentPointers[0]
                node.parent.pointers.remove(parentPointer)
                node.parent.pointers.append([time, 1, parentPointer[2], yNew])

        yNew.pointers.append([time, 1, "left", node])
        node.parent = yNew

    # Code for right rotate
    def rightRotate(self, node, time):
        # Node 'y' will be the new root of the rotated subtree
        leftPointers = [
            pointer
            for pointer in node.pointers
            if pointer[2] == "left" and pointer[1] == 1
        ]
        if not leftPointers:
            return  # Can't rotate if there is no left child

        leftPointers.sort(key=lambda x: x[0], reverse=True)
        y = leftPointers[0][3]  # The left child

        # Create a copy of y since we will modify it
        yNew = Node(y.key, y.val, time)
        yNew.color = y.color.copy()
        yNew.color[time] = y.color[time]

        # Update pointers for yNew
        yNew.pointers = [[time, 1, "right", node]]

        # If y has a right child, update its parent to node
        rightPointers = [
            pointer
            for pointer in y.pointers
            if pointer[2] == "right" and pointer[1] == 1
        ]
        if rightPointers:
            rightPointers.sort(key=lambda x: x[0], reverse=True)
            yNewRightChild = rightPointers[0][3]
            if yNewRightChild:
                yNewRightChild.parent = node
            yNew.pointers.append([time, 1, "right", yNewRightChild])

        # Update left child of node
        node.pointers.append(
            [time, 1, "left", yNewRightChild if rightPointers else None]
        )

        # Handle the parent pointers
        yNew.parent = node.parent
        if node.parent is None:
            self.root = yNew
        else:
            if node.key < node.parent.key:
                parentPointers = [
                    pointer
                    for pointer in node.parent.pointers
                    if pointer[2] == "left" and pointer[1] == 1
                ]
            else:
                parentPointers = [
                    pointer
                    for pointer in node.parent.pointers
                    if pointer[2] == "right" and pointer[1] == 1
                ]

            if parentPointers:
                parentPointers.sort(key=lambda x: x[0], reverse=True)
                parentPointer = parentPointers[0]
                node.parent.pointers.remove(parentPointer)
                node.parent.pointers.append([time, 1, parentPointer[2], yNew])

        yNew.pointers.append([time, 1, "right", node])
        node.parent = yNew

    # insert item at time
    def delete(self, node, item, time):
        return "TODO"

    def fixInsert(self, node):
        while node.parent and node.parent.color[max(node.parent.color)] == 1:
            grandparent = node.parent.parent
            if node.parent == grandparent.left():
                uncle = grandparent.right()
                if uncle and uncle.color[max(uncle.color)] == 1:
                    node.parent.color[max(node.parent.color)] = 0
                    uncle.color[max(uncle.color)] = 0
                    grandparent.color[max(grandparent.color)] = 1
                    node = grandparent
                else:
                    if node == node.parent.right():
                        node = node.parent
                        self.leftRotate(node, node.timeCreated)
                    node.parent.color[max(node.parent.color)] = 0
                    grandparent.color[max(grandparent.color)] = 1
                    self.rightRotate(grandparent, grandparent.timeCreated)
            else:
                uncle = grandparent.left()
                if uncle and uncle.color[max(uncle.color)] == 1:
                    node.parent.color[max(node.parent.color)] = 0
                    uncle.color[max(uncle.color)] = 0
                    grandparent.color[max(grandparent.color)] = 1
                    node = grandparent
                else:
                    if node == node.parent.left():
                        node = node.parent
                        self.rightRotate(node, node.timeCreated)
                    node.parent.color[max(node.parent.color)] = 0
                    grandparent.color[max(grandparent.color)] = 1
                    self.leftRotate(grandparent, grandparent.timeCreated)

        self.root.color[max(self.root.color)] = 0

    def printTree(self, node, indent="", last=True):
        if node is not None:
            print(
                indent,
                "`- " if last else "|- ",
                f"Key: {node.key}, Val: {node.val}, Color: {'Red' if max(node.color) == 1 else 'Black'}",
                sep="",
            )
            indent += "   " if last else "|  "
            children = [p[3] for p in node.pointers if p[1] == 1 and p[3] is not None]
            for i, child in enumerate(children):
                self.printTree(child, indent, i == len(children) - 1)
        else:
            print("None")

    def printTreeStructure(self):
        self.printTree(self.root)

    # ... (other methods) ...


# Test case
tree = persistentRBTree()
tree.insert(10, "A", 1)
tree.insert(20, "B", 2)
tree.insert(30, "C", 3)
tree.insert(15, "D", 4)
tree.insert(25, "E", 5)
tree.printTreeStructure()
