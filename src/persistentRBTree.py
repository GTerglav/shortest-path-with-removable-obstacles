from math import inf

# Persistent RB Trees from article:
# Planar Point Location Using Persistent Search Trees
# by Sarnak & Tarjan


class Node:
    def __init__(self, key, val, time):
        self.key = key  # Key of Node
        self.val = val  # Value of Node
        self.timeCreated = time  # Time created
        self.timeDeleted = inf  # Time deleted
        self.parents = {time: None}  # Dict of parents and their times
        self.pointers = [
            [time, "left", None],
            [time, "right", None],
        ]  # list of pointers of form [timeCreated, "left or right", node]
        self.pointerLimit = 3  # Each node can have at most this many pointers
        self.colors = {
            time: "red"
        }  # Red Node as new node is always inserted as Red Node


class persistentRBTree:
    def __init__(self):
        self.roots = {0: None}  # root node can be copied so we have a dict of roots

    # Returns oldes root of current tree that was created before time
    def getCurrentRoot(self, time):
        eligibleColors = [[t, root] for t, root in self.roots.items() if t <= time]
        eligibleColors.sort(key=lambda x: x[0], reverse=True)
        return eligibleColors[0][1]

    # Returns color of node at time or 0 if its none
    def getCurrentColor(self, node, time):
        if node is None:
            return 0
        else:
            eligibleColors = [
                [t, color] for t, color in node.colors.items() if t <= time
            ]
            eligibleColors.sort(key=lambda x: x[0], reverse=True)
            return eligibleColors[0][1]

    # Returns current parent of node at time
    def getCurrentParent(self, node, time):
        eligibleParents = [
            [t, parent] for t, parent in node.parents.items() if t <= time
        ]
        eligibleParents.sort(key=lambda x: x[0], reverse=True)
        return eligibleParents[0][1]

    # Returns latest child node in chosen direction at time
    def getLatestChild(self, node, time, direction):  # direction is "right" or "left"
        # We separate "None" children and "node" children
        children = [
            pointer[2]
            for pointer in node.pointers
            if (pointer[2] is not None)
            and (pointer[1] == direction)
            and pointer[0] <= time
        ]  # all children in chosen direction that are not None
        noneChildren = [
            [pointer[0], None]
            for pointer in node.pointers
            if (pointer[2] is None) and (pointer[1] == direction) and pointer[0] <= time
        ]  # all children in chosen direction that ARE None

        noneTime = 0
        if len(noneChildren) > 0:
            noneChildren.sort(key=lambda x: x[0], reverse=True)
            noneTime = noneChildren[0][0]  # latest pointer to none

        if len(children) == 0:
            return None  # if no node children
        else:
            # Among node children choose the latest one that is still alive
            currentChildren = [
                child
                for child in children
                if child.timeCreated <= time < child.timeDeleted
            ]
            currentChildren.sort(key=lambda x: x.timeCreated, reverse=True)
            if len(currentChildren) == 0:
                return None  # all children are dead
            else:
                if currentChildren[0].timeCreated < noneTime:
                    return None  # none child is latest
                else:
                    return currentChildren[0]  # real child is latest

    # find and return item in tree with greatest key value <= to key at time.
    # Need to save the node at which we last went right
    def access(self, node, key, time, lastRight):
        if node is None:
            return lastRight

        # First check correct time should always be true though
        if node.timeCreated <= time < node.timeDeleted:

            # found
            if node.key == key:
                return node

            # go left
            if node.key > key:
                return self.access(
                    self.getLatestChild(node, time, "left"), key, time, lastRight
                )

            # go right
            if node.key < key:
                return self.access(
                    self.getLatestChild(node, time, "right"), key, time, node
                )  # Update lastRight to this node

        else:
            print("error: wrong time")
            return lastRight

    # find and return item in tree with smalles key value >= to key at time.
    # Like access only here we save last left node
    def leftAccess(self, node, key, time, lastLeft):
        if node is None:
            return lastLeft

        # First check correct time should always be true though
        if node.timeCreated <= time < node.timeDeleted:

            # found
            if node.key == key:
                return node

            # go left
            if node.key > key:
                return self.leftAccess(
                    self.getLatestChild(node, time, "left"), key, time, node
                )  # Update lastLeft to this node

            # go right
            if node.key < key:
                return self.leftAccess(
                    self.getLatestChild(node, time, "right"), key, time, lastLeft
                )

        else:
            print("error: wrong time")
            return lastLeft

    # Returns all items with lB <= key uB at time
    def accessRange(self, lowerBound, upperBound, time):
        root = self.getCurrentRoot(time)

        # Find the node with smallest key that is greater or equal to lower bound
        lowestNode = self.leftAccess(root, lowerBound, time, None)

        # We will now search the tree from lowest node.
        # First we go to right parent (if it exists) and right child, since they are both bigger
        # We check that they are smaller then lowerBound and add them
        # This step is done in inorderHelperTR (top right)

        # Then From right child we can go left or right (inorderHelperLR)
        # If we go left we know that all children will be in our range so we add the whole subtree (getAllChildren)
        # If we go right we again do (inorderHelperLR)

        # We repeat until we meet a node with key higher than upper bound

        def getAllChildren(node):
            if node is None:
                return []

            leftChild = self.getLatestChild(node, time, "left")
            rightChild = self.getLatestChild(node, time, "right")

            return (
                getAllChildren(leftChild)
                + [(node, node.key)]
                + getAllChildren(rightChild)
            )

        def inorderHelperLR(node):
            if node is None or node.key > upperBound:
                return []
            leftChild = self.getLatestChild(node, time, "left")
            rightChild = self.getLatestChild(node, time, "right")
            return (
                getAllChildren(leftChild)
                + [(node, node.key)]
                + inorderHelperLR(rightChild)
            )

        def inorderHelperTR(node):
            if node is None or node.key > upperBound:
                return []

            parent = self.getCurrentParent(node, time)
            rightChild = self.getLatestChild(node, time, "right")

            if parent and parent.key > node.key and parent.key <= upperBound:
                return (
                    inorderHelperTR(parent)
                    + [(node, node.key)]
                    + inorderHelperLR(rightChild)
                )
            else:
                return [(node, node.key)] + inorderHelperLR(rightChild)

        return inorderHelperTR(lowestNode)

    # Similar to access, instead returns the last real node instead of last right.
    # Finds parent for insertion
    def findParent(self, node, key, time, lastNode):
        # If none we return the last node
        if node is None:
            return lastNode

        # First check correct time should always be true though
        if node.timeCreated <= time < node.timeDeleted:

            # go left
            if node.key > key:
                return self.findParent(
                    self.getLatestChild(node, time, "left"), key, time, node
                )

            # go right
            if node.key < key:
                return self.findParent(
                    self.getLatestChild(node, time, "right"), key, time, node
                )

        else:
            print("error: wrong time")
            return lastNode

    # Adds pointer to node at time. Returns list of nodes that got copied during execution
    def addPointer(self, pointer, node, time):
        copiedNodes = []

        # node has free pointers
        if len(node.pointers) < node.pointerLimit:
            # First attemt to overwrite pointer if has current time
            for pointerNode in node.pointers:
                if pointerNode[0] == time and pointerNode[1] == pointer[1]:
                    node.pointers.remove(pointerNode)
                    node.pointers.append(pointer)
                    return copiedNodes
            node.pointers.append(pointer)
            return copiedNodes
        # Node doesnt have free pointers
        else:
            # Attempt to overwrite a current pointer
            for pointerNode in node.pointers:
                if pointerNode[0] == time and pointerNode[1] == pointer[1]:
                    node.pointers.remove(pointerNode)
                    node.pointers.append(pointer)
                    return copiedNodes

            # Cannot overwrite: we have to copy the node
            while (len(node.pointers) >= node.pointerLimit) and (node is not None):
                copyNode = Node(node.key, node.val, time)

                # Set parent of node in pointer to copyNode
                if pointer[2] is not None:
                    pointer[2].parents[time] = copyNode

                # Same parent as original
                parentNode = self.getCurrentParent(node, time)
                copyNode.parents[time] = parentNode

                # Same color as original
                copyNode.colors[time] = self.getCurrentColor(node, time)

                # Give correct pointers to copy
                # The nodes it points to should have copy as parent
                if pointer[1] == "left":
                    rightChild = self.getLatestChild(node, time, "right")
                    copyNode.pointers = [pointer]
                    copyNode.pointers.append([time, "right", rightChild])

                    if rightChild is not None:
                        rightChild.parents[time] = copyNode

                else:
                    copyNode.pointers = [pointer]
                    leftChild = self.getLatestChild(node, time, "left")
                    copyNode.pointers.append([time, "left", leftChild])
                    if leftChild is not None:
                        leftChild.parents[time] = copyNode

                # Kill our original node
                node.timeDeleted = time

                # Add our copyNode to copied nodes
                copiedNodes.append(copyNode)

                # Give parent of copy a pointer to copy
                if parentNode is not None:

                    direction = ""
                    if parentNode.key < copyNode.key:
                        direction = "right"
                    else:
                        direction = "left"

                    # Parent has full pointer
                    if len(parentNode.pointers) == parentNode.pointerLimit:
                        # Try to overwrite existing pointer
                        for pointerNode in parentNode.pointers:
                            if pointerNode[0] == time and pointerNode[1] == direction:
                                parentNode.pointers.remove(pointerNode)
                                parentNode.pointers.append([time, direction, copyNode])
                                return copiedNodes
                        # If no overwrite: repeat while loop but now one lvl higher (We copy the parent)
                        pointer = [time, direction, copyNode]
                        node = parentNode
                    # pointer not full
                    else:
                        # First try to overwrite
                        for pointerNode in parentNode.pointers:
                            if pointerNode[0] == time and pointerNode[1] == direction:
                                parentNode.pointers.remove(pointerNode)
                                parentNode.pointers.append([time, direction, copyNode])
                                return copiedNodes
                        # No overwrite so we just append
                        parentNode.pointers.append([time, direction, copyNode])
                        return copiedNodes
                # Our copy is the new root
                else:
                    self.roots[time] = copyNode
                    return copiedNodes

    # insert item at time
    def insert(self, key, val, time):

        # create new Node
        node = Node(key, val, time)

        # find parent for node
        root = self.getCurrentRoot(time)
        parent = self.findParent(root, key, time, None)
        node.parents[time] = parent

        # when adding pointers, incase parent gets copied, I want to work with new copy
        potentialCopies = []

        if parent is None:  # If node is root
            self.roots[time] = node
            node.colors[time] = "black"
            return

        else:  # Add the pointer from parent to our new node
            if node.key < parent.key:
                potentialCopies = self.addPointer([time, "left", node], parent, time)
            else:
                potentialCopies = self.addPointer([time, "right", node], parent, time)

        if len(potentialCopies) >= 1:
            parent = potentialCopies[0]  # if parent was copied update it to new copy

        if self.getCurrentParent(parent, time) is None:  # If parent is Root Node
            return

        self.fixInsert(node, time)  # Else call for Fix Up

    # Returns node with minimum key
    def minimum(self, node, time):
        leftChild = self.getLatestChild(node, time, "left")
        while leftChild is not None:
            node = leftChild
        return node

    # Left rotation, also returns nodes that got copied
    def leftRotate(self, node, time):

        # Right child
        y = self.getLatestChild(node, time, "right")
        if y is None:
            return []

        yLeftChild = self.getLatestChild(y, time, "left")

        # First Assignt new parents to all nodes
        nodeParent = self.getCurrentParent(node, time)
        y.parents[time] = nodeParent
        if nodeParent is None:
            self.roots[time] = y
        if yLeftChild is not None:
            yLeftChild.parents[time] = node
        node.parents[time] = y

        # Now add pointers from bottom up
        # We can remove current pointer from node to y if it exists, to free up space
        for pointer in node.pointers:
            if pointer[0] == time and pointer[2] == y:
                node.pointers.remove(pointer)

        # Add right pointer from node to yLeftChild
        copies = self.addPointer([time, "right", yLeftChild], node, time)

        # this means node got copied, we already have pointer from y to copy of node
        if len(copies) == 1:

            # update our nodes to new versions
            node = copies[0]
            y = self.getCurrentParent(node, time)
            parent = self.getCurrentParent(y, time)

            # Only have to add pointer from parent to y
            if parent is not None:
                if parent.key > node.key:
                    copies2 = self.addPointer([time, "left", y], parent, time)
                    return copies + copies2
                else:
                    copies2 = self.addPointer([time, "right", y], parent, time)
                    return copies + copies2

        # means node and y got copied and we have all the pointers we need
        elif len(copies) > 1:
            return copies

        # No copying done
        else:
            # Add Left pointer from y to node a
            copies3 = self.addPointer([time, "left", node], y, time)

            # If y got copied we are done
            if len(copies3) >= 1:
                return copies + copies3
            else:
                # add pointer from parent to y
                parent = self.getCurrentParent(y, time)
                if parent is not None:
                    if parent.key > node.key:
                        copies4 = self.addPointer([time, "left", y], parent, time)
                        return copies + copies3 + copies4
                    else:
                        copies4 = self.addPointer([time, "right", y], parent, time)
                        return copies + copies3 + copies4

    # Same as left rotate
    def rightRotate(self, node, time):

        # Left child
        y = self.getLatestChild(node, time, "left")
        if y is None:
            return

        yRightChild = self.getLatestChild(y, time, "right")

        # First Assignt new parents to all nodes
        nodeParent = self.getCurrentParent(node, time)
        y.parents[time] = nodeParent
        if nodeParent is None:
            self.roots[time] = y
        if yRightChild is not None:
            yRightChild.parents[time] = node
        node.parents[time] = y

        # Now add pointers from bottom up
        # We can remove current pointer from node to y if it exists, to free up space
        for pointer in node.pointers:
            if pointer[0] == time and pointer[2] == y:
                node.pointers.remove(pointer)
        # Add left pointer from node to yRightChild
        copies = self.addPointer([time, "left", yRightChild], node, time)
        if (
            len(copies) == 1
        ):  # that means node got copied and we already have pointer from new node to y
            node = copies[0]
            y = self.getCurrentParent(node, time)
            parent = self.getCurrentParent(y, time)
            if parent is not None:
                if parent.key > node.key:
                    copies2 = self.addPointer([time, "left", y], parent, time)
                    return copies + copies2
                else:
                    copies2 = self.addPointer([time, "right", y], parent, time)
                    return copies + copies2

        elif (
            len(copies) > 1
        ):  # means node and y got copied and we have all the connections we need
            return copies
        else:  # No copying done
            # Add Right pointer from y to node a
            copies3 = self.addPointer([time, "right", node], y, time)
            if len(copies3) >= 1:  # If y got copied we are done
                return copies + copies3
            else:
                parent = self.getCurrentParent(y, time)
                if parent is not None:
                    if parent.key > node.key:
                        copies4 = self.addPointer([time, "left", y], parent, time)
                        return copies + copies3 + copies4
                    else:
                        copies4 = self.addPointer([time, "right", y], parent, time)
                        return copies + copies3 + copies4

    # Almost the same as for ephemereal (normal) RB Trees
    def fixInsert(self, node, time):
        while self.getCurrentColor(self.getCurrentParent(node, time), time) == "red":
            parent = self.getCurrentParent(node, time)
            grandparent = self.getCurrentParent(parent, time)

            if parent == self.getLatestChild(grandparent, time, "left"):
                uncle = self.getLatestChild(grandparent, time, "right")
                if uncle and self.getCurrentColor(uncle, time) == "red":
                    parent.colors[time] = "black"
                    uncle.colors[time] = "black"
                    grandparent.colors[time] = "red"
                    node = grandparent
                else:
                    if node == self.getLatestChild(parent, time, "right"):
                        node = parent
                        copies = self.leftRotate(node, time)
                        if len(copies) > 0:
                            grandparent = copies[0]
                        parent = self.getCurrentParent(node, time)
                    parent.colors[time] = "black"
                    grandparent.colors[time] = "red"
                    self.rightRotate(grandparent, time)
            else:
                uncle = self.getLatestChild(grandparent, time, "left")
                if uncle and self.getCurrentColor(uncle, time) == "red":
                    parent.colors[time] = "black"
                    uncle.colors[time] = "black"
                    grandparent.colors[time] = "red"
                    node = grandparent
                else:
                    if node == self.getLatestChild(parent, time, "left"):
                        node = parent
                        copies = self.rightRotate(node, time)
                        if len(copies) > 0:
                            grandparent = copies[0]
                        parent = self.getCurrentParent(node, time)
                    parent.colors[time] = "black"
                    grandparent.colors[time] = "red"
                    self.leftRotate(grandparent, time)
        root = self.getCurrentRoot(time)
        root.colors[time] = "black"

    ## CHATGPT DELETE  implementation ###

    def transplant(self, u, v, time):
        parent = self.getCurrentParent(u, time)
        if parent is None:
            self.roots[time] = v
        else:
            direction = (
                "left" if u == self.getLatestChild(parent, time, "left") else "right"
            )
            self.addPointer([time, direction, v], parent, time)
        if v is not None:
            v.parents[time] = parent

    # Deletes node with largest key <= given key at time
    def delete(self, key, time):
        root = self.getCurrentRoot(time)
        nodeToDelete = self.access(root, key, time, None)
        if nodeToDelete is None:
            return  # Node not found

        original_color = self.getCurrentColor(nodeToDelete, time)

        if self.getLatestChild(nodeToDelete, time, "left") is None:
            child = self.getLatestChild(nodeToDelete, time, "right")
            self.transplant(nodeToDelete, child, time)
        elif self.getLatestChild(nodeToDelete, time, "right") is None:
            child = self.getLatestChild(nodeToDelete, time, "left")
            self.transplant(nodeToDelete, child, time)
        else:
            successor = self.minimum(
                self.getLatestChild(nodeToDelete, time, "right"), time
            )
            original_color = self.getCurrentColor(successor, time)
            child = self.getLatestChild(successor, time, "right")
            if self.getCurrentParent(successor, time) == nodeToDelete:
                if child is not None:
                    child.parents[time] = successor
            else:
                self.transplant(successor, child, time)
                successor.pointers.append(
                    [time, "right", self.getLatestChild(nodeToDelete, time, "right")]
                )
                if self.getLatestChild(nodeToDelete, time, "right") is not None:
                    self.getLatestChild(nodeToDelete, time, "right").parents[
                        time
                    ] = successor
            self.transplant(nodeToDelete, successor, time)
            successor.pointers.append(
                [time, "left", self.getLatestChild(nodeToDelete, time, "left")]
            )
            if self.getLatestChild(nodeToDelete, time, "left") is not None:
                self.getLatestChild(nodeToDelete, time, "left").parents[
                    time
                ] = successor
            successor.colors[time] = self.getCurrentColor(nodeToDelete, time)

        nodeToDelete.timeDeleted = time

        if original_color == "black":
            self.fixDelete(
                child,
                (
                    self.getCurrentParent(child, time)
                    if child
                    else self.getCurrentParent(nodeToDelete, time)
                ),
                time,
            )

    def fixDelete(self, node, parent, time):
        while (
            node != self.getCurrentRoot(time)
            and self.getCurrentColor(node, time) == "black"
        ):
            if node == self.getLatestChild(parent, time, "left"):
                sibling = self.getLatestChild(parent, time, "right")
                if self.getCurrentColor(sibling, time) == "red":
                    sibling.colors[time] = "black"
                    parent.colors[time] = "red"
                    self.leftRotate(parent, time)
                    sibling = self.getLatestChild(parent, time, "right")
                if (
                    self.getCurrentColor(
                        self.getLatestChild(sibling, time, "left"), time
                    )
                    == "black"
                    and self.getCurrentColor(
                        self.getLatestChild(sibling, time, "right"), time
                    )
                    == "black"
                ):
                    sibling.colors[time] = "red"
                    node = parent
                    parent = self.getCurrentParent(node, time)
                else:
                    if (
                        self.getCurrentColor(
                            self.getLatestChild(sibling, time, "right"), time
                        )
                        == "black"
                    ):
                        self.getLatestChild(sibling, time, "left").colors[
                            time
                        ] = "black"
                        sibling.colors[time] = "red"
                        self.rightRotate(sibling, time)
                        sibling = self.getLatestChild(parent, time, "right")
                    sibling.colors[time] = self.getCurrentColor(parent, time)
                    parent.colors[time] = "black"
                    self.getLatestChild(sibling, time, "right").colors[time] = "black"
                    self.leftRotate(parent, time)
                    node = self.getCurrentRoot(time)
            else:
                sibling = self.getLatestChild(parent, time, "left")
                if self.getCurrentColor(sibling, time) == "red":
                    sibling.colors[time] = "black"
                    parent.colors[time] = "red"
                    self.rightRotate(parent, time)
                    sibling = self.getLatestChild(parent, time, "left")
                if (
                    self.getCurrentColor(
                        self.getLatestChild(sibling, time, "left"), time
                    )
                    == "black"
                    and self.getCurrentColor(
                        self.getLatestChild(sibling, time, "right"), time
                    )
                    == "black"
                ):
                    sibling.colors[time] = "red"
                    node = parent
                    parent = self.getCurrentParent(node, time)
                else:
                    if (
                        self.getCurrentColor(
                            self.getLatestChild(sibling, time, "left"), time
                        )
                        == "black"
                    ):
                        self.getLatestChild(sibling, time, "right").colors[
                            time
                        ] = "black"
                        sibling.colors[time] = "red"
                        self.leftRotate(sibling, time)
                        sibling = self.getLatestChild(parent, time, "left")
                    sibling.colors[time] = self.getCurrentColor(parent, time)
                    parent.colors[time] = "black"
                    self.getLatestChild(sibling, time, "left").colors[time] = "black"
                    self.rightRotate(parent, time)
                    node = self.getCurrentRoot(time)
        if node:
            node.colors[time] = "black"

    def inorderTraversal(self, time):
        def inorderHelper(node):
            if node is None:
                return []

            leftChild = self.getLatestChild(node, time, "left")
            rightChild = self.getLatestChild(node, time, "right")

            return (
                inorderHelper(leftChild)
                + [(node, node.key, node.val)]
                + inorderHelper(rightChild)
            )

        root = self.getCurrentRoot(time)
        return inorderHelper(root)

    def printTree(self, time):
        root = self.getCurrentRoot(time)
        self.printSubTree(root, time, "", True)

    def printSubTree(self, node, time, indent, last):
        if node is not None:
            timeC = node.timeCreated
            timeD = node.timeDeleted
            print(
                indent,
                "R--- " if last else "L--- ",
                node.key,
                "(",
                self.getCurrentColor(node, time),
                ")",
                "(created: ",
                timeC,
                ")",
                "(deleted: ",
                timeD,
                ")",
                sep="",
            )
            indent += "   " if last else "|  "
            leftChild = self.getLatestChild(node, time, "left")
            rightChild = self.getLatestChild(node, time, "right")
            if leftChild or rightChild:
                if leftChild:
                    self.printSubTree(leftChild, time, indent, False)
                if rightChild:
                    self.printSubTree(rightChild, time, indent, True)


tree = persistentRBTree()

# Tests

# tree.insert(10, "A", 1)
# tree.insert(20, "B", 1)
# tree.insert(15, "C", 1)

# tree.delete(10, 2)
# tree.insert(30, "E", 2)
# tree.insert(40, "F", 2)

# tree.delete(20, 3)
# tree.insert(35, "G", 3)
# tree.insert(45, "H", 3)

# tree.delete(15, 4)
# tree.delete(30, 4)
# tree.insert(7, "I", 4)


# print("Tree at time 1:")
# tree.printTree(1)

# print("Tree at time 2:")
# tree.printTree(2)

# print("Tree at time 3:")
# tree.printTree(3)

# print("Tree at time 4:")
# tree.printTree(4)

# print("inorder Traversal:")
# print(tree.inorderTraversal(4))
# print("range:")
# print(tree.accessRange(30, 40, 4))
