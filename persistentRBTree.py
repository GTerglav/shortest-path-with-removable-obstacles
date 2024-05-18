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
        # root node can be copied so we have a list of roots
        self.roots = {0: None}  # Dict of roots

    # Returns oldes root of current tree that was created before time
    def getCurrentRoot(self, time):
        eligibleColors = [[t, root] for t, root in self.roots.items() if t <= time]
        eligibleColors.sort(key=lambda x: x[0], reverse=True)
        return eligibleColors[0][1]

    def getCurrentColor(self, node, time):
        if node is None:
            return 0
        else:
            eligibleColors = [
                [t, color] for t, color in node.colors.items() if t <= time
            ]
            eligibleColors.sort(key=lambda x: x[0], reverse=True)
            return eligibleColors[0][1]

    # Returns node of current parent
    def getCurrentParent(self, node, time):
        eligibleParents = [
            [t, parent] for t, parent in node.parents.items() if t <= time
        ]
        eligibleParents.sort(key=lambda x: x[0], reverse=True)
        return eligibleParents[0][1]

    # Returns latest child node in chosen direction with latest time
    def getLatestChild(self, node, time, direction):  # direction is "right" or "left"
        children = [
            pointer[2]
            for pointer in node.pointers
            if (pointer[2] is not None) and (pointer[1] == direction)
        ]  # all children in chosen direction
        if len(children) == 0:
            return None
        else:
            # Among children choose the latest one
            currentChildren = [
                child
                for child in children
                if child.timeCreated <= time < child.timeDeleted
            ]
            currentChildren.sort(key=lambda x: x.timeCreated, reverse=True)
            if len(currentChildren) == 0:
                return None
            else:
                return currentChildren[0]

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
            if node.key > key:
                return self.access(
                    self, self.getLatestChild(node, time, "left"), key, time, lastRight
                )

            # go right
            if node.key < key:
                return self.access(
                    self, self.getLatestChild(node, time, "right"), key, time, node
                )  # Update lastRight to this node

        else:
            print("error: wrong time")
            return lastRight

    # Similar to access, instead returns the last real node instead of last right. Finds parent for insertion
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

    def addPointer(self, pointer, node, time):
        # I want to return the list of copied nodes so that i can use them outside of this function
        copiedNodes = []
        if len(node.pointers) < node.pointerLimit:
            for pointerNode in node.pointers:
                if (
                    pointerNode[0] == time
                    and pointerNode[1] == pointer[1]
                    and pointerNode[2] is None
                ):  # Replace pointer if its same just points to None
                    node.pointers.remove(pointerNode)
                    node.pointers.append(pointer)
                    return copiedNodes
            node.pointers.append(pointer)
            return copiedNodes
        else:
            # can overwrite current pointers
            for pointerNode in node.pointers:
                if (
                    pointerNode[0] == time
                    and pointerNode[1] == pointer[1]
                    and pointerNode[2] is None
                ):
                    node.pointers.remove(pointerNode)
                    node.pointers.append(pointer)
                    return copiedNodes
            # If cannot overwrite then copy
            while (len(node.pointers) >= node.pointerLimit) and (node is not None):
                # While parent has max amount of pointers we have to copy it
                copyNode = Node(
                    node.key, node.val, time
                )  # create another node that is like parent of node

                # set parent of node in pointer to copyNode
                if pointer[2] is not None:
                    pointer[2].parents[time] = copyNode

                # Set parent of copy to match the original
                parentNode = self.getCurrentParent(node, time)
                copyNode.parents[time] = parentNode

                # Same color
                copyNode.colors[time] = self.getCurrentColor(node, time)

                # Copy should also have the correct pointers
                # One to our node and the other is the latest in other direction
                if pointer[1] == "left":
                    copyNode.pointers = [pointer]
                    copyNode.pointers.append(
                        [time, "right", self.getLatestChild(node, time, "right")]
                    )
                else:
                    copyNode.pointers = [pointer]
                    copyNode.pointers.append(
                        [time, "left", self.getLatestChild(node, time, "left")]
                    )

                # Kill our node
                node.timeDeleted = time

                # Add our copyNode to copied nodes
                copiedNodes.append(copyNode)

                if parentNode is not None:
                    # I want to give parent a pointer to our copy
                    # Find out direction of pointer
                    direction = ""
                    if parentNode.key < copyNode.key:
                        direction = "right"
                    else:
                        direction = "left"

                    if len(parentNode.pointers) == parentNode.pointerLimit:
                        # If there is already a pointer here to our node at the same time we change it to our copy
                        for time1, direction1, node1 in parentNode.pointers:
                            if (
                                time1 == time
                                and direction1 == direction
                                and (node1 == node or node1 is None)
                            ):
                                node1 = copyNode
                                return copiedNodes
                        # If pointers are full i want to repeat above loop but now one lvl higher
                        pointer = [time, direction, copyNode]
                        node = parentNode
                    # else just add the pointer to our copy
                    else:
                        parentNode.pointers.append([time, direction, copyNode])
                        return copiedNodes
                else:  # This means we have to copy the root
                    # copyNode.colors[time] = "black"  # root is black
                    self.roots[time] = copyNode
                    return copiedNodes

    # # Creates a copy of a node and potentialy its parents
    # def copyParentOfNode(self, node, time):
    #     parentNode = self.getCurrentParent(node, time)
    #     while (
    #         parentNode is not None
    #         and len(parentNode.pointers) == parentNode.pointerLimit
    #     ):  # While parent has max amount of pointers we have to copy it
    #         copyParent = Node(
    #             parentNode.key, parentNode.val, time
    #         )  # create another node that is like parent of node

    #         # Set parent of copy to match the original
    #         grandparentNode = self.currentParent(parentNode, time)
    #         copyParent.parents[time] = grandparentNode
    #         # Have same color as copy
    #         copyParent.colors[time] = self.getCurrentColor(parentNode, time)

    #         # Copy should also have the correct pointers
    #         # One to our node and the other is the latest in other direction
    #         if node.key < parentNode.key:  # add pointer from copyParent to node

    #             copyParent.pointers = [[time, "left", node]]
    #             copyParent.pointers.append(
    #                 [time, "right", self.latestChild(parentNode, time, "right")]
    #             )
    #         else:
    #             copyParent.pointers = [[time, "right", node]]
    #             copyParent.pointers.append(
    #                 [time, "left", self.latestChild(parentNode, time, "left")]
    #             )

    #         # Update the parent of our node
    #         node.parents[time] = copyParent

    #         # Now kill the parent
    #         parentNode.timeDeleted = time

    #         # Now i want to give grandparent a pointer to our copyParent
    #         if grandparentNode is not None:
    #             # If pointers are full i want to repeat above loop but now one lvl higher
    #             if len(grandparentNode.pointers) == grandparentNode.pointerLimit:
    #                 parentNode = grandparentNode
    #                 node = copyParent
    #             # else just add the pointer to our copy
    #             else:
    #                 if grandparentNode.key < copyParent.key:
    #                     grandparentNode.pointers.append([time, "right", copyParent])
    #                     break
    #                 else:
    #                     grandparentNode.pointers.append([time, "left", copyParent])
    #                     break

    #         # If it is none we have to copy the root and append it to roots
    #         else:
    #             self.roots.append([time, copyParent])
    #             break

    # insert item at time
    def insert(self, key, val, time):
        # create new Node
        node = Node(key, val, time)

        root = self.getCurrentRoot(time)
        # find parent for node
        parent = self.findParent(root, key, time, None)
        node.parents[time] = parent

        # when adding pointer incase parent gets copied i want to work with new copy
        potentialCopies = []

        if parent is None:  # If parent is none then it is root node
            self.roots[time] = node
            node.colors[time] = "black"
            return

        else:  # If parent is not none we add the pointer from parent to our new node
            if node.key < parent.key:
                potentialCopies = self.addPointer([time, "left", node], parent, time)
            else:
                potentialCopies = self.addPointer([time, "right", node], parent, time)

        if len(potentialCopies) >= 1:
            parent = potentialCopies[0]  # if parent was copied update it to new copy

        if self.getCurrentParent(parent, time) is None:  # If parent is Root Node
            return

        self.fixInsert(node, time)  # Else call for Fix Up

    def minimum(self, node, time):
        leftChild = self.getLatestChild(node, time, "left")
        while leftChild is not None:
            node = leftChild
        return node

    # Also returns nodes that got copied
    def leftRotate(self, node, time):

        y = self.getLatestChild(node, time, "right")  # y is right child of node
        if y is None:
            return

        yLeftChild = self.getLatestChild(y, time, "left")

        # First Assignt new parents to all nodes
        nodeParent = self.getCurrentParent(node, time)
        y.parents[time] = nodeParent
        if nodeParent is None:
            self.roots[time] = y
            # y.colors[time] = "black"

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
        ):  # meand node and y got copied and we have all the connections we need
            return copies
        else:  # No copying done
            # Add Left pointer from y to node a
            copies3 = self.addPointer([time, "left", node], y, time)
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

    def rightRotate(self, node, time):

        y = self.getLatestChild(node, time, "left")  # y is right child of node
        if y is None:
            return

        yRightChild = self.getLatestChild(y, time, "right")

        # First Assignt new parents to all nodes
        nodeParent = self.getCurrentParent(node, time)
        y.parents[time] = nodeParent
        if nodeParent is None:
            self.roots[time] = y
            # y.colors[time] = "black"

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
        ):  # meand node and y got copied and we have all the connections we need
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

    # def rightRotate(self, node, time):

    #     y = self.getLatestChild(node, time, "left")  # y is left child of node
    #     if y is None:
    #         return

    #     yRightChild = self.getLatestChild(y, time, "right")

    #     originalParentOfNode = self.getCurrentParent(node, time)

    #     # Set y to be parent of node
    #     node.parents[time] = y

    #     # Add Right pointer from y to node
    #     self.addPointer([time, "right", node], y, time)

    #     # Remove current pointer from node to y if it exists
    #     for pointer in node.pointers:
    #         if pointer[0] == time and pointer[2] == y:
    #             node.pointers.remove(pointer)

    #     # Add left pointer from node to yLeftChild
    #     self.addPointer([time, "left", yRightChild], node, time)

    #     # Also have to make parent of node parent of y and add pointer from parent to y
    #     y.parents[time] = originalParentOfNode

    #     if originalParentOfNode is not None:
    #         if originalParentOfNode.key > node.key:
    #             self.addPointer([time, "left", y], originalParentOfNode, time)
    #         else:
    #             copies = self.addPointer([time, "right", y], originalParentOfNode, time)
    #             return copies

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
                        grandparent = copies[0]
                        parent = self.getCurrentParent(node, time)
                    parent.colors[time] = "black"
                    grandparent.colors[time] = "red"
                    self.leftRotate(grandparent, time)
        root = self.getCurrentRoot(time)
        root.colors[time] = "black"

    def printTree(self, time):
        root = self.getCurrentRoot(time)
        self.printSubTree(root, time, "", True)

    def printSubTree(self, node, time, indent, last):
        if node is not None:
            print(
                indent,
                "R--- " if last else "L--- ",
                node.key,
                "(",
                self.getCurrentColor(node, time),
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


# Create an instance of the persistent red-black tree
tree = persistentRBTree()

# Insert nodes
tree.insert(10, "A", 1)
tree.insert(20, "B", 2)
tree.insert(15, "C", 3)
tree.insert(25, "D", 3)
tree.insert(5, "E", 3)
tree.insert(3, "F", 3)

# Print the tree structure at different times
print("Tree at time 1:")
tree.printTree(1)

print("\nTree at time 2:")
tree.printTree(2)

print("\nTree at time 3:")
tree.printTree(3)

print("\nTree at time 4:")
tree.printTree(4)

print("\nTree at time 5:")
tree.printTree(5)

print("\nTree at time 6:")
tree.printTree(6)
