class Node:
    def __init__(self, val, time):
        self.val = val  # Value of Node
        self.time = time  # Time created
        self.parent = None  # Parent of Node
        self.pointers = [
            (time, "left", None),
            (time, "right", None),
        ]  # list of pointers of form (timeCreated, "left or right", node) Max 3 pointers
        self.color = 1  # Red Node as new node is always inserted as Red Node


class persistentRBTree:
    def __init__(self):
        self.root = None

    # find and return item in tree with greatest key value <= to x at time. Need to save the node at which we last went right
    def access(self, node, x, time, lastRight):
        if node.val == None:
            if lastRight is not None:
                return lastRight
            else:
                return None
        if node.val == x:
            return node
        if node.val < x:
            for pointer in node.pointers:
                if pointer[1] == "left":
                    return "blabla"

        return "TODO"

    # insert item at time
    def insert(self, node, item, time):
        return "TODO"

    # insert item at time
    def delete(self, node, item, time):
        return "TODO"
