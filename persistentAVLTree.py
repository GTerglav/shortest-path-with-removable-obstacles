import numpy as np


class TreeNode:
    def __init__(self, key):
        self.key = key
        self.left = None
        self.right = None
        self.height = 1


class AVLTree:
    def getHeight(self, node):
        if not node:
            return 0
        return node.height

    def getBalance(self, node):
        if not node:
            return 0
        return self.getHeight(node.left) - self.getHeight(node.right)

    def rotateRight(self, y):
        x = y.left
        if x is None:
            # print("Avl tree rotate right not working")
            return y
        T2 = x.right

        x.right = y
        y.left = T2

        y.height = 1 + max(self.getHeight(y.left), self.getHeight(y.right))
        x.height = 1 + max(self.getHeight(x.left), self.getHeight(x.right))

        return x

    def rotateLeft(self, x):
        y = x.right
        if y is None:
            # print("Avl tree rotate left not working")
            return x
        T2 = y.left

        y.left = x
        x.right = T2

        x.height = 1 + max(self.getHeight(x.left), self.getHeight(x.right))
        y.height = 1 + max(self.getHeight(y.left), self.getHeight(y.right))

        return y

    def insert(self, node, edge):
        if not node:
            return TreeNode(edge)
        elif edge[2] < node.key[2]:
            node.left = self.insert(node.left, edge)
        else:
            node.right = self.insert(node.right, edge)

        node.height = 1 + max(self.getHeight(node.left), self.getHeight(node.right))

        balance = self.getBalance(node)

        # Left Left Case
        if balance > 1 and edge[2] < node.left.key[2]:
            return self.rotateRight(node)

        # Right Right Case
        if balance < -1 and edge[2] > node.right.key[2]:
            return self.rotateLeft(node)

        # Left Right Case
        if balance > 1 and edge[2] > node.left.key[2]:
            node.left = self.rotateLeft(node.left)
            return self.rotateRight(node)

        # Right Left Case
        if balance < -1 and edge[2] < node.right.key[2]:
            node.right = self.rotateRight(node.right)
            return self.rotateLeft(node)

        return node
