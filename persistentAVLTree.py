import numpy as np


class AVLNode:
    def __init__(self, key):
        self.key = key
        self.left = None
        self.right = None
        self.height = 1

class PersistentAVLTree:
    def __init__(self):
        self.root = None
        self.version = 0
        self.versions = {}

    def insert(self, root, key):
        if not root:
            return AVLNode(key)
        
        if key < root.key:
            root.left = self.insert(root.left, key)
        else:
            root.right = self.insert(root.right, key)
        
        root.height = 1 + max(self.getHeight(root.left), self.getHeight(root.right))
        
        balance = self.getBalance(root)
        
        if balance > 1 and key < root.left.key:
            return self.rightRotate(root)
        
        if balance < -1 and key > root.right.key:
            return self.leftRotate(root)
        
        if balance > 1 and key > root.left.key:
            root.left = self.leftRotate(root.left)
            return self.rightRotate(root)
        
        if balance < -1 and key < root.right.key:
            root.right = self.rightRotate(root.right)
            return self.leftRotate(root)
        
        return root

    def rightRotate(self, z):
        y = z.left
        T3 = y.right
        
        y.right = z
        z.left = T3
        
        z.height = 1 + max(self.getHeight(z.left), self.getHeight(z.right))
        y.height = 1 + max(self.getHeight(y.left), self.getHeight(y.right))
        
        return y

    def leftRotate(self, z):
        y = z.right
        T2 = y.left
        
        y.left = z
        z.right = T2
        
        z.height = 1 + max(self.getHeight(z.left), self.getHeight(z.right))
        y.height = 1 + max(self.getHeight(y.left), self.getHeight(y.right))
        
        return y

    def getHeight(self, root):
        if not root:
            return 0
        return root.height

    def getBalance(self, root):
        if not root:
            return 0
        return self.getHeight(root.left) - self.getHeight(root.right)

    def insertVersion(self, key):
        self.version += 1
        self.root = self.insert(self.root, key)
        self.versions[self.version] = self.root

    def getRoot(self, version):
        return self.versions.get(version)

    # Find and return the item with greatest key less than or equal to x.
    def access(self, x, node, time):
        return "TODO"

    def insert2(self, item, node, time):
        return "TODO"

    def delete(self, item, node, time):
        return "TODO"
