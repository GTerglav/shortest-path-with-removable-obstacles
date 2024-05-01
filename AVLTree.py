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
        T2 = x.right

        x.right = y
        y.left = T2

        y.height = 1 + max(self.getHeight(y.left), self.getHeight(y.right))
        x.height = 1 + max(self.getHeight(x.left), self.getHeight(x.right))

        return x

    def rotateLeft(self, x):
        y = x.right
        T2 = y.left

        y.left = x
        x.right = T2

        x.height = 1 + max(self.getHeight(x.left), self.getHeight(x.right))
        y.height = 1 + max(self.getHeight(y.left), self.getHeight(y.right))

        return y

    def insert(self, node, key):
        if not node:
            return TreeNode(key)
        elif key < node.key:
            node.left = self.insert(node.left, key)
        else:
            node.right = self.insert(node.right, key)

        node.height = 1 + max(self.getHeight(node.left), self.getHeight(node.right))

        balance = self.getBalance(node)

        # Left Left Case
        if balance > 1 and key < node.left.key:
            return self.rotateRight(node)

        # Right Right Case
        if balance < -1 and key > node.right.key:
            return self.rotateLeft(node)

        # Left Right Case
        if balance > 1 and key > node.left.key:
            node.left = self.rotateLeft(node.left)
            return self.rotateRight(node)

        # Right Left Case
        if balance < -1 and key < node.right.key:
            node.right = self.rotateRight(node.right)
            return self.rotateLeft(node)

        return node

    def preorder(self, root):
        if root:
            print("{0} ".format(root.key), end="")
            self.preorder(root.left)
            self.preorder(root.right)

# Usage example:
# Create an AVL tree object
avl_tree = AVLTree()
root = None

# Insert elements into the AVL tree
keys = [10, 20, 30, 40, 50, 25]
for key in keys:
    root = avl_tree.insert(root, key)

# Print the preorder traversal of the AVL tree
print("Preorder traversal of the constructed AVL tree is:")
avl_tree.preorder(root)
