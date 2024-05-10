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
            print("Avl tree rotate right not working")
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
            print("Avl tree rotate left not working")
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

    def delete(self, node, key):
        if not node:
            return node

        # Perform standard BST delete
        if key < node.key[2]:
            node.left = self.delete(node.left, key)
        elif key > node.key[2]:
            node.right = self.delete(node.right, key)
        else:
            # Node with only one child or no child
            if node.left is None:
                temp = node.right
                node = None
                return temp
            elif node.right is None:
                temp = node.left
                node = None
                return temp

            # Node with two children: Get the inorder successor
            # (smallest in the right subtree)
            temp = self.getMinValueNode(node.right)

            # Copy the inorder successor's content to this node
            node.key = temp.key

            # Delete the inorder successor
            node.right = self.delete(node.right, temp.key[2])

        # If the tree had only one node then return
        if node is None:
            return node

        # Update the height of the current node
        node.height = 1 + max(self.getHeight(node.left), self.getHeight(node.right))

        # Get the balance factor
        balance = self.getBalance(node)

        # Balancing the tree

        # Left Left Case
        if balance > 1 and self.getBalance(node.left) >= 0:
            return self.rotateRight(node)

        # Left Right Case
        if balance > 1 and self.getBalance(node.left) < 0:
            node.left = self.rotateLeft(node.left)
            return self.rotateRight(node)

        # Right Right Case
        if balance < -1 and self.getBalance(node.right) <= 0:
            return self.rotateLeft(node)

        # Right Left Case
        if balance < -1 and self.getBalance(node.right) > 0:
            node.right = self.rotateRight(node.right)
            return self.rotateLeft(node)

        return node

    def deleteByVertex(self, node, vertex):
        if not node:
            return node

        # Traverse left and right subtrees recursively
        node.left = self.deleteByVertex(node.left, vertex)
        node.right = self.deleteByVertex(node.right, vertex)

        # If current node represents an edge including the given vertex, delete it
        if np.array_equal(node.key[0], vertex) or np.array_equal(node.key[1], vertex):
            return self.delete(node, node.key[2])

        return node

    def getMinValueNode(self, node):
        current = node

        # Loop down to find the leftmost leaf
        while current.left is not None:
            current = current.left

        return current

    def search(self, node, key):
        if node is None or node.key[2] == key:
            return node

        if node.key[2] < key:
            return self.search(node.right, key)

        return self.search(node.left, key)

    def preorder(self, root):
        listOfElements = []
        if root:
            listOfElements.append(root.key)
            # print("{0} ".format(root.key), end="")
            listOfElements += self.preorder(root.left)
            listOfElements += self.preorder(root.right)
        return listOfElements

    def printStructure(self, node, level=0, prefix="Root: "):
        if node is not None:
            print(" " * (level * 4) + prefix + str(node.key))
            if node.left is not None or node.right is not None:
                self.printStructure(node.left, level + 1, "L--- ")
                self.printStructure(node.right, level + 1, "R--- ")


# # Usage example:
# # Create an AVL tree object
# avl_tree = AVLTree()
# root = None

# # Insert elements into the AVL tree
# edges = [
#     ((1, 2), (3, 4), 5, 6),
#     ((2, 3), (4, 5), 3, 5),
#     ((3, 4), (5, 6), 7, 2),
#     ((3, 4), (5, 6), 8, 9),
#     ((3, 4), (5, 6), 9, 11),
#     ((3, 4), (5, 6), 10, 13),
# ]  # Example edges
# for edge in edges:
#     root = avl_tree.insert(root, edge)

# # Print the preorder traversal of the AVL tree
# print("Preorder traversal of the constructed AVL tree is:")
# print(avl_tree.preorder(root))
# print("Structure is")
# avl_tree.printStructure(root)

# key_to_delete = 10
# root = avl_tree.delete(root, key_to_delete)

# # Print the structure of the AVL tree after deletion
# print("\nStructure of the AVL tree after deletion of key", key_to_delete, "is:")
# avl_tree.printStructure(root)
# result = avl_tree.search(root, 9)
# if result:
#     print("Element", 9, "found in the AVL tree:", result.key)
# else:
#     print("Element", 9, "not found in the AVL tree.")
