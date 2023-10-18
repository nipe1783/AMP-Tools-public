#include "Tree.h"
#include "TreeNode.h"

Tree::Tree(){
};

Tree::~Tree(){
};

void Tree::addEdge(TreeNode* parent, TreeNode* child){
    parent->children.push_back(child);
    child->parent = parent;
}

void Tree::bfs(TreeNode* goal){
    std::vector<TreeNode*> path;
}

void Tree::deleteTree(TreeNode* root){
    if (root == nullptr) {
        return; // Base case: Node is null
    }

    // Delete all child nodes
    for (TreeNode* child : root->children) {
        deleteTree(child);
    }

    // Delete the current node
    delete root;
}