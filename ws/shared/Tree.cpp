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