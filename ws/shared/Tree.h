#pragma once

#include "TreeNode.h"
#include "AMPCore.h"
#include <vector>

class Tree{

    public:
        Tree();
        ~Tree();
        void bfs(TreeNode* goal);
        void addEdge(TreeNode* parent, TreeNode* child);
        void deleteTree(TreeNode* root);

        // fields:
        TreeNode* root;
};