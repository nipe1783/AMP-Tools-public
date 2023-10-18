#pragma once

#include <vector>

class TreeNode{

    public:
        TreeNode();
        ~TreeNode();

        TreeNode *parent;
        std::vector<TreeNode*> children;
        std::pair<int, int> cell;
};
