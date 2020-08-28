/* \author William Nguyen */
// KD tree Data Type

#ifndef KDTREE_H
#define KDTREE_H

#include <vector>


struct Node
{
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;

    Node(std::vector<float>& p, int setId);
    ~Node();
};

struct KdTree
{
    Node* root;
    int n_dims;

    KdTree();
    ~KdTree();
    void insert(std::vector<float>& p, int id);
    std::vector<int> search(std::vector<float>& target, float distanceTol);

    void _insert(Node** node, std::vector<float>& point, int id, int dim_level);
    void _search(Node* node, std::vector<float>& target, int dim_level, float distanceTol, std::vector<int>& ids);
    inline float _pointValue(std::vector<float>& p, int dim_level);
    inline float _calcDistance(std::vector<float>& a, std::vector<float>& b);
    inline bool _aWithinB(float a, float b, float distanceTol);
    inline bool _aWithinB(std::vector<float>& a, std::vector<float>& b, float distanceTol);
};

#endif