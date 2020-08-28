#include <iostream>
#include <cmath>
#include "kdtree.h"

/*******************************************
 * Node Struct
 *******************************************/
Node::Node(std::vector<float>& setPoint, int setId)
: point(setPoint), id(setId), left(nullptr), right(nullptr) {}

Node::~Node()
{
    if (left != nullptr) delete left;
    if (right != nullptr) delete right;
}

/*******************************************
 * KdTree Struct
 *******************************************/
KdTree::KdTree()
: root(nullptr), n_dims(-1) {}

/**
 * Helper Function to 
 */
KdTree::~KdTree() {
    if (root != nullptr) delete root;
}

/**
 * Helper Function to extract point values by dimension label
 */
inline float KdTree::_pointValue(std::vector<float>& p, int dim_level) {
    return p[dim_level % n_dims];
}

/**
 * Insert Helper Function
 */
void KdTree::_insert(Node** node, std::vector<float>& point, int id, int dim_level) {
    if (*node == nullptr) {
        *node = new Node(point, id);
    } else if (_pointValue(point, dim_level) < _pointValue((*node)->point, dim_level)) {
        _insert(&((*node)->left), point, id, dim_level + 1);
    } else {
        _insert(&((*node)->right), point, id, dim_level + 1);
    }
}

/**
 * KdTree Insert - inserts point and accompanying id into Kd Tree
 */
void KdTree::insert(std::vector<float>& point, int id) {
    if (n_dims == -1) {
        n_dims = point.size();
    }
    _insert(&root, point, id, 0);
}

/**
 * Calculates the distance between points a and b
 */
inline float KdTree::_calc_distance(std::vector<float>& a, std::vector<float>& b) {
    float distance = 0;
    for (int i = 0; i < n_dims; ++i) {
        float c = _pointValue(a, i) - _pointValue(b, i);
        distance += c*c;
    }
    return sqrt(distance);
}

/**
 * Checks if scalar value a is within distance tolerance of scalar value b
 */
inline bool KdTree::_a_within_b(float a, float b, float distanceTol) {
    return a >= (b - distanceTol) && a <= (b + distanceTol);
}

/**
 * Checks if point a is within distance tolerance of point b
 */
inline bool KdTree::_a_within_b(std::vector<float>& a, std::vector<float>& b, float distanceTol)
{
    for (int i = 0; i < n_dims; ++i) {
        float ai = _pointValue(a,i);
        float bi = _pointValue(b,i);
        if (!_a_within_b(ai, bi, distanceTol)) {
            return false;
        }
    }
    return true;
}

/**
 * Search Helper Function - Preorder tree traversal to keep track of nearby points in the KD tree
 */
void KdTree::_search(Node* node, std::vector<float>& target, int dim_level, float distanceTol, std::vector<int>& ids) {
    if (node == nullptr) {
        return;
    }

    if (_a_within_b(node->point, target, distanceTol)) {
        if (_calc_distance(node->point, target) <= distanceTol) {
            ids.push_back(node->id);
        }
    }

    float target_i = _pointValue(target, dim_level);
    float node_point_i = _pointValue(node->point, dim_level);
    if ((target_i-distanceTol) < node_point_i) {
        _search(node->left, target, dim_level + 1, distanceTol, ids);
    }
    if ((target_i+distanceTol) > node_point_i) {
        _search(node->right, target, dim_level + 1, distanceTol, ids);
    }
}

/**
 * KdTree Search - searches Kd Tree for nearby points and returns their index values
 **/
std::vector<int> KdTree::search(std::vector<float>& target, float distanceTol) {
    std::vector<int> ids;
    _search(root, target, 0, distanceTol, ids);
    return ids;
}