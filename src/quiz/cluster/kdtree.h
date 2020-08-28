/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;
	int n_dims;

	KdTree()
	: root(NULL), n_dims(-1)
	{}

	void insert(std::vector<float> point, int id)
	{
		_insert(&root, point, id, 0);
	}

	void _insert(Node** node, std::vector<float>& point, int id, int dim_level) {
		if (*node == nullptr) {
			*node = new Node(point, id);
			if (n_dims == -1) {
				n_dims = point.size();
			}
		} else if (point[dim_level] < (*node)->point[dim_level]) {
			_insert(&((*node)->left), point, id, (dim_level + 1) % n_dims);
		} else {
			_insert(&((*node)->right), point, id, (dim_level + 1) % n_dims);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		_search(root, target, 0, distanceTol, ids);
		return ids;
	}

	inline bool a_within_b(float a, float b, float distanceTol) {
		return a >= (b - distanceTol) && a <= (b + distanceTol);
	}

	inline float calc_distance(std::vector<float> a, std::vector<float> b) {
		float distance = 0;
		for (int i = 0; i < a.size(); ++i) {
			distance += (a[i] - b[i]) * (a[i] - b[i]);
		}
		return sqrt(distance);
	}

	inline bool a_within_b(std::vector<float>& a, std::vector<float>& b, float distanceTol)
	{
		for (int i = 0; i < n_dims; ++i) {
			if (!a_within_b(a[i], b[i], distanceTol)) {
				return false;
			}
		}
		return true;
	}

	void _search(Node* node, std::vector<float>& target, int dim_level, float distanceTol, std::vector<int>& ids) {
		if (node == nullptr) {
			return;
		}

		if (a_within_b(node->point, target, distanceTol)) {
			if (calc_distance(node->point, target) <= distanceTol) {
				ids.push_back(node->id);
			}
		}

		if ((target[dim_level]-distanceTol) < node->point[dim_level]) {
			_search(node->left, target, (dim_level + 1) % n_dims, distanceTol, ids);
		}
		if ((target[dim_level]+distanceTol) > node->point[dim_level]) {
			_search(node->right, target, (dim_level + 1) % n_dims, distanceTol, ids);
		}
	}
	

};




