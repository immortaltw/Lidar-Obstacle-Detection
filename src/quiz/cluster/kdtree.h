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

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// Insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insert_helper(&root, point, id, 2, 0);
	}

	void insert_helper(Node **node, std::vector<float> point, int id, int dim, int depth) {
		if (!(*node)) {
			*node = new Node(point, id);
			return;
		}

		int idx = depth % dim;
		if (point[idx] < (*node)->point[idx]) {
			insert_helper(&((*node)->left), point, id, dim, depth+1);
		} else {
			insert_helper(&((*node)->right), point, id, dim, depth+1);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search_helper(root, target, distanceTol, 2, 0, ids);
		return ids;
	}

	void search_helper(Node *node, std::vector<float> target, float distanceTol, int dim, int depth, std::vector<int> &ids) {
		float xmin = target[0] - distanceTol;
		float xmax = target[0] + distanceTol;
		float ymin = target[1] - distanceTol;
		float ymax = target[1] + distanceTol;

		if (!node) return;

		if (node->point[0] >= xmin &&
			node->point[0] <= xmax &&
			node->point[1] >= ymin &&
			node->point[1] <= ymax) {
			float dist = sqrt((node->point[0]-target[0])*(node->point[0]-target[0]) + (node->point[1]-target[1])*(node->point[1]-target[1]));

			if (dist <= distanceTol) ids.push_back(node->id);
		}

		int idx = depth % dim;
		if (target[idx] - distanceTol < node->point[idx]) {
			search_helper(node->left, target, distanceTol, dim, depth+1, ids);
		}
		if (target[idx] + distanceTol > node->point[idx]) {
			search_helper(node->right, target, distanceTol, dim, depth+1, ids);
		}
	}
};

// // Structure to represent node of kd tree
// struct Node
// {
// 	std::vector<float> point;
// 	int id;
// 	Node* left;
// 	Node* right;

// 	Node(std::vector<float> arr, int setId)
// 	:	point(arr), id(setId), left(NULL), right(NULL)
// 	{}
// };

// struct KdTree
// {
// 	Node* root;
// 	int dim_;

// 	KdTree(int dim=2)
// 	: root(NULL), dim_(dim)
// 	{}

// 	void insert(std::vector<float>& point, int id)
// 	{
// 		// Insert a new point into the tree
// 		// the function should create a new node and place correctly with in the root
// 		if (point.size() != dim_) {
// 			std::cout << "Wrong dimension!" << std::endl;
// 			return;
// 		}
// 		insert_helper(&root, point, id, dim_, 0);
// 	}

// 	void insert_helper(Node **node, std::vector<float>& point, int id, int dim, int depth) {
// 		if (!(*node)) {
// 			*node = new Node(point, id);
// 			return;
// 		}

// 		int idx = depth % dim;
// 		if (point[idx] < (*node)->point[idx]) {
// 			insert_helper(&((*node)->left), point, id, dim, depth+1);
// 		} else {
// 			insert_helper(&((*node)->right), point, id, dim, depth+1);
// 		}
// 	}

// 	// return a list of point ids in the tree that are within distance of target
// 	std::vector<int> search(const std::vector<float>& target, float distanceTol)
// 	{
// 		std::vector<int> ids;
// 		search_helper(root, target, distanceTol, dim_, 0, ids);
// 		return ids;
// 	}

// 	void search_helper(Node *node, const std::vector<float>& target, float distanceTol, int dim, int depth, std::vector<int> &ids) {
// 		if (!node) return;

// 		if (within_distanceTol_(node, target, distanceTol, dim)) {
// 			float dist = calculate_dist(node, target, dim);
// 			if (dist <= distanceTol) ids.push_back(node->id);
// 		}

// 		int idx = depth % dim;
// 		if (target[idx] - distanceTol < node->point[idx]) {
// 			search_helper(node->left, target, distanceTol, dim, depth+1, ids);
// 		}
// 		if (target[idx] + distanceTol > node->point[idx]) {
// 			search_helper(node->right, target, distanceTol, dim, depth+1, ids);
// 		}
// 	}

// 	bool within_distanceTol_(Node *node, const std::vector<float>& target, float distanceTol, int dim) {
// 		bool res = true;

// 		for (int i=0; i<dim; ++i) {
// 			float min_ = target[i] - distanceTol;
// 			float max_ = target[i] - distanceTol;
// 			res &= (node->point[i] >= min_ && node->point[i] <= max_);
// 		}
// 		return res;
// 	}

// 	float calculate_dist(Node *node, const std::vector<float>& target, int dim) {
// 		float res = 0.0;
// 		for (int i=0; i<dim; ++i) {
// 			res += (node->point[i]-target[i])*(node->point[i]-target[i]);
// 		}
// 		return sqrt(res);
// 	}
// };
