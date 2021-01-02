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

	bool visited;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL), visited(false)
	{}
};

struct KdTree
{
	Node* root;
	uint dims;

	KdTree()
	: root(NULL), dims(2)
	{}

	void setDimentions(int _dims)
	{
		dims = _dims;
	}

	void insertHelper(Node *node, std::vector<float> point, int index, uint32_t depth)
	{
		uint currentDim = depth % dims;

		if ( point[currentDim] < node->point[currentDim] )
		{
			if (!node->left)
				node->left = new Node(point, index);
			else
				insertHelper(node->left, point, index, ++depth);
		}
		else
		{
			if (!node->right)
				node->right = new Node(point, index);
			else
				insertHelper(node->right, point, index, ++depth);				
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// if first point
		if (!root)
			root = new Node(point, id);
		else
			insertHelper(root, point, id, 0);

	}


	void searchHelper(Node *node, std::vector<float> target, float distanceTol, int depth, std::vector<int> &ids)
	{
		if (!node)
			return;

		// if point inside the box
		if ( ( abs(node->point[0] - target[0]) <= distanceTol) && ( abs(node->point[1] - target[1]) <= distanceTol) )
		{
			auto distance = hypot( abs(node->point[0] - target[0]) , abs(node->point[1] - target[1]) );
			if (distance <= distanceTol)
				ids.push_back(node->id);
		}

		auto dim = depth % dims;
		if (node->point[dim] >  (target[dim] - distanceTol) )
			searchHelper(node->left , target, distanceTol, depth+1, ids);
		if (node->point[dim] <=  (target[dim] + distanceTol) )
			searchHelper(node->right, target, distanceTol, depth+1, ids);
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, target, distanceTol, 0, ids);

		return ids;
	}
	
};




