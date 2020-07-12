/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}
};

struct KdTree_simple
{
	Node *root;

	KdTree_simple()
		: root(NULL)
	{
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root

		insertNode(&root, 0, point, id);

		cout << "------------------------------" << endl;
	}

	// void insert(BinaryTreeNode **node, int data)
	//    {
	//       if(*node == NULL)
	//       {
	//         *node = getNewNode(data);
	//       }
	//       else if(data < (*node)->data)
	//       {
	//         insert(&(*node)->left, data);
	//       }
	//       else
	//       {
	//         insert(&(*node)->right, data);
	//       }
	//    }
	// 	void insert(BinaryTreeNode *&node, int data)
	//    {
	//       if(node == NULL)
	//       {
	//         node = getNewNode(data);
	//       }
	//       else if(data < node->data)
	//       {
	//         insert(node->left, data);
	//       }
	//       else
	//       {
	//         insert(node->right, data);
	//       }
	//    }

	// void insertNode(Node *&node, uint depth, std::vector<float> point, int id)
	// {
	// 	if (node == NULL)
	// 	{
	// 		node = new Node(point, id);
	// 	}
	// }

	/// depth starts at 0 and increments as we progress down recursively
	void insertNode(Node **node, uint depth, std::vector<float> point, int id)
	{
		
		// cout << "Node: " << &((*node)->id) << ", passed depth: " << depth << ", point: ";

		// for (int i = 0; i < point.size(); i ++) {
		// 	cout << point[i] << ", ";
		// }
		// cout << endl;

		// cout << point[0] << endl;
		// cout << ((*node)->point[depth_current]) << endl;
		// cout << "-- This Node: " << ((*node)->id) << endl;

		if (*node == NULL)
		{
			// cout << "null" << endl;
			*node = new Node(point, id);
		}
		else
		{
			// cout << "not null" << endl;
			// revised to handle 2d and 3d...
			uint dimension2check = depth % point.size();


			// cout << "dimension2check: " << dimension2check << endl;
			// cout << "This Node id: " << ((*node)->id) << endl;
			// cout << "Next Left Node id: " << &((*node)->left)->id << endl;
			// cout << "Next Right Node id: " << &((*node)->right)->id << endl;

			// depth current defines if should compare an x or y value
			// if its even, then compare x, if odd compare y
			// compare the point that was passed in to the point stored at this node
			if (point[dimension2check] < ((*node)->point[dimension2check]))
			{
				// insert left
				insertNode(&((*node)->left), depth + 1, point, id);
			}
			else
			{
				// insert right
				insertNode(&((*node)->right), depth + 1, point, id);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		// cout << "---------------------------" << endl;
		// cout << "search" << endl;

		std::vector<int> ids;

		searchNode(target, root, 0, distanceTol, ids);

		// cout << " search - exit" << endl;
		return ids;
	}

	void searchNode(std::vector<float> target, Node *node, uint depth, float distanceTol, std::vector<int> &ids)
	{
		// cout << "searchNode - entry" << endl;
		// cout << "target size: " << target.size() << endl;

		if (node != NULL)
		{
		cout << "This Node id: " << ((node)->id) << endl;
			// cout << "not null, checking if inside tolerances" << endl;

			bool inTolerance = true;
			// revised if statement so that it can be used on 2d and 3d.
			// loop through the taregt vector and check agains the point coordinates.
			for (int i = 0; target.size(); i++)
			{
				if (!((node->point[i] >= (target[i] - distanceTol)) && (node->point[i] >= (target[i] - distanceTol))))
					inTolerance = false;
					break;
			}

			if (inTolerance)
			{
				// cout << "inside rough tolerance check" << endl;
				// inside tolerance.
				// caluclate euclidean using pythagorus

// revised to cope with mult dimensions.
// for each dimension, calc the delat between point and target, and square it. Add to running total. Finally sqrt...

			float running_sqd_error = 0;
			// cout << "target size: " << target.size()<< endl;
			for (int i = 0; i < target.size(); i++)
			{
				running_sqd_error += pow(node->point[i] - target[i], 2);
				// cout << "running_sqd_error: " << running_sqd_error << endl;
			}
			float dist_to_node = sqrt(running_sqd_error);

				// float delta_x = node->point[0] - target[0];
				// float delta_y = node->point[1] - target[1];
				// float dist_to_node = sqrt(pow(delta_x, 2) + pow(delta_y, 2));

				// // cout << "debugging: " << 
				 cout << "euclidean: " << dist_to_node << endl;

				if (dist_to_node <= distanceTol)
				{
					// cout << "inside the euclidean check - add node ID" << endl;
					ids.push_back(node->id);
				}
			}

			// if ((node->point[0] >= (target[0] - distanceTol)) && (node->point[0] <= (target[0] + distanceTol)) && ((node->point[1] >= (target[1] - distanceTol)) && (node->point[1] <= (target[1] + distanceTol))))
			// {
			// 	cout << "inside rough tolerance check" << endl;
			// 	// inside tolerance.
			// 	// caluclate euclidean using pythagorus
			// 	float delta_x = node->point[0] - target[0];
			// 	float delta_y = node->point[1] - target[1];

			// 	float dist_to_node = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
			// 	cout << "euclidean: " << dist_to_node << endl;

			// 	if (dist_to_node <= distanceTol)
			// 	{
			// 		cout << "inside the euclidean check - add node ID" << endl;
			// 		ids.push_back(node->id);
			// 	}
			// }




			// cout << "check across boundary" << endl;
			// check if the depth level is odd or even, which tells us which dimension to check.
			// revised to cope with varying dimensions
			uint dimension2check = depth % target.size();

			// check the lower boundary
			// cout << "check lower" << endl;
			if ((target[dimension2check] - distanceTol) < node->point[dimension2check])
			{
				// cout << "check lower - inside" << endl;
				searchNode(target, node->left, depth + 1, distanceTol, ids);
			}
			// check the upper boundary
			// cout << "check upper" << endl;
			if ((target[dimension2check] + distanceTol) > node->point[dimension2check])
			{
				// cout << "check upper - inside" << endl;
				searchNode(target, node->right, depth + 1, distanceTol, ids);
			}
		}
		else
		{
			// cout << "NODE NULL" << endl;
		}

		// cout << "exiting searchnode" << endl;
	}
};
