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
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		std::function<void(Node**,int)> helper;
		helper = [&point,id,&helper](Node **root,int depth)
		{
			auto dim = depth %2; 
			if(*root == NULL)
			{
				*root = new Node(point,id);
			}
			else if (point[dim]< (*root)->point[dim])
			{
				helper(&((*root)->left),depth+1);
			}
			else
			{
				helper(&((*root)->right),depth+1);
			}
		};
		helper(&root,0);


	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		std::function<void(Node*,int)> helper;
		helper = [target,distanceTol,&ids,&helper](Node *root,int depth)
		{
			if(root == NULL)
				return;

			if(target[0]-distanceTol < root->point[0] &&
			target[0]+distanceTol > root->point[0] &&
			target[1]-distanceTol < root->point[1] &&
			target[1]+distanceTol > root->point[1])
			{
				auto distance = sqrt((target[0] -root->point[0])*(target[0] -root->point[0]) +
				(target[1] -root->point[1])*(target[1] -root->point[1]));
				if(distance <=distanceTol)
				{
					ids.push_back(root->id);
				}
			}

			if(target[depth%2]-distanceTol < root->point[depth%2])
			{
				helper(root->left,depth+1);
			}
			if(target[depth%2]+distanceTol > root->point[depth%2])
			{
				helper(root->right,depth+1);
			}

		};

		helper(this->root,0);

		return ids;
	}
	

};




