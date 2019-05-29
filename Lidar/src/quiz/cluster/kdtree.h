/* \author Aaron Brown and Shiva Verma*/
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

// TODO: Fill in this function to insert a new point into the tree
void insertHelper(Node** node, std::vector<float> point, int id, int depth)
{
    if(*node == NULL)
        *node = new Node(point, id);
    else
    {
        int ind = depth % 2;
        
        if(point[ind] <= (*node)->point[ind])
            insertHelper(&((*node)->left), point, id, depth+1);
        else
            insertHelper(&((*node)->right), point, id, depth+1);
    }
}

void searchHelper(Node* node, std::vector<float> target, float distanceTol, int depth, std::vector<int> &ids)
{
    if(node == NULL)
        return;
    
    int ind = depth % 2;
    
    if(abs(node->point[ind] - target[ind]) > distanceTol)
    {
        if(node->point[ind] >= target[ind])
            searchHelper(node->left, target, distanceTol, depth+1 , ids);
        else
            searchHelper(node->right, target, distanceTol, depth+1 , ids);
    }
    else
    {
        float dis = sqrt(pow(node->point[0] - target[0], 2) + pow(node->point[1] - target[1], 2));
        
        if(dis < distanceTol)
            ids.push_back(node->id);
            
        searchHelper(node->left, target, distanceTol, depth+1 , ids);
        searchHelper(node->right, target, distanceTol, depth+1 , ids);
    }
}

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
        insertHelper(&root, point, id, 0);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
        
        searchHelper(root, target, distanceTol, 0, ids);
        
		return ids;
	}
};





