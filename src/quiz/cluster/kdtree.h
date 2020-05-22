/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <vector>


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
		// DONE: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		//Start recursively  inserting a new value to a KDtree
		insertHelper(&root, point,id, 0 );

	}

	void insertHelper(Node** node, std::vector<float> point, int id, uint depth){
		
		//Check if current node is available to the inserted
		if(*node == NULL){
			//Create a new node at the empty position
			*node = new Node(point,id);
		}

		else{
			//check if x or y will be explrored based on the depth
			uint cd = depth % 2;
			//move to left
			if(point[cd] < ((*node)->point[cd])){
				//continue traversing the tree passing the left node as reference recursively
				insertHelper(&((*node)->left),point,id,depth+1);
			}//move to right
			else{
				//continue traversing the tree passing the right node as reference recursively
				insertHelper(&((*node)->right),point,id,depth+1);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		
		std::vector<int> ids;

		searchHelper(root, target, ids, 0, distanceTol);
				
		return ids;
	}
	
	void searchHelper(Node* node, std::vector<float> target, std::vector<int>& ids, int depth, float distanceTol){
		
		if(node != NULL){
			//check if point is within the box
			float distances[] = {target[0]-node->point[0],target[1]-node->point[1] };
			float distance = sqrt(distances[0]*distances[0] + distances[1]*distances[1]);
			//if it is within the box, then add to output list
			if(distance <= distanceTol){
				ids.push_back(node->id);
			}

			// explore right and left leaves is distance is within boundaries
			uint cd = depth % 2;
			if(distances[cd] < distanceTol){
				searchHelper(node->left,target, ids, depth+1,distanceTol);
			}
			if(distances[cd] > -distanceTol){
				searchHelper(node->right, target, ids, depth+1,distanceTol);
			}
		}
		
		
	}

};