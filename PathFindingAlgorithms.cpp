#include <cstdlib>
#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <iterator>
#include <algorithm>
#include <math.h>
#include <sstream>     
#include <cmath> 
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <numeric> 
#include <locale>
#include <set>
#include <stack>
#include <stdio.h>
#include <queue>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <regex>
#include <bitset>
#include <typeinfo>
#include <functional>
#include <list>
#include <stdlib.h>
#include <time.h>
using namespace std;

/**
* Path Finding Algorithms.
* 
* @license    http://opensource.org/licenses/MIT The MIT License (MIT)
* @author     Omar El Gabry <omar.elgabry.93@gmail.com>
*/

/**
* A node.
*/
//
struct Node {

	/**
	* the value
	*
	* @var int
	*/
	int value;

	/**
	* the distance from the root node(or from root node + heuristics)
	* the distance could be useful to search for the shortest path between nodes in the tree(i.e. from starting node till goal node)
	* 
	* @var int
	*/
	int distance;

	/**
	* heuristic distances between the node and the goal node
	*
	* @var int
	*/
	int heuristic;
	
	/**
	* a reference to parent node
	* the parent of each node can be useful to access the nodes from the goal up to the starting node.
	* 
	* @var Node*
	*/
	struct Node *parent;
	
	/**
	* list of node's children
	*
	* @var list<Node *>
	*/
	list<Node *> children;

};

/**
* The Tree Class.
*/
class Tree {

	/**
	* number of nodes
	*
	* @var int
	*/
	int N;
	
	/**
	* pointer to an array containing tree nodes
	*
	* @var list<Node *>
	*/
	list<Node *> nodes;
	
	/**
	* mark visited nodes
	* it needs to be cleared every time you run an algorithm
	* 
	* @var bool
	*/
	bool *visited;
	
	/**
	* distances between every two connected nodes
	*
	* @var int
	*/
	int **dist_between;

public:
	Tree(int N);
	
	Node * createNode(int value);
	void add_edge(Node *parent, Node *child, int distance);
	void reset();
	
	Tree & print_tree(Node *root);
	Tree & print_path(Node *goal);
	
	int get_distance(Node *parent, Node *child);
	int get_heuristics(Node *current, Node *goal);
	Node * get_min_node();
	
	Node * BFS(Node *root, Node *goal);
	Node * DFS(Node *root, Node *goal);
	Node * DFS_R(Node *current, Node *goal, Node *best);
	Node * DFS_Iterative(Node *root, Node *goal);
	Node * Dijkstra(Node *root, Node *goal);
	Node * Greedy(Node *root, Node *goal);
	Node * Astar(Node *root, Node *goal);
};

/**
* Constructor
*
* @param int N
*/
Tree::Tree(int N) {

	this->N = N;

	// mark all the nodes as not visited
	this->visited = new bool[N];
	for (int i = 0; i < N; i++)
		this->visited[i] = false;

	// assign initial value for distance between every two connected nodes
	this->dist_between = new int *[N];
	for (int i = 0; i < N; ++i) {
		this->dist_between[i] = new int[N];
	}
}

/**
* Creates a new node.
*
* @param int value
* @return Node*
*/
Node * Tree::createNode(int value) {

	Node *node = new Node;
	node->value = value;
	node->distance = 0;
	node->parent = NULL;

	this->nodes.push_back(node);

	return node;
}

/**
* Add an edge to tree.
*
* @param Node* parent
* @param Node* child
* @param int   distance
*/
void Tree::add_edge(Node *parent, Node *child, int distance){

	// Add child to parent's list.
	parent->children.push_back(child);

	// Assign distance
	if (distance != NULL){
		this->dist_between[parent->value][child->value] = 
			this->dist_between[child->value][parent->value] = distance;
	}

}

/**
* Reset the tree to it's origianl state.
*
*/
void Tree::reset(){

	for (int i = 0; i < N; i++)
		this->visited[i] = false;
}

/**
* Print every node and it's children.
*
* @param Node* root
*/
Tree & Tree::print_tree(Node *root){
	
	for (Node *&current : this->nodes){
		printf("Node (%c, h=%d) connected to: ", ('A' + current->value), current->heuristic);
		for (Node *&child : current->children){
			printf("(%c, d=%d)", ('A' + child->value), 
				this->dist_between[child->value][current->value]);
		}
		printf("\n");
	}
	
	// printf("Heuristic distance from %c to %c = %d \n",
	// 	('A' + current->value), ('A' + goal->value), distance);

	// printf("Distance from %c to %c = %d \n",
	//	('A' + parent->value), ('A' + child->value), random);

	return *this;
}

/**
* Print the path from the goal node up to the root node
*
* @param Node* current
*/
Tree & Tree::print_path(Node *goal){

	printf("The path from goal node to root node: ");

	Node *current = goal;

	// compute cost for every algorithm
	int cost = 0;

	while (current != NULL){
		printf("%c ", ('A' + current->value));

		if (current->parent != NULL)
			cost += this->get_distance(current, current->parent);
		
		current = current->parent;
	}

	printf("\t and cost: %d ", cost);
	printf("\n");

	return *this;
}

/**
* Get distance between two nodes.
*
* @param Node* current
* @param Node* child
* @return int
*/
int Tree::get_distance(Node *parent, Node *child){

	// check if distance already assigned
	int distance = 0;
	if (parent->value == child->value)	return distance;
	
	distance = this->dist_between[parent->value][child->value];
	if (distance != NULL)	return distance;

	// if not, generate a random distance between parent and child node
	int random = (int)rand() % 10 + 1;
	this->dist_between[parent->value][child->value] =
		this->dist_between[child->value][parent->value] = random;

	return random;
}

/**
* Get the heuristic distance between a node and the goal.
*
* @param Node* current
* @param Node* goal
* @return int
*/
int Tree::get_heuristics(Node *current, Node *goal){

	// check if heuristic distance already assigned
	int distance = 0;
	if (current->value == goal->value) return distance;

	distance = current->heuristic;
	if (distance == NULL){
		distance = abs(goal->value - current->value);
	}

	return distance;
}

/**
* Get the node with minimum distance.
*
* @return Node*
*/
Node * Tree::get_min_node(){

	Node * next = NULL;
	int mn = INT_MAX;

	for (Node *&node : this->nodes){
		if (!this->visited[node->value] && node->distance < mn){
			mn = node->distance, next = node;
		}
	}

	return next;
}
///////////////////////////////////////////////////////////////////
// Everything Between these lines are the A* code that I made based on various references and research,
// while also staying in the boundaries that this repo started with.
// Every piece of code outside is from the github repo given in the references section, along with
// lots of references to said code inside here.

Node * Tree::Astar(Node *start, Node *finish){

	for (Node *&loop : this->nodes){
		loop->distance = INT_MAX; //Initializes the distances to be the max values for the purposes of the queue
	}

	start->distance = 0 + get_heuristics(start, finish);
	Node* active = this->get_min_node(); //Creates earliest node to allow looping over tree

	while (active != NULL){

		this->visited[active->value] = true;
		if (active->value == finish->value) //You have reached the end and there is nothing more to do
			return active;

		for (Node *&child : active->children){
			
			int dijk = this->get_distance(active, child); //Gets the actual value that Dijkstra would use
			int heur = get_heuristics(child, finish); //Gets the heuristic value pre programmed in
			int aStar = (active->distance - get_heuristics(active, finish)) + dijk + heur; //Computes value based on Dijk and the Heuristic

			if (aStar < child->distance){ //Path was rediscovered as a shorter distance
				child->distance = aStar;
				child->parent = active;
			}
		}	
		active = this->get_min_node();
	}

	return NULL;	
}
//End of my altered form of A*
///////////////////////////////////////////////////////////////////////////

int main() {

	// initialize random seed
	srand((unsigned int) time(NULL));

	// initialize a new tree instance
	const int num_nodes = 7;
	Tree tree(num_nodes);

	// create nodes
	Node * nodes[num_nodes];
	for (int i = 0; i < num_nodes; i++){
		nodes[i] = tree.createNode(i);
	}

	// create edges
	vector< vector <int > > node_edges = {
			{ 1, 2 }, { 0, 3, 4 }, { 0, 3, 5 }, { 1, 2, 4 }, { 1, 3, 6 }, { 2, 6 }, { 4, 5 }
	};

	// distances between nodes
	vector< vector <int > > nodes_distances = {
			{ 4, 1 }, { NULL, 3, 8 }, { NULL, 2, 6 }, { NULL, NULL, 4 }, { NULL, NULL, 2 }, { NULL, 8 }, { NULL, NULL }
	};

	// heuristic distance to the goal
	vector< int > nodes_heuristics = { 8, 8, 6, 5, 1, 4, 0 };

	for (int i = 0; i < num_nodes; i++){
		nodes[i]->heuristic = nodes_heuristics[i];
		for (int j = 0; j < (int)node_edges[i].size(); j++){
			tree.add_edge(nodes[i], nodes[node_edges[i][j]], nodes_distances[i][j]);
		}
	}



	// define root and goal nodes
	Node * A = nodes[0], * G = nodes[6];

	// print the tree
	tree.print_tree(A);

	// run each algorithm & print the path from the goal up to the root node
	// dont' forget to reset the tree after running each algorithm
	tree.print_path(tree.Astar(A, G)).reset();

	return 0;
}
