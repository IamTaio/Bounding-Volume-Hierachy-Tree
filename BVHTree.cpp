#include "BVHTree.h"

void clear(BVHTreeNode* Root);
void deletebranch(BVHTreeNode* node);
void updateRemoval(BVHTreeNode* node);
void updateParents(BVHTreeNode* node);
BVHTreeNode* FindTBD(BVHTreeNode* node);
BVHTreeNode* Findexisting(BVHTreeNode* node, BVHTreeNode* newNode);
std::vector<std::string> getCollisions(AABB object, std::vector<std::string>& collisions, BVHTreeNode* branch);
bool iscovered(BVHTreeNode* node, BVHTreeNode* branch);

/***************************************************
Default constructor- Creates an empty tree with the 
root pointing to NULL
***************************************************/
BVHTree::BVHTree() {
	root = NULL;
}

/*******************************************************
Destructor- Deletes all dynamically allocated variables
*******************************************************/
BVHTree::~BVHTree() {
	clear(root);
	root = NULL;
}

/********************************
Deletes all elements of the tree
********************************/
void clear(BVHTreeNode* Root) {
	while (Root != NULL) {
#ifdef DEBUG
		BVHTreeNode* tbd = FindTBD(Root);
#endif // DEBUG
#ifndef DEBUG
		BVHTreeNode* tbd = NULL;
		tbd = FindTBD(Root);
#endif // !DEBUG

		if (tbd == Root) {
			Root = NULL;
			if (tbd != NULL) {
				delete tbd;
				tbd = NULL;
			}
		}
		else if (tbd->isLeaf) {
			BVHTreeNode* parent = tbd->parent;
			if (parent->leftChild == tbd) {
				parent->leftChild = NULL;
				if (tbd != NULL) {
					delete tbd;
					tbd = NULL;
				};
			}
			else {
				parent->rightChild = NULL;
				if (tbd != NULL) {
					delete tbd;
					tbd = NULL;
				}
			}
		}

		else {
			deletebranch(tbd);
		}
	}
}


/*************************************************************************
Given an existing node of a tree and a new node, desired to be inserted
finds an existing leaf at which position newNode can be inserted
with minimal increase in area occupied by the tree
************************************************************************/
BVHTreeNode* Findexisting(BVHTreeNode* node, BVHTreeNode* newNode) {
	if (node->isLeaf){
		return node;
	}
	int increaseInRightTreeSize = AABB::unionArea(newNode->aabb,
		node->rightChild->aabb) - node->rightChild->aabb.getArea();
	int increaseInLeftTreeSize = AABB::unionArea(newNode->aabb,
		node->leftChild->aabb) - node->leftChild->aabb.getArea();
	if (increaseInRightTreeSize < increaseInLeftTreeSize)
	{
		if (node->rightChild->isLeaf) {
			return node->rightChild;
		}
		else {
			Findexisting(node->rightChild, newNode);
		}
	}
	else {
		if (node->leftChild->isLeaf) {
			return node->leftChild;
		}
		else {
			Findexisting(node->leftChild, newNode);
		}
	}
}


/************************************************************************
Given a node, finds the last branch in tree with both leaves as children
*************************************************************************/
BVHTreeNode* FindTBD(BVHTreeNode* node) {
	if ((node->leftChild == NULL || node->leftChild->isLeaf) && (node->rightChild == NULL || node->rightChild->isLeaf)) {
		return node;
	}
	if (node->leftChild != NULL && (!node->leftChild->isLeaf)) {
		FindTBD(node->leftChild);
	}
	else if (node->rightChild != NULL && (!node->rightChild->isLeaf)) {
		FindTBD(node->rightChild);
	}
}

/***************************************************
* Given a branch node with both children as leaves,
* deallocates both children and branch node
***************************************************/
void deletebranch(BVHTreeNode* node) {
	if (node->leftChild != NULL)
	{
		BVHTreeNode* temp = node->leftChild;
		delete temp;
		node->leftChild = NULL;
	}
	else if (node->rightChild != NULL)
	{
		BVHTreeNode* temp = node->rightChild;
		delete temp;
		node->rightChild = NULL;
	}
	if (node->parent == NULL) {
		BVHTreeNode* temp = node;
		delete node;
		node = NULL;
	}
	else {
		BVHTreeNode* parent = node->parent;
		if (parent->leftChild == node) {
			parent->leftChild = NULL;
			if (node != NULL) {
				delete node;
				node = NULL;
			};
		}
		else {
			parent->rightChild = NULL;
			if (node != NULL) {
				delete node;
				node = NULL;
			}
		}
	}
	
}


//update aabb of all ancestors of a given node
void updateParents(BVHTreeNode* node) {
	BVHTreeNode* temp = node;
	while (temp->parent != NULL) {
		if (!iscovered(temp, temp->parent)) {
			temp->parent->aabb = temp->parent->aabb + temp->aabb;
		}
		temp = temp->parent;
	}
}

//adds a new leaf to an existing tree
void BVHTree::addBVHMember(AABB objectarea, std::string name) {
	if (map.find(name) != map.end()) {
		return;
	}
	BVHTreeNode* Tba = new BVHTreeNode(objectarea, name, true);
	map[name] = Tba;
	if (root == NULL) {
		root = Tba;
		return;
	}
	else if (root != NULL && root->leftChild == NULL) {
		BVHTreeNode* temp = new BVHTreeNode(root->aabb,root->name, true);
		map.erase(root->name);
		map[temp->name] = temp;
		AABB area = root->aabb + objectarea;
		root = new BVHTreeNode(area, "branch", false);
		root->leftChild = Tba;
		root->rightChild = temp;
		Tba->parent = root;
		temp->parent = root;
		return;
	}
	else {
		BVHTreeNode* temp = Findexisting(root, Tba);
		BVHTreeNode* parent = temp->parent;
		std::string sibling = "";
		if (parent->leftChild == temp) {
			sibling = "right";
		}
		else {
			sibling = "left";
		}
		BVHTreeNode* newtemp = new BVHTreeNode(temp->aabb, temp->name, true);
		map.erase(temp->name);
		map[temp->name] = newtemp;
		AABB area = newtemp->aabb + objectarea;
		temp = new BVHTreeNode(area, "branch", false);
		temp->parent = parent;
		if (sibling == "right") {
			parent->leftChild = temp;
		}
		else {
			parent->rightChild = temp;
		}
		temp->leftChild = Tba;
		temp->rightChild = newtemp;
		Tba->parent = temp;
		newtemp->parent = temp;
		updateParents(temp);
		return;
	}
}

//updates all ancestors of a given node upon deletion
void updateRemoval(BVHTreeNode* node) {
	if (node->isLeaf) {
		return;
	}
	else if (node->parent == NULL) {
		AABB adjusted = node->leftChild->aabb + node->rightChild->aabb;
		node->aabb = adjusted;
		return;
	}
	else {
		AABB adjusted = node->leftChild->aabb + node->rightChild->aabb;
		node->aabb = adjusted;
		updateRemoval(node->parent);
	}

}

//removes a leaf from an existing tree
void BVHTree::removeBVHMember(std::string name) {
	if (map.find(name) == map.end()) {
		return;
	}
	BVHTreeNode* Tbr = map[name];
	if (Tbr == root) {
		clear(root);
		return;
	}
	map.erase(name);
	BVHTreeNode* temp = Tbr->parent;
	BVHTreeNode* ancestor = temp->parent;
	if (ancestor == NULL) {
		return;
	}
	if (temp->leftChild == Tbr) {
		if (ancestor->leftChild == temp) {
			ancestor->leftChild = temp->rightChild;
			temp->rightChild->parent = ancestor;
		}
		else {
			ancestor->rightChild = temp->rightChild;
			temp->rightChild->parent = ancestor;
		}
	}
	else {
		if (ancestor->leftChild == temp) {
			ancestor->leftChild = temp->leftChild;
			temp->leftChild->parent = ancestor;
		}
		else {
			ancestor->rightChild = temp->leftChild;
			temp->leftChild->parent = ancestor;
		}
	}
	updateRemoval(ancestor);
	temp->leftChild = NULL;
	temp->rightChild = NULL;
	temp->parent = NULL;
	delete Tbr;
	Tbr = NULL;
	delete temp;
	temp = NULL;
}

bool iscovered(BVHTreeNode* node, BVHTreeNode* branch) {
	if (branch->aabb.minX <= node->aabb.minX && branch->aabb.minY <= node->aabb.minY) {
		if (branch->aabb.maxX >= node->aabb.maxX && branch->aabb.maxY >= node->aabb.maxY) {
			return true;
		}
		else {
			return false;
		}
	}
	else {
		return false;
	}
}

//Moves a leaf within an existing tree from its current location to new given location
void BVHTree::moveBVHMember(std::string name, AABB location) {
	BVHTreeNode* Tbm = map[name];
	BVHTreeNode temp(location, name, false);
	BVHTreeNode* tempptr = &temp;
	if (Tbm->parent == NULL) {
		Tbm->aabb = location;
		return;
	}
	else if (iscovered(tempptr, Tbm->parent)) {
		Tbm->aabb = location;
		return;
	}
	else {
		removeBVHMember(Tbm->name);
		addBVHMember(location, name);
	}
}

//returns a vector of all leafs that collide with an object
std::vector<std::string> getCollisions(AABB object, std::vector<std::string>& collisions, BVHTreeNode* branch) {
	if (branch == NULL) 
	{
		return collisions;
	}
	else if (branch->isLeaf) {
		if (branch->aabb.collide(object)) {
			collisions.push_back(branch->name);
			return collisions;
		}
		return collisions;
	}
	else if (branch->leftChild->isLeaf && branch->rightChild->isLeaf) {
		if (branch->leftChild->aabb.collide(object)) {
			collisions.push_back(branch->leftChild->name);
		}
		if (branch->rightChild->aabb.collide(object)) {
			collisions.push_back(branch->rightChild->name);
		}
		return collisions;
	}
	else if (branch->leftChild->isLeaf && !branch->rightChild->isLeaf) {
		if (branch->leftChild->aabb.collide(object)) {
			collisions.push_back(branch->leftChild->name);
		}
		collisions = getCollisions(object, collisions, branch->rightChild);
		return collisions;
	}
	else if (branch->rightChild->isLeaf && !branch->leftChild->isLeaf) {
		if (branch->rightChild->aabb.collide(object)) {
			collisions.push_back(branch->rightChild->name);
		}
		collisions = getCollisions(object, collisions, branch->leftChild);
		return collisions;
	}
	else {
		collisions = getCollisions(object, collisions, branch->leftChild);
		collisions = getCollisions(object, collisions, branch->rightChild);
		return collisions;
	}
}

//returns a vector of all collided elements within the tree
std::vector<std::string> BVHTree::getCollidingObjects(AABB object) {
	std::vector<std::string> collisions;
	getCollisions(object, collisions, root);
	return collisions;
}


void BVHTree::printNode(std::ostream &out, BVHTreeNode *node, int level) {
	if (root == nullptr) return;
	for (int i = 0; i < level; i++) {
		out << "  ";
	}
	if (!node->isLeaf) {
		out << "+ branch || ";
		node->aabb.printAABB(out);
		out << std::endl;
		printNode(out, node->rightChild, level + 1);
		printNode(out, node->leftChild, level + 1);
	}
	else {
		out << "- ";
		if (node->parent) {
			if (node->parent->rightChild == node)
				out << "R ";
			else
				out << "L ";
		}
		out << "- leaf: " << node->name << " || ";
		node->aabb.printAABB(out);
		out << std::endl;
	}
}
std::ostream &operator<<(std::ostream &out, BVHTree &tree) {
	tree.printNode(out, tree.root, 0);
	return out;
}