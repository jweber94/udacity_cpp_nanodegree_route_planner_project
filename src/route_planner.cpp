#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
        // start_x, start_y, end_x and end_y are in the range 0 - 100 and are given by the user --> convert to percentage in order to use them in the FindClosestNode method
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    this->start_node = &this->m_Model.FindClosestNode(start_x, start_y);  
    this->end_node = &this->m_Model.FindClosestNode(end_x, end_y); 
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // calculation of the h-value --> heuristic cost (h <= real cost from the current node to the aim-node to be a valid heuristic)
    return node->distance(*this->end_node); 
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    // get a vector of all neighboring nodes
    current_node->FindNeighbors(); 
    this->start_node->visited = true; 

    for (auto node : current_node->neighbors)
    {
        if (node->visited){
            continue; // do not add nodes that are already visited add to the open list
        }

        // update the node attributes
        node->parent = current_node;
        node->h_value = this->CalculateHValue(node); 
        node->g_value = current_node->g_value + current_node->distance(*node);
        node->visited = true; 

        // add to the open list
        this->open_list.push_back(node); 
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

bool NodeComparison (const RouteModel::Node * node_1, const RouteModel::Node * node_2){
    // comparison function for the vector of custom node classes
    float f_node_1 = 0.f; 
    float f_node_2 = 0.f;

    f_node_1 = node_1->g_value + node_1->h_value;
    f_node_2 = node_2->g_value + node_2->h_value;

    if (f_node_1 > f_node_2) // TODO: Check if this must be a < sign
    {
        return true; 
    }
    else{
        return false; 
    }
}

RouteModel::Node *RoutePlanner::NextNode() {
    // sort the open list
    std::sort(this->open_list.begin(), this->open_list.end(), NodeComparison); 

    // remove best node from the open list and return its pointer
    RouteModel::Node * best_node = this->open_list.back(); 
    this->open_list.pop_back(); 
    return best_node; 
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // traversing the path back from the final node to the start node
    
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    bool start_node_reached = false;
    RouteModel::Node * tmp_node_old = current_node; 
    RouteModel::Node * tmp_node_new = nullptr;
    float tmp_distance = 0.f;

    path_found.push_back(*tmp_node_old);   
    
    while (tmp_node_old->parent != nullptr) // loop until the start node is reached
    {
        tmp_node_new = tmp_node_old->parent; 
        tmp_distance = tmp_node_old->distance(*tmp_node_new); // dereferencing the tmp_node to fit the method definition
        distance += tmp_distance; 
        path_found.push_back(*tmp_node_new); // dereferencing --> deep copy of the node information 

        // set the new to old to get the next parent node in the next loop iteration
        tmp_node_old = tmp_node_new; 
    }

    // reverse the vector since we add the end node element as the first entry in the path_found vector
    std::reverse(path_found.begin(), path_found.end()); 

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    bool end_node_reached = false; 

    // start the first step manually before entering the while loop
    current_node = this->start_node;
    this->AddNeighbors(current_node);  
    

    while ((!this->open_list.empty())) // loop until the open list is not empty or the end node is added to the closed list
    {
        current_node = this->NextNode();    // choose the next node to expand in a greedy manner
        this->AddNeighbors(current_node);   // adding next neighbors to the open list

        // check if we expanded the end node
        if (current_node->distance(*this->end_node) == 0)
        {
            std::cout << "added end node to the closed list\n"; 
            end_node_reached = true; 
        }
    }
    
    // traverse back the resulting path
    this->m_Model.path = this->ConstructFinalPath(this->end_node); 
}