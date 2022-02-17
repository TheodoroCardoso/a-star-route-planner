#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// Implement the CalculateHValue method.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    node->distance(*end_node);
}


// Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto node : current_node->neighbors){
        node->parent = current_node;
        node->g_value = current_node->g_value + current_node->distance(*node);
        node->h_value = CalculateHValue(node);
        open_list.push_back(node);
        node->visited = true;
    }    
}


// Complete the NextNode method to sort the open list and return the next node.
bool Compare(const RouteModel::Node *node1, const RouteModel::Node *node2) {
  float f1 = node1->g_value + node1->h_value; // f1 = g1 + h1
  float f2 = node2->g_value + node2->h_value; // f2 = g2 + h2
  return f1 > f2; 
}

RouteModel::Node *RoutePlanner::NextNode() {
    sort(open_list.begin(), open_list.end(), Compare);
    auto next = open_list.back();
    open_list.pop_back();
    return next;
}


// Complete the ConstructFinalPath method to return the final path found from your A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    // TODO: Implement your solution here.
    distance = current_node->g_value;
    while (current_node != start_node) {
        path_found.push_back(*current_node);
        current_node = current_node->parent;
    }
    path_found.push_back(*current_node);
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// A* Search algorithm
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    start_node->visited = true;
    open_list.push_back(start_node);

    while (open_list.size() > 0){        
        current_node = NextNode();        
        if (current_node == end_node){
            m_Model.path = ConstructFinalPath(current_node);
            break;
        }
        else AddNeighbors(current_node);        
    }
}