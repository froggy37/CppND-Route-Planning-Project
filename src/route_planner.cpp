#include "route_planner.h"
#include <algorithm>

using std::sort;
using std::vector;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    this -> start_node = &m_Model.FindClosestNode(start_x, start_y);
    this -> end_node = &m_Model.FindClosestNode(end_x, end_y);
};


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*this->end_node);
};


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto *neighbor : current_node->neighbors){
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value; + current_node->distance(*neighbor);
        this->open_list.push_back(neighbor);
        neighbor->visited = true;
    }
};


bool Compare (const RouteModel::Node * a, const RouteModel::Node * b){
    auto fA =  a->g_value + a->h_value;
    auto fB = b->g_value + b->h_value;
    return fA > fB;    
}

RouteModel::Node *RoutePlanner::NextNode() {
    sort(this->open_list.begin(), this->open_list.end(), Compare);
    auto next_node = this->open_list.back();
    this->open_list.pop_back();
    return next_node;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    
    // TODO: Implement your solution here.
    auto current = current_node;
    while(true){
        if(current == this->start_node){
            path_found.push_back(*current);
            break;
        };
        auto parent = current->parent;
        distance += current_node->distance(*parent);
        path_found.push_back(*current);
        current = parent;
    }
    std::reverse(path_found.begin(),path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    this->start_node->visited = true;
    this->AddNeighbors(this->start_node);

    while(this->open_list.size() >0){
        auto next_node = this->NextNode();
        if(next_node->x == this->end_node->x && next_node->y == this->end_node->y){
            m_Model.path = this->ConstructFinalPath(next_node);
            return;
        };
        this->AddNeighbors(next_node);
    };
}