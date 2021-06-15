#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"


/*
* The RoutePlanner is written as a graph based search algorithm:
*   If you want to do path finding, you can either do it in a 
*   matrix based fashion or with a graph. 
*   The assumption of a connected graph was implicitly made
*/

class RoutePlanner {
  public:
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);
    // Add public variables or methods declarations here.
    float GetDistance() const {return distance;} // returns the complete distance of the path after running the A* algorithm
    void AStarSearch();

    // The following methods have been made public so we can test them individually.
      // methods to conduct the A*-search
    void AddNeighbors(RouteModel::Node *current_node);
    float CalculateHValue(RouteModel::Node const *node);
    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *);
    RouteModel::Node *NextNode();

  private:
    // Add private variables or methods declarations here.
    std::vector<RouteModel::Node*> open_list;
    RouteModel::Node *start_node;
    RouteModel::Node *end_node;

    float distance = 0.0f;
    RouteModel &m_Model; // here, the parsed osm data is stored 
};

#endif