#include "astar.h"
#include <iostream>
#include <algorithm>
#include <cmath>

using namespace std;

AStar::AStar() {
    // Initialize the grid
    grid = vector<vector<bool>>(20, vector<bool>(20, false));

    // Set start and goal to null
    start = nullptr;
    goal = nullptr;
    
    // Initialize the open and closed lists
    open = vector<AStarPoint*>();
    closed = vector<AStarPoint*>();
}

AStar::~AStar() {
    // Delete all the points in the open and closed lists
    for (auto point : open) {
        if (point != nullptr) {
            delete point;
        }
    }
    for (auto point : closed) {
        if (point != nullptr) {
            delete point;
        }
    }

    // Delete the start and goal points
    if (start != nullptr) {
        delete start;
    }

    if (goal != nullptr) {
        delete goal;
    }
}

AStarPoint* AStar::pull_lowest_open() {
    // open is sorted by est_cost, so the last element is the lowest
    AStarPoint* lowest = open.back();
    open.pop_back();

    closed.push_back(lowest);
    return lowest;
}

void AStar::push_open(AStarPoint* point) {
    // Check if the point is already in the open list
    for (auto open_point : open) {
        if (open_point->x == point->x && open_point->y == point->y && open_point->z == point->z) {
            // If the point is already in the open list, check if the new point has a lower cost
            if (point->prev_cost < open_point->prev_cost) {
                // If the new point has a lower cost, replace the old point with the new point
                open_point->prev_cost = point->prev_cost;
                open_point->est_cost = point->est_cost;
                open_point->parent = point->parent;
            }

            // Sort the open list by est_cost
            sort(open.begin(), open.end(), [](AStarPoint* a, AStarPoint* b) {
                return a->est_cost > b->est_cost;
            });

            // Delete the new point
            delete point;
            return;
        }
    }

    // Check if the point is already in the closed list
    for (auto closed_point : closed) {
        if (closed_point->x == point->x && closed_point->y == point->y && closed_point->z == point->z) {
            // If the point is already in the closed list, check if the new point has a lower cost
            if (point->prev_cost < closed_point->prev_cost) {
                // If the new point has a lower cost, replace the old point with the new point
                closed_point->prev_cost = point->prev_cost;
                closed_point->est_cost = point->est_cost;
                closed_point->parent = point->parent;
                // Move the point from the closed list to the open list
                closed.erase(remove(closed.begin(), closed.end(), closed_point), closed.end());
                open.push_back(closed_point);
                // Sort the open list by est_cost
                sort(open.begin(), open.end(), [](AStarPoint* a, AStarPoint* b) {
                    return a->est_cost > b->est_cost;
                });
            }
            // Delete the new point
            delete point;
            return;
        }
    }

    // If the point is not in the open or closed list, add it to the open list
    open.push_back(point);

    // Sort the open list by est_cost in descending order
    sort(open.begin(), open.end(), [](AStarPoint* a, AStarPoint* b) {
        return a->est_cost > b->est_cost;
    });    
}
