#include "astar.h"

using namespace std;

Point::Point() {
    this->x = 0;
    this->y = 0;
    this->z = 0;
    this->type = PointType::Obstacle;
}

Point::Point(int x, int y, int z) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->type = PointType::Obstacle;
}

// In this constructor, we create a cube of size num_cells x num_cells x num_cells for the occupancy grid
// It may be more efficient to use a non uniform grid, but this will work for now
AStar::AStar(Point tl_corner, Point bl_corner, int num_cells, Point start, Point goal, std::vector<Point> obstacles) {
    this->tl_point = new Point(tl_corner.x, tl_corner.y, tl_corner.z);
    this->br_point = new Point(bl_corner.x, bl_corner.y, bl_corner.z);
    this->grid_size = num_cells;

    // First we need to create the occupancy grid
    double x_size = abs(tl_corner.x - bl_corner.x);
    x_size = x_size / num_cells;

    double y_size = abs(tl_corner.y - bl_corner.y);
    y_size = y_size / num_cells;

    double z_size = abs(tl_corner.z - bl_corner.z);
    z_size = z_size / num_cells;

    grid_cell_len_x = x_size;
    grid_cell_len_y = y_size;
    grid_cell_len_z = z_size;

    // Encode the start and goal points into the AStarPoint format
    // (basically we just need to convert the x y and z coordinates to cell indices)
    this->start = nullptr;
    this->goal = nullptr;
    this->start = conv_point(&start, nullptr);
    this->goal = conv_point(&goal, nullptr);

    // Reshape this->occ_grid to be a 3D vector of size num_cells x num_cells x num_cells
    this->occ_grid.resize(num_cells, vector<vector<bool>>(num_cells, vector<bool>(num_cells, false)));

    // Alrighty, lets fill er in
    for (auto obstacle : obstacles) {
        // First we need to find the index of the obstacle in the occupancy grid
        int x_index = (obstacle.x - tl_corner.x) / x_size;
        int y_index = (obstacle.y - tl_corner.y) / y_size;
        int z_index = (obstacle.z - tl_corner.z) / z_size;

        // Now we can set the occupancy grid at that index to true
        this->occ_grid[x_index][y_index][z_index] = true;
    }

    // Lets print out the occupancy grid again to make sure it looks right 
    // for (int i = 0; i < num_cells; i++) {
    //     for (int j = 0; j < num_cells; j++) {
    //         for (int k = 0; k < num_cells; k++) {
    //             cout << this->occ_grid[i][j][k] << " ";
    //         }
    //         cout << endl;
    //     }
    //     cout << endl;
    // }
}

AStar::~AStar() {
    // Delete all the points in the open and closed lists
    for (auto point : open) {
        if (point != nullptr) {
            delete point;
            point = nullptr;
        }
    }
    for (auto point : closed) {
        if (point != nullptr) {
            delete point;
            point = nullptr;
        }
    }

    // Delete the start and goal points
    // if (start != nullptr) { // start point already deleted as it is in the closed list
    //     delete start;
    //     start = nullptr;
    // }

    if (goal != nullptr) {
        delete goal;
        goal = nullptr;
    }

    // Delete the top left and bottom right points
    if (tl_point != nullptr) {
        delete tl_point;
        tl_point = nullptr;
    }

    if (br_point != nullptr) {
        delete br_point;
        br_point = nullptr;
    }
}

AStarPoint* AStar::pull_lowest_open() {
    // open is sorted by est_cost, so the last element is the lowest
    AStarPoint* lowest = open.back();
    open.pop_back();

    // closed.push_back(lowest);
    this->insert_close(lowest);

    return lowest;
}

void AStar::push_open(AStarPoint* point) {
    // store time
    // auto list_start = std::chrono::high_resolution_clock::now();
    // Check if the point is already in the open list
    // for (auto open_point : open) {
    //     if (open_point->x == point->x && open_point->y == point->y && open_point->z == point->z) {
    //         // Output both points
    //         // cout << "Point already in open list: " << endl;
    //         // cout << "Point 1: " << open_point->x << ", " << open_point->y << ", " << open_point->z << endl;
    //         // cout << "Point 2: " << point->x << ", " << point->y << ", " << point->z << endl;

    //         // Delete the new point
    //         delete point;
    //         return;
    //     }
    // }

    if (this->in_open(point)) {
        // cout << "Point already in open list: " << endl;
        // cout << "Point 1: " << open_point->x << ", " << open_point->y << ", " << open_point->z << endl;
        // cout << "Point 2: " << point->x << ", " << point->y << ", " << point->z << endl;

        // Delete the new point
        delete point;
        return;
    }

    // Check if the point is already in the closed list
    // for (auto closed_point : closed) {
    //     if (closed_point->x == point->x && closed_point->y == point->y && closed_point->z == point->z) {
    //         // Output both points
    //         // cout << "Point already in closed list: " << endl;
    //         // cout << "Point 1: " << closed_point->x << ", " << closed_point->y << ", " << closed_point->z << endl;
    //         // cout << "Point 2: " << point->x << ", " << point->y << ", " << point->z << endl;

    //         // Delete the new point
    //         delete point;
    //         return;
    //     }
    // }

    if (this->in_closed(point)) {
        // cout << "Point already in closed list: " << endl;
        // cout << "Point 1: " << closed_point->x << ", " << closed_point->y << ", " << closed_point->z << endl;
        // cout << "Point 2: " << point->x << ", " << point->y << ", " << point->z << endl;

        // Delete the new point
        delete point;
        return;
    }

    // auto list_end = std::chrono::high_resolution_clock::now();

    // print time
    // std::chrono::duration<double> list_time = list_end - list_start;
    // cout << "List time: " << list_time.count() << endl;
    

    // open.push_back(point);

    this->insert_open(point);

    // auto sort_start = std::chrono::high_resolution_clock::now();
    // // Sort the open list by est_cost in descending order
    // std::sort(open.begin(), open.end(), [](AStarPoint* a, AStarPoint* b) {
    //     return (a->est_cost+a->prev_cost) > (b->est_cost+b->prev_cost);
    // });
    // auto sort_end = std::chrono::high_resolution_clock::now();

    // print time
    // std::chrono::duration<double> sort_time = sort_end - sort_start;
    // cout << "Sort time: " << sort_time.count() << endl;
}

std::vector<AStarPoint*> AStar::get_neighbors(AStarPoint* point) {
    std::vector<AStarPoint*> neighbors;

    // There will be at most 26 neighbors for each point
    // There may be less if the point is on the edge of the grid
    neighbors.reserve(26);

    // Loop through all the possible neighbors
    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            for (int k = -1; k <= 1; k++) {
                // Make sure the neighbor is not the point itself
                if (i == 0 && j == 0 && k == 0) {
                    continue;
                }

                // Make sure the neighbor is within the grid
                if (point->x + i < 0 || point->x + i >= this->grid_size) {
                    // std::cout << "Neighbor out of bounds: " << point->x + i << ", " << point->y + j << ", " << point->z + k << std::endl;
                    continue;
                }
                if (point->y + j < 0 || point->y + j >= this->grid_size) {
                    // std::cout << "Neighbor out of bounds: " << point->x + i << ", " << point->y + j << ", " << point->z + k << std::endl;
                    continue;
                }
                if (point->z + k < 0 || point->z + k >= this->grid_size) {
                    // std::cout << "Neighbor out of bounds: " << point->x + i << ", " << point->y + j << ", " << point->z + k << std::endl;
                    continue;
                }

                // Make sure the neighbor is not an obstacle
                if (this->occ_grid[point->x + i][point->y + j][point->z + k]) {
                    // std::cout << "Neighbor is an obstacle: " << point->x + i << ", " << point->y + j << ", " << point->z + k << std::endl;
                    continue;
                }

                // Add the neighbor to the list
                AStarPoint* new_point = new AStarPoint();
                new_point->x = point->x + i;
                new_point->y = point->y + j;
                new_point->z = point->z + k;
                new_point->parent = point;
                new_point->prev_cost = point->prev_cost + dist(point, new_point);
                new_point->est_cost = dist(new_point, this->goal);

                neighbors.push_back(new_point);
            }
        }
    }

    return neighbors;
}

std::vector<Point*> AStar::run() {
    // Use the cache if we have already run the algorithm before
    if (this->path.size() > 0) {
        return this->path;
    }

    this->open.clear();
    this->closed.clear();
    this->path.clear();

    // Add the start point to the open list
    // std::cout << "Start point: " << *this->start << std::endl;
    this->push_open(this->start);

    // std::cout << "Push open [0] is " << this->open[0]->x << ", " << this->open[0]->y << ", " << this->open[0]->z << std::endl;

    // auto main_start_time = std::chrono::system_clock::now();
    // init elapsed push time to zero
    // std::chrono::duration<double> elapsed_push_time = std::chrono::duration<double>::zero();
    // While the open list is not empty
    while (this->open.size() > 0) {
        // Get the lowest cost point from the open list
        AStarPoint* current = this->pull_lowest_open();

        // std::cout << "\nDistance heuristic: " << this->dist(conv_astar_point(current), goal) << std::endl;
        // std::cout << "Current point: " << *current << " (converted: " << *conv_astar_point(current) << ")" << std::endl;

        // If the current point is the goal point, we are done
        if (current->x == this->goal->x && current->y == this->goal->y && current->z == this->goal->z) {
            // We are done, lets build the path
            AStarPoint* path_point = current;
            while (path_point != nullptr) {
                this->path.push_back(conv_astar_point(path_point));
                path_point = path_point->parent;
            }

            // Reverse the path so that it starts at the start point
            reverse(this->path.begin(), this->path.end());

            // Report time info
            // auto main_end_time = std::chrono::system_clock::now();
            // std::chrono::duration<double> elapsed_main_time = main_end_time - main_start_time;
            // std::cout << "elapsed main time: " << elapsed_main_time.count() << "s" << std::endl;
            // std::cout << "elapsed push time: " << elapsed_push_time.count() << "s" << std::endl;

            // Return the path
            return this->path;
        }

        // Get the neighbors of the current point
        vector<AStarPoint*> neighbors = this->get_neighbors(current);

        // std::cout << "Neighbors: " << neighbors.size() << std::endl;

        // For each neighbor
        // auto push_start_time = std::chrono::system_clock::now();
        for (auto neighbor : neighbors) {
            // Add the new point to the open list
            // Automatically performs the necessary checks to make sure we
            // don't add the same point twice or add a point that is in the closed list
            // std::cout << "Neighbor: " << neighbor->x << ", " << neighbor->y << ", " << neighbor->z << std::endl;
            this->push_open(neighbor);
        }
        // add time to elapsed push time
        // elapsed_push_time += std::chrono::system_clock::now() - push_start_time;

        // std::cout << "Open: " << this->open.size() << std::endl;
    }


    // If we get here, there is no path
    return this->path;
}

AStarPoint* AStar::conv_point(Point* point, AStarPoint* parent) {
    AStarPoint* astar_point = new AStarPoint();
    astar_point->x = (point->x - tl_point->x) / this->grid_cell_len_x;
    astar_point->y = (point->y - tl_point->y) / this->grid_cell_len_y;
    astar_point->z = (point->z - tl_point->z) / this->grid_cell_len_z;
    astar_point->parent = parent;
    if (parent == nullptr) {
        astar_point->prev_cost = 0;
    } else {
        astar_point->prev_cost = parent->prev_cost;
    }

    // BIG TODO
    // THIS IS NOT CORRECT IF THE GRID IS NOT A CUBIC SHAPE IN SPACE
    // Add this->gride size for straight moves and sqrt(2 * this->grid_size) for diagonal moves and sqrt(3 * this->grid_size) for 3D diagonal moves
    // Everytime I write something like this I think that I should return my CS degree
    if (parent != nullptr) {
        if ((parent->x == point->x && parent->y == point->y) || (parent->x == point->x && parent->z == point->z) || (parent->y == point->y && parent->z == point->z)) {
            astar_point->prev_cost += this->grid_cell_len_x;
        } else if ((parent->x == point->x && parent->y != point->y && parent->z != point->z)
                || (parent->x != point->x && parent->y == point->y && parent->z != point->z)
                || (parent->x != point->x && parent->y != point->y && parent->z == point->z)) {
            astar_point->prev_cost += 2 * this->grid_cell_len_x;
            // astar_point->prev_cost += sqrt(2 * pow(this->grid_cell_len_x, 2));
        } else {
            astar_point->prev_cost += 3 * this->grid_cell_len_x;
            // astar_point->prev_cost += sqrt(3 * pow(this->grid_cell_len_x, 2));
        }
    }

    // This only happens when we are initializing converting the start point and
    // goal point to AStarPoints. We don't care about the estimated cost of the
    // start/goal points, so we just set it to 0
    if (this->goal == nullptr) {
        astar_point->est_cost = 0;
    } else {
        astar_point->est_cost = dist(astar_point, this->goal);
    }

    return astar_point;
}


Point* AStar::conv_offset_astar_point(AStarPoint* point, int x_off, int y_off, int z_off) {
    Point* conv_point = new Point();
    conv_point->x = double(point->x + x_off) * this->grid_cell_len_x + tl_point->x ;
    conv_point->y = double(point->y + y_off) * this->grid_cell_len_y + tl_point->y ;
    conv_point->z = double(point->z + z_off) * this->grid_cell_len_z + tl_point->z ;

    return conv_point;
}

Point* AStar::conv_astar_point(AStarPoint* point) {
    Point* conv_point = new Point();
    conv_point->x = double(point->x) * this->grid_cell_len_x + tl_point->x;
    conv_point->y = double(point->y) * this->grid_cell_len_y + tl_point->y;
    conv_point->z = double(point->z) * this->grid_cell_len_z + tl_point->z;

    return conv_point;
}

double AStar::dist(AStarPoint* a, AStarPoint* b) {
    return sqrt(pow(a->x - b->x, 2) + pow(a->y - b->y, 2) + pow(a->z - b->z, 2));
    // return abs(a->x - b->x) + abs(a->y - b->y) + abs(a->z - b->z);
}

double AStar::dist(double x1, double y1, double z1, double x2, double y2, double z2) {
    return abs(x1 - x2) + abs(y1 - y2) + abs(z1 - z2);
}

int AStar::insert_open(AStarPoint* point) {
    // If the open list is empty, just add the point
    if (this->open.size() == 0) {
        this->open.push_back(point);
        return 0;
    }

    // If the open list is not empty, find the correct place to insert the point
    // The open list is sorted by the sum of the estimated cost and the previous cost
    // The estimated cost is the distance from the current point to the goal
    // The previous cost is the distance from the start to the current point
    // The lower the sum, the closer the point is to the goal

    // We start in the middle of the list and check if the point is closer to the
    // goal than the point in the middle of the list. If it is, we check the
    // points to the left of the middle. If it is not, we check the points to the
    // right of the middle. We repeat this process until we find the correct
    // place to insert the point
    int left = 0;
    int right = this->open.size() - 1;
    int middle = (left + right) / 2;
    while (left <= right) {
        middle = (left + right) / 2;
        if (point->est_cost + point->prev_cost > this->open[middle]->est_cost + this->open[middle]->prev_cost) {
            right = middle - 1;
        } else {
            left = middle + 1;
        }
    }

    // If we get here, we have found the correct place to insert the point
    // We insert the point at the left index
    this->open.insert(this->open.begin() + left, point);
    return left;
}

int AStar::insert_close(AStarPoint* point) {
    // If the closed list is empty, just add the point
    if (this->closed.size() == 0) {
        this->closed.push_back(point);
        return 0;
    }

    // If the closed list is not empty, find the correct place to insert the point
    // The closed list is sorted by the sum of the estimated cost and the previous cost
    // The estimated cost is the distance from the current point to the goal
    // The previous cost is the distance from the start to the current point
    // The lower the sum, the closer the point is to the goal

    // We start in the middle of the list and check if the point is closer to the
    // goal than the point in the middle of the list. If it is, we check the
    // points to the left of the middle. If it is not, we check the points to the
    // right of the middle. We repeat this process until we find the correct
    // place to insert the point
    int left = 0;
    int right = this->closed.size() - 1;
    int middle = (left + right) / 2;
    while (left <= right) {
        middle = (left + right) / 2;
        if (point->est_cost + point->prev_cost > this->closed[middle]->est_cost + this->closed[middle]->prev_cost) {
            right = middle - 1;
        } else {
            left = middle + 1;
        }
    }

    // If we get here, we have found the correct place to insert the point
    // We insert the point at the left index
    this->closed.insert(this->closed.begin() + left, point);
    return left;
}

bool AStar::in_open(AStarPoint* point) {
    // If the open list is empty return false
    if (this->open.size() == 0) {
        return false;
    }

    int left = 0;
    int right = this->open.size() - 1;
    int middle = (left + right) / 2;
    while (left <= right) {
        middle = (left + right) / 2;
        if (point->est_cost + point->prev_cost > this->open[middle]->est_cost + this->open[middle]->prev_cost) {
            right = middle - 1;
        } else {
            left = middle + 1;
        }
    }

    if (left >= this->open.size()) {
        left = this->open.size() - 1;
    }

    // Check if we found the point
    if (this->open[left]->x == point->x && this->open[left]->y == point->y && this->open[left]->z == point->z) {
        return true;
    }

    // If we get here, we did not find the point
    return false;
}

bool AStar::in_closed(AStarPoint* point) {
    // If the closed list is empty return false
    if (this->closed.size() == 0) {
        return false;
    }

    int left = 0;
    int right = this->closed.size() - 1;
    int middle = (left + right) / 2;
    while (left <= right) {
        middle = (left + right) / 2;
        if (point->est_cost + point->prev_cost > this->closed[middle]->est_cost + this->closed[middle]->prev_cost) {
            right = middle - 1;
        } else {
            left = middle + 1;
        }
    }

    if (left >= this->closed.size()) {
        left = this->closed.size() - 1;
    }

    // Check if we found the point
    if (this->closed[left]->x == point->x && this->closed[left]->y == point->y && this->closed[left]->z == point->z) {
        return true;
    }

    // If we get here, we did not find the point
    return false;
}