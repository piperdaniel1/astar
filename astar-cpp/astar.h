#include <vector>

enum PointType {
    Obstacle,
    Start,
    Goal,
};

// Represents a point in space
struct Point {
    PointType type;
    int x;
    int y;
    int z;
};

// Represents a point in the AStar grid
struct AStarPoint {
    int x;
    int y;
    int z;
    int prev_cost;
    int est_cost;
    AStarPoint* parent;
};

class AStar {
    public:
    AStar();
    ~AStar();
    
    private:
    // Occupancy grid for obstacles
    std::vector<std::vector<bool>> grid;

    // The start and goal points
    AStarPoint* start;
    AStarPoint* goal;
    // The open and closed lists
    std::vector<AStarPoint*> open;
    std::vector<AStarPoint*> closed;

    // has side effect of moving the lowest open into
    // the closed list
    AStarPoint* pull_lowest_open();

    void push_open(AStarPoint* point);
};