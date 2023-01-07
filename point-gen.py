import sys
from points import *

def gen_new_cloud(num_obstacles, total_points) -> PointCloud:
    cloud = PointCloud([])
    obstacles = ObstacleSpace([], Point(-10, -10, -10), Point(10, 10, 10))

    for _ in range(num_obstacles):
        center = Point(random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(-10, 10))
        radius = random.uniform(0.5, 2)
        obstacles.add_obstacle(Sphere(center, radius))
    
    for _ in range(total_points):
        cloud.add_point(obstacles.sample_colliding_point())
    
    start = obstacles.sample_safe_point()
    start._type = 'start'
    cloud.add_point(start)

    end = obstacles.sample_safe_point()
    while end.l1_distance(start) < 19:
        end = obstacles.sample_safe_point()

    end._type = 'end'
    cloud.add_point(end)

    return cloud

def main():
    num_obstacles = 3
    resolution = 20
    out_file = "point-cloud.cld"

    for i, arg in enumerate(sys.argv):
        if i == len(sys.argv) - 1:
            break

        if arg == "--num":
            num_obstacles = int(sys.argv[i + 1])
            
        if arg == "--res":
            resolution = int(sys.argv[i + 1])
        
        if arg == "--out":
            out_file = sys.argv[i + 1]
    
    num_points = num_obstacles * resolution

    cloud = gen_new_cloud(num_obstacles, num_points)
    cloud.to_file(out_file, Point(-10, -10, -10), Point(10, 10, 10))

if __name__ == "__main__":
    main()