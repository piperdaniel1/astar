import sys
import random
from matplotlib import pyplot as plt

class Point:
    def __init__(self, x, y, z, _type='obstacle'):
        self.x = x
        self.y = y
        self.z = z
        self._type = _type
    
    def l1_distance(self, other):
        return abs(self.x - other.x) + abs(self.y - other.y) + abs(self.z - other.z)

    def __str__(self):
        return f"{self.x=} {self.y=} {self.z=} {self._type}"
    
    def __repr__(self):
        return f"Point({self.x}, {self.y}, {self.z}, _type='{self._type}')"

class PointCloud:
    def __init__(self, point_list):
        self.point_list = point_list
        self.start_point = None
        self.end_point = None
    
    def __str__(self):
        return f"PointCloud (n={len(self.point_list)}): {self.point_list}"
    
    def add_point(self, point):
        if point._type == 'start':
            self.start_point = point
        elif point._type == 'end':
            self.end_point = point
        else:
            self.point_list.append(point)
    
    def plot(self, tl_corner: Point, br_corner: Point):
        for point in self.point_list:
            plt.scatter(point.x, point.y, c='b')

        assert(self.start_point is not None)
        plt.scatter(self.start_point.x, self.start_point.y, c='g')

        assert(self.end_point is not None)
        plt.scatter(self.end_point.x, self.end_point.y, c='r')
        
        plt.xlim(tl_corner.x, br_corner.x)
        plt.ylim(tl_corner.y, br_corner.y)
        
        plt.show()
    
    def to_file(self, filename):
        with open(filename, 'w') as f:
            if self.start_point is not None and self.end_point is not None:
                f.write(f"{self.start_point.x} {self.start_point.y} {self.start_point.z}")
                f.write(f"\n{self.end_point.x} {self.end_point.y} {self.end_point.z}")
            else:
                raise ValueError("Start and end points must be set")

            f.write(f"\n{len(self.point_list)}")
            for point in self.point_list:
                    f.write(f"\n{point.x} {point.y} {point.z}")


class Obstacle:
    def __init__(self, _type, center):
        self._type = _type
        self.center = center
    
    def __str__(self):
        return f"{self._type=} {self.center=}"
    
    def is_colliding(self, point):
        raise NotImplementedError

class Sphere(Obstacle):
    def __init__(self, center, radius):
        super().__init__('sphere', center)
        self.radius = radius
    
    def is_colliding(self, point):
        return (point.x - self.center.x)**2 + (point.y - self.center.y)**2 + (point.z - self.center.z)**2 <= self.radius**2

class ObstacleSpace:
    def __init__(self, obstacle_list, tl_corner, br_corner):
        self.obstacle_list = obstacle_list
        self.tl_corner: Point = tl_corner
        self.br_corner: Point = br_corner
    
    def is_colliding(self, point):
        for obstacle in self.obstacle_list:
            if obstacle.is_colliding(point):
                return True
        return False
    
    def add_obstacle(self, obstacle):
        self.obstacle_list.append(obstacle)
    
    # oh god this is so inefficient
    def sample_colliding_point(self):
        while True:
            point = Point(random.uniform(self.tl_corner.x, self.br_corner.x), random.uniform(self.tl_corner.y, self.br_corner.y), random.uniform(self.tl_corner.z, self.br_corner.z))
            if self.is_colliding(point):
                return point
    
    def sample_safe_point(self):
        while True:
            point = Point(random.uniform(self.tl_corner.x, self.br_corner.x), random.uniform(self.tl_corner.y, self.br_corner.y), random.uniform(self.tl_corner.z, self.br_corner.z))
            if not self.is_colliding(point):
                return point
    
def print_help():
    print("Usage:")
    print("python3 point-gen.py --new <num-obstacles>")
    sys.exit(1)

def gen_new_cloud(num_obstacles) -> PointCloud:
    cloud = PointCloud([])
    obstacles = ObstacleSpace([], Point(-10, -10, -10), Point(10, 10, 10))

    for _ in range(num_obstacles):
        center = Point(random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(-10, 10))
        radius = random.uniform(0.5, 2)
        obstacles.add_obstacle(Sphere(center, radius))
    
    for _ in range(50):
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
    if len(sys.argv) == 1:
        print_help()
    elif sys.argv[1] == "--new":
        if len(sys.argv) != 3:
            print_help()

        num_obstacles = int(sys.argv[2])

        cloud = gen_new_cloud(num_obstacles)

        print(cloud)
        cloud.plot(Point(-10, -10, -10), Point(10, 10, 10))

        cloud.to_file("test.cld")
        

if __name__ == "__main__":
    main()