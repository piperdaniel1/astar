from matplotlib import pyplot as plt
import random

# File for storing classes and functions for points
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
        self.path = []
    
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
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        for point in self.point_list:
            ax.scatter(point.x, point.y, point.z, c='b')

        for point in self.path:
            ax.scatter(point.x, point.y, point.z, c='g')

        assert(self.start_point is not None)
        ax.scatter(self.start_point.x, self.start_point.y, self.start_point.z, c='g')

        assert(self.end_point is not None)
        ax.scatter(self.end_point.x, self.end_point.y, self.end_point.z, c='r')

        ax.set_xlim(tl_corner.x, br_corner.x)
        ax.set_ylim(tl_corner.y, br_corner.y)
        ax.set_zlim(tl_corner.z, br_corner.z)

        plt.show()

    def to_file(self, filename, tl_corner: Point, br_corner: Point):
        if self.start_point is None or self.end_point is None:
            raise ValueError("Start and end points must be set")

        with open(filename, 'w') as f:
            f.write(f"version=1")
            f.write(f"\n{tl_corner.x} {tl_corner.y} {tl_corner.z}")
            f.write(f"\n{br_corner.x} {br_corner.y} {br_corner.z}")

            f.write(f"\n{self.start_point.x} {self.start_point.y} {self.start_point.z}")
            f.write(f"\n{self.end_point.x} {self.end_point.y} {self.end_point.z}")

            f.write(f"\n{len(self.point_list)}")
            for point in self.point_list:
                    f.write(f"\n{point.x} {point.y} {point.z}")

    def import_path(self, path):
        self.path = path

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