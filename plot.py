import sys
from points import *

def gen_cloud_from_file(filename) -> PointCloud:
    with open(filename) as f:
        lines = f.readlines()
    
    if lines[0].split("=")[1].strip() != "1":
        raise ValueError("Invalid file version")
    
    start: Point = Point(float(lines[3].split()[0]), float(lines[3].split()[1]), float(lines[3].split()[2]))
    end: Point = Point(float(lines[4].split()[0]), float(lines[4].split()[1]), float(lines[4].split()[2]))

    num_points = int(lines[5])
    points = []
    for i in range(num_points):
        points.append(Point(float(lines[6 + i].split()[0]), float(lines[6 + i].split()[1]), float(lines[6 + i].split()[2])))

    cloud = PointCloud(points)
    cloud.start_point = start
    cloud.end_point = end

    print(cloud.start_point)
    print(cloud.end_point)

    return cloud

def main():
    cloud_file = "point-cloud.cld"
    path_file = "path.pcl"

    for i, arg in enumerate(sys.argv):
        if i == len(sys.argv) - 1:
            break

        if arg == "--cloud":
            cloud_file = sys.argv[i + 1]
            break
            
        if arg == "--path":
            path_file = sys.argv[i + 1]
            break
    cloud = gen_cloud_from_file(cloud_file)

    path = []
    try:
        with open(path_file) as f:
            path_lines = f.readlines()

        for line in path_lines:
            cord_split = line.split()
            path.append(Point(float(cord_split[0]), float(cord_split[1]), float(cord_split[2])))
        
        cloud.import_path(path)
    except FileNotFoundError:
        pass

    cloud.plot(Point(-10, -10, -10), Point(10, 10, 10))

if __name__ == "__main__":
    main()