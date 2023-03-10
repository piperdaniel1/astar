# astar

## Active tooling:
 - astar-cpp for generating 3d paths using the astar algorithm
 - python for generating sample point clouds and plotting
 - rust-cpp is currently inactive
 
### Build astar-cpp:
```
make
```
Or you can run ```make optimized``` to build with full compiler optimization. Run ```make clean``` to remove the executable.

### Running astar-cpp
```
./main --in [cloud_file] --out [output_path_file] --res [cell resolution]
```
If no input file is provided the program attempts to look for a file called ```point-cloud.cld```.
If no output file is specified the program just prints the resulting path to standard out. If no resolution is provided, the value ```20``` is used.
This means the pathfinding cells will be arranged in a ```20x20x20``` grid.

### Running point-gen.py (native python3)
```
python3 point-gen.py --num [number of obstacles] --res [# samples per obstacle] --out [filename]
```
If no ```--num``` is provided the value ```3``` is used. If no --res is provided, the value ```20``` is used. The points in the output point cloud are
generated by creating a ```--num``` of obstacles. Currently, these are spheres with a random radius. Then, random points in the space that are inside the obstacles are found until we reach ```--num * --res``` points. If no filename is provided the name ```point-cloud.cld``` is used.

### Running plot.py (needs matplotlib)
```
python3 plot.py --cloud [cloud file] --path [path file]
```
If no ```--cloud``` is provided, the program looks for a file called ```point-cloud.cld```. If no path is provided, the program looks for a file called ```path.pld```. This data is plotted in a 3D plot using matplotlib.
