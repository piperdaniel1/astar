use std::{fs::File, io::prelude::*};

#[derive(Debug, Clone, Copy, PartialEq)]
enum PointType {
    Obstacle,
    Start,
    Goal,
}

#[derive(Debug, Clone, Copy)]
struct Point {
    x: f64,
    y: f64,
    z: f64,
    _type: PointType,
}

impl Point {
    fn new(x: f64, y: f64, z: f64, _type: PointType) -> Point {
        Point { x, y, z, _type }
    }
}

#[derive(Debug)]
struct AStar {
    start_point: Point,
    goal_point: Point,
    tl_point: Point,
    br_point: Point,
    points: Vec<Point>,
}

impl AStar {
    fn new(start_point: Point, goal_point: Point, tl_point: Point, br_point: Point, points: Vec<Point>) -> AStar {
        AStar {
            start_point,
            goal_point,
            tl_point,
            br_point,
            points,
        }
    }

    fn find_path(&self) -> Vec<Point> {
        let cell_density: f64 = 0.25;
        let x_cells = ((self.br_point.x - self.tl_point.x) / cell_density).ceil() as usize;
        let y_cells = ((self.br_point.y - self.tl_point.y) / cell_density).ceil() as usize;

        let mut grid: Vec<Vec<Vec<Point>>> = Vec::new();
        
        // resize grid to x_cells * y_cells
        grid.resize(x_cells, Vec::new());
        grid.iter_mut().for_each(|v| v.resize(y_cells, Vec::new()));

        // now there is an empty vec at each grid cell
        // lets add the points

        for point in &self.points {
            let x_cell = ((point.x - self.tl_point.x) / cell_density).floor() as usize;
            let y_cell = ((point.y - self.tl_point.y) / cell_density).floor() as usize;
            grid[x_cell][y_cell].push(*point);
        }

        // now we have a grid of points
        // lets print it out
    
        let x_start_cell = ((self.start_point.x - self.tl_point.x) / cell_density).floor() as usize;
        let y_start_cell = ((self.start_point.y - self.tl_point.y) / cell_density).floor() as usize;

        let x_goal_cell = ((self.goal_point.x - self.tl_point.x) / cell_density).floor() as usize;
        let y_goal_cell = ((self.goal_point.y - self.tl_point.y) / cell_density).floor() as usize;

        for (x, row) in grid.iter().enumerate() {
            for (y, cell) in row.iter().enumerate() {
                if x == x_start_cell && y == y_start_cell {
                    print!("S ");
                } else if x == x_goal_cell && y == y_goal_cell {
                    print!("G ");
                } else if cell.len() != 0 {
                    print!("{} ", cell.len());
                } else {
                    print!("- ");
                }
            }
            println!();
        }

        let mut path: Vec<Point> = Vec::new();
        path.push(self.start_point);
        path
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();

    let filename = if args.len() > 1 {
        args[1].clone()
    } else {
        "point-cloud.cld".to_string()
    };

    let mut input = File::open(filename).unwrap();
    let mut input_str = String::new();
    input.read_to_string(&mut input_str).unwrap();

    let mut input_lines = input_str.lines();

    input_lines.next();

    let line = input_lines.next().unwrap();
    let tl_point = Point::new(
        line.split_whitespace().nth(0).unwrap().parse().unwrap(),
        line.split_whitespace().nth(1).unwrap().parse().unwrap(),
        line.split_whitespace().nth(2).unwrap().parse().unwrap(),
        PointType::Obstacle,
    );

    let line = input_lines.next().unwrap();
    let br_point = Point::new(
        line.split_whitespace().nth(0).unwrap().parse().unwrap(),
        line.split_whitespace().nth(1).unwrap().parse().unwrap(),
        line.split_whitespace().nth(2).unwrap().parse().unwrap(),
        PointType::Obstacle,
    );

    let line = input_lines.next().unwrap();
    let start_point = Point::new(
        line.split_whitespace().nth(0).unwrap().parse().unwrap(),
        line.split_whitespace().nth(1).unwrap().parse().unwrap(),
        line.split_whitespace().nth(2).unwrap().parse().unwrap(),
        PointType::Start,
    );

    let line = input_lines.next().unwrap();
    let goal_point = Point::new(
        line.split_whitespace().nth(0).unwrap().parse().unwrap(),
        line.split_whitespace().nth(1).unwrap().parse().unwrap(),
        line.split_whitespace().nth(2).unwrap().parse().unwrap(),
        PointType::Goal,
    );

    input_lines.next();

    let mut points: Vec<Point> = Vec::new();
    for line in input_lines {
        let point = Point::new(
            line.split_whitespace().nth(0).unwrap().parse().unwrap(),
            line.split_whitespace().nth(1).unwrap().parse().unwrap(),
            line.split_whitespace().nth(2).unwrap().parse().unwrap(),
            PointType::Obstacle,
        );
        points.push(point);
    }

    let astar = AStar::new(start_point, goal_point, tl_point, br_point, points);
    astar.find_path();


}
