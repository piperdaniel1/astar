use std::{fs::File, io::prelude::*};

#[derive(Debug)]
enum PointType {
    Obstacle,
    Start,
    Goal,
}

#[derive(Debug)]
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

    println!("{:?}", start_point);
    println!("{:?}", goal_point);
    println!("{:#?}", points);
}
