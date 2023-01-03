use std::{fs::File, io::prelude::*};

enum PointType {
    Obstacle,
    Start,
    Goal,
}

struct Point {
    x: f64,
    y: f64,
    z: f64,
    _type: PointType,
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

    let input_lines = input_str.lines();

    for line in input_lines {

    }
}
