use nalgebra::{Point3};
use std::collections::HashMap;
use rand::prelude::*;

#[derive(Debug)]
pub struct HashGrid {
    cell_size: f32,
    cells: HashMap<(i32, i32, i32), Vec<usize>>,
    points: Vec<Point3<f32>>,
}

impl HashGrid {
    pub fn new(cell_size: f32) -> Self {
        HashGrid {
            cell_size,
            cells: HashMap::new(),
            points: Vec::new(),
        }
    }

    pub fn insert(&mut self, point: Point3<f32>) {
        self.points.push(point);
        let index = self.points.len() - 1;
        let cell_coord = self.cell_coord(&point);
        let cell = self.cells.entry(cell_coord).or_insert_with(|| Vec::new());
        cell.push(index);
    }

    fn cell_coord(&self, point: &Point3<f32>) -> (i32, i32, i32) {
        (
            (point.x / self.cell_size).floor() as i32,
            (point.y / self.cell_size).floor() as i32,
            (point.z / self.cell_size).floor() as i32,
        )
    }

    pub fn aabb_query(&self, aabb_min: &Point3<f32>, aabb_max: &Point3<f32>) -> Vec<Point3<f32>> {
        let min_cell = self.cell_coord(aabb_min);
        let max_cell = self.cell_coord(aabb_max);

        let mut result = Vec::new();

        for x in min_cell.0..=max_cell.0 {
            for y in min_cell.1..=max_cell.1 {
                for z in min_cell.2..=max_cell.2 {
                    if let Some(cell) = self.cells.get(&(x, y, z)) {
                        for &index in cell {
                            let point = self.points[index];
                            if point.coords >= aabb_min.coords && point.coords <= aabb_max.coords {
                                result.push(point);
                            }
                        }
                    }
                }
            }
        }

        result
    }

    pub fn point_query(&self, point: &Point3<f32>, radius: f32) -> Vec<Point3<f32>> {
        let center_cell = self.cell_coord(point);
        let radius_cell = (radius / self.cell_size).ceil() as i32;

        let mut result = Vec::new();

        for x in center_cell.0 - radius_cell..=center_cell.0 + radius_cell {
            for y in center_cell.1 - radius_cell..=center_cell.1 + radius_cell {
                for z in center_cell.2 - radius_cell..=center_cell.2 + radius_cell {
                    if let Some(cell) = self.cells.get(&(x, y, z)) {
                        for &index in cell {
                            let other_point = self.points[index];
                            let dist = (other_point - point).norm();
                            if dist <= radius {
                                result.push(other_point);
                            }
                        }
                    }
                }
            }
        }

        result
    }
}

pub fn main() {
    /*// Create a HashGrid with cell size 1.0
    let mut grid = HashGrid::new(10.0);

    // Add some random points
    let mut rng = rand::thread_rng();
    for _ in 0..100 {
        let x = rng.gen_range(-50.0..50.0);
        let y = rng.gen_range(-50.0..50.0);
        let z = rng.gen_range(-50.0..50.0);
        grid.insert(Point3::new(x, y, z));
    }

    println!("{:?}", grid);

    // Perform a point query for points within a radius of 10.0 from (0, 0, 0)
    let center_point = Point3::new(0.0, 0.0, 0.0);
    let radius = 100.0;
    let result_points = grid.point_query(center_point, radius);
    println!("Point query result:");
    for point in result_points {
        println!("  {:?}", point);
    }

    // Perform an AABB query for points within the AABB defined by (-5, -5, -5) and (5, 5, 5)
    let aabb_min = Point3::new(-20.0, -20.0, -20.0);
    let aabb_max = Point3::new(20.0, 20.0, 20.0);
    let result_points = grid.aabb_query(aabb_min, aabb_max);
    println!("AABB query result:");
    for point in result_points {
        println!("  {:?}", point);
    }*/
}