use nalgebra::{Point3, Vector3};
use rand::prelude::*;

use std::cmp::Ordering;

#[derive(Clone, Debug)]
struct BVHNode {
    aabb: AABB,
    left: Option<Box<BVHNode>>,
    right: Option<Box<BVHNode>>,
    points: Vec<Point3<f32>>,
}

#[derive(Clone, Debug)]
pub struct AABB {
    min: Point3<f32>,
    max: Point3<f32>,
}
    
impl AABB {
    fn new(min: Point3<f32>, max: Point3<f32>) -> Self {
        AABB {
            min,
            max,
        }
    }

    pub fn surface_area(&self) -> f32 {
        let extent = self.max - self.min;
        2.0 * (extent.x * extent.y + extent.y * extent.z + extent.z * extent.x)
    }
    
    fn center(&self) -> Point3<f32> {
        Point3::new(
            (self.min.x + self.max.x) / 2.0,
            (self.min.y + self.max.y) / 2.0, 
            (self.min.z + self.max.z) / 2.0,
        )
    }
    
    fn half_size(&self) -> Vector3<f32> {
        (self.max - self.min) / 2.0
    }
    
    fn intersects(&self, other: &AABB) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
            && self.min.z <= other.max.z
            && self.max.z >= other.min.z
    }
    
    fn contains(&self, point: &Point3<f32>) -> bool {
        self.min.x <= point.x
            && self.max.x >= point.x
            && self.min.y <= point.y
            && self.max.y >= point.y
            && self.min.z <= point.z
            && self.max.z >= point.z
    }
    
    fn contains_sphere(&self, center: &Point3<f32>, radius: f32) -> bool {
        let closest_point = Point3::new(
            center.x.max(self.min.x).min(self.max.x),
            center.y.max(self.min.y).min(self.max.y),
            center.z.max(self.min.z).min(self.max.z),
        );
        (center - closest_point).norm() <= radius
    }    

    fn extent(&self) -> Vector3<f32> {
        self.max - self.min
    }

    fn grow(&mut self, point: &Point3<f32>) {
        self.min.x = self.min.x.min(point.x);
        self.min.y = self.min.y.min(point.y);
        self.min.z = self.min.z.min(point.z);
        self.max.x = self.max.x.max(point.x);
        self.max.y = self.max.y.max(point.y);
        self.max.z = self.max.z.max(point.z);
    }

    fn distance_squared(&self, point: &Point3<f32>) -> f32 {
        let extent = self.extent();
        let clamp = |value, min_value, max_value| {
            if value < min_value {
                min_value
            } else if value > max_value {
                max_value
            } else {
                value
            }
        };
        let x = clamp(point.x, self.min.x, self.max.x);
        let y = clamp(point.y, self.min.y, self.max.y);
        let z = clamp(point.z, self.min.z, self.max.z);
        let closest_point = Point3::new(x, y, z);
        (closest_point - *point).norm_squared()
    }
}

impl BVHNode {
    fn new(points: Vec<Point3<f32>>) -> Self {
        let mut aabb = AABB::new(
            Point3::new(std::f32::INFINITY, std::f32::INFINITY, std::f32::INFINITY),
            Point3::new(std::f32::NEG_INFINITY, std::f32::NEG_INFINITY, std::f32::NEG_INFINITY),
        );
        for point in &points {
            aabb.grow(&point);
        }
        BVHNode {
            aabb,
            left: None,
            right: None,
            points,
        }
    }

    fn aabb_query(&self, aabb: &AABB) -> Vec<&Point3<f32>> {
        if !self.aabb.intersects(aabb) {
            return vec![];
        }

        let mut result = vec![];

        if let Some(left) = &self.left {
            result.append(&mut left.aabb_query(aabb));
        }

        if let Some(right) = &self.right {
            result.append(&mut right.aabb_query(aabb));
        }

        result.extend(self.points.iter().filter(|&point| aabb.contains(&point)));

        result
    }

    fn point_query<'a>(&'a self, point: &Point3<f32>, radius: f32, result: &mut Vec<&'a Point3<f32>>) {
        if !self.aabb.contains_sphere(point, radius) {
            return;
        }

        result.extend(self.points.iter().filter(|&p| {
            (point - *p).norm() <= radius
        }));

        if let Some(ref left) = self.left {
            left.point_query(point, radius, result);
        }

        if let Some(ref right) = self.right {
            right.point_query(point, radius, result);
        }
    }
}

#[derive(Debug)]
pub struct BVH {
    root: Option<Box<BVHNode>>,
}

impl BVH {
    pub fn new(points: &Vec<Point3<f32>>) -> Self {
        let root = Self::build(points);
        BVH { root }
    }

    fn build(points: &[Point3<f32>]) -> Option<Box<BVHNode>> {
        if points.is_empty() {
            return None;
        }

        if points.len() <= 10 {
            return Some(Box::new(BVHNode::new(points.to_vec())));
        }

        let mut aabb = AABB::new(
            Point3::new(std::f32::INFINITY, std::f32::INFINITY, std::f32::INFINITY),
            Point3::new(std::f32::NEG_INFINITY, std::f32::NEG_INFINITY, std::f32::NEG_INFINITY),
        );

        for point in points {
            aabb.grow(&point);
        }
    
        let extent = aabb.extent();
    
        let axis = if extent.x >= extent.y && extent.x >= extent.z {
            0
        } else if extent.y >= extent.x && extent.y >= extent.z {
            1
        } else {
            2
        };
    
        let median = Self::median(points, axis);
    
        let left = Self::build(&points[..median]);
        let right = Self::build(&points[median..]);
    
        Some(Box::new(BVHNode {
            aabb,
            left,
            right,
            points: vec![],
        }))
    }

    pub fn build_sah(points: &[Point3<f32>]) -> Self {
        BVH {
            root: Self::build_sah_impl(points, 0, 20)
        }
    }

    fn build_sah_impl(points: &[Point3<f32>], depth: usize, max_depth: usize) -> Option<Box<BVHNode>> {
        if points.is_empty() {
            return None;
        }
        if depth == max_depth || points.len() <= 2 {
                return Some(Box::new(BVHNode {
                    aabb: AABB::new(points.iter().fold(Point3::origin(), |a, b| a.coords.inf(&b.coords).into()),
                                    points.iter().fold(Point3::origin(), |a, b| a.coords.sup(&b.coords).into())),
                    left: None,
                    right: None,
                    points: points.to_vec(),
                }))
            };
        let bbox = AABB::new(points.iter().fold(Point3::origin(), |a, b| a.coords.inf(&b.coords).into()),
                             points.iter().fold(Point3::origin(), |a, b| a.coords.sup(&b.coords).into()));
        let axis = bbox.max - bbox.min;
        let split_axis = if axis.x > axis.y && axis.x > axis.z { 0 }
                         else if axis.y > axis.z { 1 }
                         else { 2 };
        let mut split = points.len() / 2;
        let mut sah_cost = std::f32::INFINITY;
        let mut left_aabb = AABB::new(Point3::origin(), Point3::origin());
        let mut right_aabb = AABB::new(Point3::origin(), Point3::origin());

        let mut points = points.to_vec();

        points.sort_by(|a, b| a[split_axis].partial_cmp(&b[split_axis]).unwrap());

        for i in 0..points.len() - 1 {
            left_aabb.min = left_aabb.min.inf(&points[i]);
            left_aabb.max = left_aabb.max.sup(&points[i]);
            right_aabb.min = right_aabb.min.inf(&points[i]);
            right_aabb.max = right_aabb.max.sup(&points[i]);
            let sah = (i + 1) as f32 * left_aabb.surface_area() + (points.len() - i - 1) as f32 * right_aabb.surface_area();
            if sah < sah_cost {
                sah_cost = sah;
                split = i + 1;
            }
        }
        let (left_points, right_points) = points.split_at_mut(split);
        Some(Box::new(BVHNode {
            aabb: bbox,
            left: Self::build_sah_impl(left_points, depth + 1, max_depth),
            right: Self::build_sah_impl(right_points, depth + 1, max_depth),
            points: vec![],
            }))
    }
    
    fn median(points: &[Point3<f32>], axis: usize) -> usize {
        let mut points = points.to_vec();

        points.sort_unstable_by(|a, b| {
            let a_val = a[axis];
            let b_val = b[axis];
            if a_val < b_val {
                Ordering::Less
            } else if a_val > b_val {
                Ordering::Greater
            } else {
                Ordering::Equal
            }
        });
    
        points.len() / 2
    }
    
    pub fn aabb_query(&self, aabb: &AABB) -> Vec<&Point3<f32>> {
        if let Some(root) = &self.root {
            root.aabb_query(aabb)
        } else {
            vec![]
        }
    }
    
    pub fn point_query(&self, point: &Point3<f32>, radius: f32) -> Vec<&Point3<f32>> {
        if let Some(root) = &self.root {
            let mut result = Vec::new();

            root.point_query(point, radius, &mut result);

            result
        } else {
            vec![]
        }
    }
}

pub fn main() {
    /*// Example usage
    let points = vec![
    Point { position: Point3::new(0.0, 0.0, 0.0) },
    Point { position: Point3::new(1.0, 1.0, 1.0) },
    Point { position: Point3::new(2.0, 2.0, 2.0) },
    Point { position: Point3::new(3.0, 3.0, 3.0) },
    ];

    let bvh = BVH::new(points);

    let aabb = AABB::new(
        Point3::new(-1.0, -1.0, -1.0),
        Point3::new(2.0, 2.0, 2.0),
    );
    let points_within_aabb = bvh.aabb_query(&aabb);
    println!("Points within AABB: {:?}", points_within_aabb);

    let point = Point3::new(1.0, 1.0, 1.0);
    let radius = 1.0;
    let points_within_radius = bvh.point_query(&point, radius);
    println!("Points within radius: {:?}", points_within_radius); */

    // Create a HashGrid with cell size 1.0
    

    /*let bvh = BVH::new(list);

    // Add some random points
    

    println!("{:?}", bvh);

    // Perform a point query for points within a radius of 10.0 from (0, 0, 0)
    let center_point = Point3::new(0.0, 0.0, 0.0);
    let radius = 100.0;
    let result_points = bvh.point_query(&center_point, radius);
    println!("Point query result:");
    for point in result_points {
        println!("  {:?}", point);
    }

    // Perform an AABB query for points within the AABB defined by (-5, -5, -5) and (5, 5, 5)
    let min = Point3::new(-20.0, -20.0, -20.0);
    let max = Point3::new(20.0, 20.0, 20.0);
    let result_points = bvh.aabb_query(&AABB::new(min, max));
    println!("AABB query result:");
    for point in result_points {
        println!("  {:?}", point);
    }*/
}
