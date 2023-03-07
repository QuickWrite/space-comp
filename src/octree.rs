use nalgebra::{Point3, Vector3};

#[derive(Debug)]
pub struct OcTree {
    bounds: BoundingBox,
    points: Vec<Point3<f32>>,
    children: Option<[Box<OcTree>; 8]>,
    depth: usize,
}

impl OcTree {
    pub fn new(bounds: BoundingBox) -> Self {
        OcTree {
            bounds,
            points: Vec::new(),
            children: None,
            depth: 25,
        }
    }

    fn new_depth(bounds: BoundingBox, depth: usize) -> Self  {
        OcTree {
            bounds,
            points: Vec::new(),
            children: None,
            depth: depth,
        }
    }

    pub fn insert(&mut self, point: Point3<f32>) {
        self.insert_impl(point, self.depth);
    }

    fn insert_impl(&mut self, point: Point3<f32>, depth: usize) {
        if depth <= 0 {
            self.points.push(point);
            return;
        }

        if self.children.is_none() {
            self.points.push(point);
            if self.points.len() > 15 { // Arbitrary number of points to subdivide
                self.subdivide(depth - 1);
            }
        } else {
            let index = self.get_index(point);
            self.children.as_mut().unwrap()[index].insert_impl(point, depth - 1);
        }
    }

    fn subdivide(&mut self, depth: usize) {
        let center = self.bounds.center();
        let half_size = self.bounds.half_size();

        let mut children = [
            Box::new(OcTree::new_depth(BoundingBox::new(
                center + Vector3::new(-half_size.x, -half_size.y, -half_size.z),
                half_size,
            ), depth)),
            Box::new(OcTree::new_depth(BoundingBox::new(
                center + Vector3::new(half_size.x, -half_size.y, -half_size.z),
                half_size,
            ), depth)),
            Box::new(OcTree::new_depth(BoundingBox::new(
                center + Vector3::new(-half_size.x, half_size.y, -half_size.z),
                half_size,
            ), depth)),
            Box::new(OcTree::new_depth(BoundingBox::new(
                center + Vector3::new(half_size.x, half_size.y, -half_size.z),
                half_size,
            ), depth)),
            Box::new(OcTree::new_depth(BoundingBox::new(
                center + Vector3::new(-half_size.x, -half_size.y, half_size.z),
                half_size,
            ), depth)),
            Box::new(OcTree::new_depth(BoundingBox::new(
                center + Vector3::new(half_size.x, -half_size.y, half_size.z),
                half_size,
            ), depth)),
            Box::new(OcTree::new_depth(BoundingBox::new(
                center + Vector3::new(-half_size.x, half_size.y, half_size.z),
                half_size,
            ), depth)),
            Box::new(OcTree::new_depth(BoundingBox::new(
                center + Vector3::new(half_size.x, half_size.y, half_size.z),
                half_size,
            ), depth)),
        ];

        for point in &self.points {
            let index = self.get_index(*point);
            children[index].insert_impl(*point, depth);
        }

        self.points.clear();
        self.children = Some(children);
    }

    fn get_index(&self, point: Point3<f32>) -> usize {
        let center = self.bounds.center();
        let is_left = point.x <= center.x;
        let is_down = point.y <= center.y;
        let is_back = point.z <= center.z;
        match (is_left, is_down, is_back) {
            (false, false, false) => 0,
            (true, false, false) => 1,
            (false, true, false) => 2,
            (true, true, false) => 3,
            (false, false, true) => 4,
            (true, false, true) => 5,
            (false, true, true) => 6,
            (true, true, true) => 7,
        }
    }

    pub fn aabb_query(&self, aabb_min: Point3<f32>, aabb_max: Point3<f32>) -> Vec<Point3<f32>> {
        let mut results = Vec::new();

        let query_bounds = &BoundingBox { min: aabb_min, max: aabb_max };

        self.aabb_query_impl(query_bounds, &mut results);

        results
    }
    
    fn aabb_query_impl(&self, query_bounds: &BoundingBox, results: &mut Vec<Point3<f32>>) {
        if !self.bounds.intersects(&query_bounds) {
            return;
        }
    
        if self.children.is_none() {
            results.extend(self.points.iter().filter(|&p| query_bounds.contains(*p)));
        } else {
            for child in self.children.as_ref().unwrap() {
                child.aabb_query_impl(&query_bounds, results);
            }
        }
    }

    pub fn point_query(&self, query_point: &Point3<f32>, radius: f32) -> Vec<Point3<f32>> {
        let mut results = Vec::new();

        self.point_query_impl(query_point, radius, &mut results);

        results
    }
    
    fn point_query_impl(&self, query_point: &Point3<f32>, radius: f32, results: &mut Vec<Point3<f32>>) {
        if !self.bounds.contains_sphere(query_point, radius) {
            return;
        }

        if self.children.is_some() {
            for child in self.children.as_ref().unwrap() {
                child.point_query_impl(query_point, radius, results);
            }
        } else {
            results.extend(self.points.iter().filter(|&p| (query_point - *p).norm() <= radius));
        }
    }
}

#[derive(Debug)]
pub struct BoundingBox {
    min: Point3<f32>,
    max: Point3<f32>,
}
    
impl BoundingBox {
    pub fn new(center: Point3<f32>, half_size: Vector3<f32>) -> Self {
        BoundingBox {
            min: center - half_size,
            max: center + half_size,
        }
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
    
    fn intersects(&self, other: &BoundingBox) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
            && self.min.z <= other.max.z
            && self.max.z >= other.min.z
    }
    
    fn contains(&self, point: Point3<f32>) -> bool {
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
}

pub fn main() {
    /*let mut tree = OcTree::new(BoundingBox::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(10.0, 10.0, 10.0)));

    // Insert some points
    tree.insert(Point3::new(1.0, 2.0, 3.0));
    tree.insert(Point3::new(-4.0, 5.0, 6.0));
    tree.insert(Point3::new(7.0, 8.0, -9.0));
    tree.insert(Point3::new(-10.0, -11.0, 12.0));
    tree.insert(Point3::new(13.0, -14.0, -15.0));

    // Perform AABB query
    let query_bounds = BoundingBox::new(Point3::new(-2.0, -3.0, -4.0), Vector3::new(6.0, 8.0, 10.0));
    let mut results = Vec::new();
    tree.aabb_query(&query_bounds, &mut results);
    println!("AABB query results:");
    for result in results {
        println!("  {:?}", result);
    }

    // Perform point query
    let query_point = Point3::new(0.0, 0.0, 0.0);
    let radius = 5.0;
    let mut results = Vec::new();
    tree.point_query(query_point, radius, &mut results);
    println!("Point query results:");
    for result in results {
        println!("  {:?}", result);
    } */

    // Create a HashGrid with cell size 1.0
    /*let mut octree = OcTree::new(BoundingBox {
        min: Point3::new(-50.0, -50.0, -50.0), 
        max: Point3::new(50.0, 50.0, 50.0)
    });

    // Add some random points
    let mut rng = rand::thread_rng();
    for _ in 0..100 {
        let x = rng.gen_range(-50.0..50.0);
        let y = rng.gen_range(-50.0..50.0);
        let z = rng.gen_range(-50.0..50.0);
        octree.insert(Point3::new(x, y, z));
    }

    println!("{:#?}", octree);

    // Perform a point query for points within a radius of 10.0 from (0, 0, 0)
    let center_point = Point3::new(0.0, 0.0, 0.0);
    let radius = 100.0;
    let result_points = octree.point_query(center_point, radius);
    println!("Point query result:");
    for point in result_points {
        println!("  {:?}", point);
    }

    // Perform an AABB query for points within the AABB defined by (-5, -5, -5) and (5, 5, 5)
    let aabb_min = Point3::new(-20.0, -20.0, -20.0);
    let aabb_max = Point3::new(20.0, 20.0, 20.0);
    let result_points = octree.aabb_query(aabb_min, aabb_max);
    println!("AABB query result:");
    for point in result_points {
        println!("  {:?}", point);
    }*/
}
