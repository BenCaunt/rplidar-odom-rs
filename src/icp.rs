use rplidar_drv::ScanPoint;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Point2 {
    pub x: f32,
    pub y: f32,
}


#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Pose2d {
    pub x: f32,
    pub y: f32,
    pub theta: f32,
}

impl Pose2d {
    pub fn new(x: f32, y: f32, theta: f32) -> Self {
        Self { x, y, theta }
    }
    pub fn compose(&self, other: &Self) -> Self {
        let x = self.x + other.x * self.theta.cos() - other.y * self.theta.sin();
        let y = self.y + other.x * self.theta.sin() + other.y * self.theta.cos();   
        let theta = self.theta + other.theta;
        Self::new(x, y, theta)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PointCloud(Vec<Point2>);

impl PointCloud {
    pub fn new(points: Vec<Point2>) -> Self {
        Self(points)
    }
    
}

impl From<&Vec<ScanPoint>> for PointCloud {
    fn from(points: &Vec<ScanPoint>) -> Self {
        Self(
            points
                .into_iter()
                .map(|p| Point2 {
                    x: p.angle(),
                    y: p.distance(),
                })
                .collect(),
        )
    }
}

fn find_closest_point(point: Point2, model_points: &PointCloud) -> Point2 {
    let mut min_distance = f32::INFINITY;
    let mut closest_point = model_points.0[0];

    for model_point in &model_points.0 {
        let distance =
            (point.x - model_point.x).powi(2) + (point.y - model_point.y).powi(2);
        if distance < min_distance {
            min_distance = distance;
            closest_point = *model_point;
        }
    }

    closest_point
}

fn calculate_centroid(point_cloud: &PointCloud) -> Point2 {
    let mut sum_x = 0.0;
    let mut sum_y = 0.0;

    for point in &point_cloud.0 {
        sum_x += point.x;
        sum_y += point.y;
    }

    let num_points = point_cloud.0.len() as f32;
    Point2 {
        x: sum_x / num_points,
        y: sum_y / num_points,
    }
}

fn calculate_error(
    scene_points: &PointCloud,
    model_points: &PointCloud,
) -> f32 {
    let mut error = 0.0;

    for point in &scene_points.0 {
        let closest_point = find_closest_point(*point, model_points);
        error += (point.x - closest_point.x).powi(2)
            + (point.y - closest_point.y).powi(2);
    }

    error / scene_points.0.len() as f32
}

pub fn icp(
    scene_points: &PointCloud,
    model_points: &PointCloud,
    max_iterations: usize,
    tolerance: f32,
) -> Option<(Pose2d, f32)> {
    let current_scene = scene_points.clone();
    let mut angle = 0.0;
    let mut scale = 1.0;
    let mut tx = 0.0;
    let mut ty = 0.0;

    for _ in 0..max_iterations {
        let mut correspondences = Vec::new();
        for point in &current_scene.0 {
            let closest_point = find_closest_point(*point, model_points);
            correspondences.push((point, closest_point));
        }

        let scene_centroid = calculate_centroid(&current_scene);
        let model_centroid = calculate_centroid(model_points);

        let mut sum_xx = 0.0;
        let mut sum_xy = 0.0;
        let mut sum_yy = 0.0;
        let mut sum_yx = 0.0;

        for (scene_point, model_point) in &correspondences {
            let scene_x = scene_point.x - scene_centroid.x;
            let scene_y = scene_point.y - scene_centroid.y;
            let model_x = model_point.x - model_centroid.x;
            let model_y = model_point.y - model_centroid.y;

            sum_xx += scene_x * model_x;
            sum_xy += scene_x * model_y;
            sum_yy += scene_y * model_y;
            sum_yx += scene_y * model_x;
        }

        angle = (sum_xy - sum_yx).atan2(sum_xx + sum_yy);

        let mut sum_scene_squared = 0.0;
        let mut sum_model_squared = 0.0;

        for (scene_point, model_point) in &correspondences {
            let scene_x = scene_point.x - scene_centroid.x;
            let scene_y = scene_point.y - scene_centroid.y;
            let model_x = model_point.x - model_centroid.x;
            let model_y = model_point.y - model_centroid.y;

            sum_scene_squared += scene_x.powi(2) + scene_y.powi(2);
            sum_model_squared += model_x.powi(2) + model_y.powi(2);
        }

        scale = (sum_model_squared / sum_scene_squared).sqrt();

        tx = model_centroid.x
            - scale * (scene_centroid.x * angle.cos() - scene_centroid.y * angle.sin());
        ty = model_centroid.y
            - scale * (scene_centroid.x * angle.sin() + scene_centroid.y * angle.cos());

        let new_error = calculate_error(&current_scene, model_points);
        if new_error < tolerance {
            break;
        }
    }

    if tx.is_nan() || ty.is_nan() || angle.is_nan() || scale.is_nan() {
        return None;
    }

    Some((Pose2d { x: tx, y: ty, theta: angle }, scale))
}
