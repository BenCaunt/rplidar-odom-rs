extern crate rplidar_drv;
extern crate serialport;
use std::time::Duration;

use rplidar_drv::{RplidarDevice, ScanPoint};
use rerun::{Points2D, Position2D};
mod icp;
use icp::{PointCloud, Pose2d, icp};
use serde::Serialize;

#[tokio::main]
async fn main() -> ! {
    // let rec = rerun::RecordingStreamBuilder::new("lidar_odom").spawn().unwrap();
    let session = zenoh::open(zenoh::Config::default()).await.unwrap();


    let ports = serialport::available_ports().expect("No ports found!");
    for p in ports {
        println!("{}", p.port_name);
    }

    let port = serialport::new("/dev/ttyUSB0", 115_200)
        .timeout(Duration::from_millis(10))
        .open().expect("Failed to open port");

    let mut rplidar = RplidarDevice::with_stream(port);
    rplidar.start_scan().unwrap();


    loop {
        let scan = rplidar.grab_scan();
        if let Ok(scan) = scan {
            let new_cloud = PointCloud::from(&scan);
            if let Ok(new_cloud) = serde_json::to_string(&new_cloud) {
                println!("New cloud");
                session.put("lidar/scan", new_cloud).await.unwrap();
            } else {
                println!("Failed to serialize new cloud");
            }
        }
    }

}


fn scan_point_to_rerun(points: &Vec<ScanPoint>) -> Points2D {
    Points2D::new(points.iter().map(|p| Position2D::new(p.angle().cos() * p.distance(), p.angle().sin() * p.distance())))
}