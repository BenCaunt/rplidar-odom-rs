extern crate rplidar_drv;
extern crate serialport;
use std::time::Duration;

use rplidar_drv::{RplidarDevice, ScanPoint};
use rerun::{demo_util::grid, external::glam, Points2D, Position2D};


fn main() -> ! {
    let rec = rerun::RecordingStreamBuilder::new("lidar_odom").spawn().unwrap();


    let ports = serialport::available_ports().expect("No ports found!");
    for p in ports {
        println!("{}", p.port_name);
    }

    let port = serialport::new("/dev/tty.usbserial-0001", 115_200)
    .timeout(Duration::from_millis(10))
    .open().expect("Failed to open port");

    let mut rplidar = RplidarDevice::with_stream(port);
    rplidar.start_scan().unwrap();

    loop {
        let scan = rplidar.grab_scan();

        if let Ok(scan) = scan {
            let rr_points: Points2D = scan_point_to_rerun(scan);
            rec.log("scan_points", &rr_points).unwrap();
        }

    }

}


fn scan_point_to_rerun(points: Vec<ScanPoint>) -> Points2D {
    Points2D::new(points.into_iter().map(|p| Position2D::new(p.angle().cos() * p.distance(), p.angle().sin() * p.distance())))
}
