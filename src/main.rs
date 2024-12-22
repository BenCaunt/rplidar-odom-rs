extern crate rplidar_drv;
extern crate serialport;
use std::time::Duration;

use rplidar_drv::{RplidarDevice, ScanPoint};
use rerun::{demo_util::grid, external::glam, Points2D, Position2D};


fn main() -> ! {
    let rec = rerun::RecordingStreamBuilder::new("lidar_odom").connect_tcp().unwrap();


    let ports = serialport::available_ports().expect("No ports found!");
    for p in ports {
        println!("{}", p.port_name);
    }

    let port = serialport::new("/dev/tty.usbserial-0001", 115_200)
    .timeout(Duration::from_millis(10))
    .open().expect("Failed to open port");

    let mut rplidar = RplidarDevice::with_stream(port);

    let device_info = rplidar.get_device_info().unwrap();
    rplidar.start_scan().unwrap();

    loop {
        let scan = rplidar.grab_scan();

        if let Ok(scan) = scan {
            for point in scan {
                println!("angle: {:?}", point.angle());
                println!("distance: {:?}", point.distance()); 
            }
        }

    }

}


fn scan_point_to_rerun(points: Vec<ScanPoint>) -> Points2D {
    Points2D::new(points.into_iter().map(|p| Position2D::new(p.angle(), p.distance())))
}