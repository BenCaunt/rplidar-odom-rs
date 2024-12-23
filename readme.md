# Zenoh Rplidar publisher 

publishes the rplidar scan data to zenoh topic `lidar/scan`

## for compilation on raspberry pi from arm64 mac os


```
rustup target add aarch64-unknown-linux-gnu
brew tap messense/macos-cross-toolchains
brew install aarch64-unknown-linux-gnu
cargo build --release --target aarch64-unknown-linux-gnu
```