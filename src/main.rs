use std::fs;
use saros_imu::process_file;

fn main() {
    let file_data = fs::read("Cormorant_Cold.DAT").expect("Data file not found");
    let states = process_file(file_data);
    println!("{:?}", states);
}
