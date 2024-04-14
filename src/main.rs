use std::error::Error;
use std::fs;
use std::io::Write;

use saros_imu::process_file;

use crate::csv_row::CsvRow;

mod csv_row;

fn main() -> Result<(), Box<dyn Error>> {
    let file_data = fs::read("Cormorant_Cold.DAT").expect("Data file not found");
    let states = process_file(file_data);

    let mut csv_writer = csv::WriterBuilder::new()
        .from_path("output.csv")
        .expect("Couldn't create CSV Writer");

    // First ~250 values are bad
    for state in &states[250..] {
        csv_writer.serialize::<CsvRow>(state.clone().into())?;
    }

    csv_writer.flush()?;
    
    Ok(())
}
