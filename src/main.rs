use std::error::Error;
use std::fs;

use saros_imu::process_file;
use saros_imu::state::ExtraOutput;

use crate::csv_row::CsvRow;

mod csv_row;

fn main() -> Result<(), Box<dyn Error>> {
    let file_data = fs::read("Cormorant_Cold.DAT").expect("Data file not found");
    let (states, positions) = process_file(file_data);

    let mut csv_writer = csv::WriterBuilder::new()
        .from_path("output_flight.csv")
        .expect("Couldn't create CSV Writer");

    // let mut csv_writer2 = csv::WriterBuilder::new()
    //     .from_path("output_positions.csv")
    //     .expect("Couldn't create CSV Writer");

    for state in &states[0..15000] {
        csv_writer.serialize::<CsvRow>(state.clone().into())?;
    }
    // for position in positions {
    //     if position.time > 15010.00 {
    //         break;
    //     }
    //     csv_writer2.serialize::<ExtraOutput>(position.clone())?;
    // }

    csv_writer.flush()?;
    // csv_writer2.flush()?;
    
    Ok(())
}

//  -54137
// -121495
