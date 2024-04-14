use std::ops::Sub;
use std::time::Duration;

#[derive(Debug, Copy, Clone)]
pub struct Timestamp {
    pub year: u8,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub minutes: u8,
    pub seconds: u8,
    pub milliseconds: u16,
}

impl Sub for Timestamp {
    type Output = Duration;

    fn sub(self, rhs: Self) -> Self::Output {
        if self.year != rhs.year {
            panic!("Year mismatch");
        }
        if self.month != rhs.month {
            panic!("Month mismatch");
        }

        let a: u64 = (self.milliseconds as u64)
            + (self.seconds as u64) * 1000
            + (self.minutes as u64) * 1000 * 60
            + (self.hour as u64) * 1000 * 60 * 60
            + (self.day as u64) * 1000 * 60 * 60 * 24;

        let b: u64 = (rhs.milliseconds as u64)
            + (rhs.seconds as u64) * 1000
            + (rhs.minutes as u64) * 1000 * 60
            + (rhs.hour as u64) * 1000 * 60 * 60
            + (rhs.day as u64) * 1000 * 60 * 60 * 24;

        Duration::from_millis(
            a - b
        )
    }
}
