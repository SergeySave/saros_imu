
pub struct DataWalker {
    file_data: Vec<u8>,
    position: usize,
}

impl DataWalker {

    pub fn new(file_data: Vec<u8>) -> Self {
        Self {
            file_data,
            position: 0
        }
    }

    pub fn read<T : WalkableType<T>>(&mut self) -> Option<T> {
        if self.file_data.len() < self.position + T::LENGTH {
            return None;
        }
        let result = Some(T::read(&self.file_data[self.position..(self.position + T::LENGTH)]));
        self.position += T::LENGTH;
        result
    }

    pub fn skip(&mut self, bytes: usize) {
        self.position += bytes;
    }

}

pub trait WalkableType<T> {
    const LENGTH: usize;
    fn read(data: &[u8]) -> T;
}

impl WalkableType<u8> for u8 {
    const LENGTH: usize = 1;

    fn read(data: &[u8]) -> u8 {
        data[0]
    }
}

impl WalkableType<u16> for u16 {
    const LENGTH: usize = 2;

    fn read(data: &[u8]) -> u16 {
        u16::from_le_bytes([data[0], data[1]])
    }
}

impl WalkableType<i32> for i32 {
    const LENGTH: usize = 4;

    fn read(data: &[u8]) -> i32 {
        i32::from_le_bytes([data[0], data[1], data[2], data[3]])
    }
}

impl WalkableType<f32> for f32 {
    const LENGTH: usize = 4;

    fn read(data: &[u8]) -> f32 {
        f32::from_le_bytes([data[0], data[1], data[2], data[3]])
    }
}
