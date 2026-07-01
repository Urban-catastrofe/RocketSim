pub mod cpp_records;
mod data_reader;
pub mod tick_record;

use std::io::ErrorKind;

use glam::Vec3A;
use rocketsim::consts::TICK_TIME;

use crate::rl_comparison_test::recording::tick_record::TickRecord;
use cpp_records::*;
use data_reader::DataReader;

const RLPR_MAGIC_BYTES: [u8; 4] = [82, 76, 80, 82];
const RLPR_VERSION: u32 = 2;
const RLPR_MAX_CARS: usize = 8;

#[allow(dead_code)]
pub struct Recording {
    pub name: String,
    pub info: RecordingInfo,
    pub ticks: Vec<TickRecord>,
    pub stride: usize,
}

impl Recording {
    pub fn from_bytes(name: &str, bytes: &[u8]) -> Result<Recording, std::io::Error> {
        let mut reader = DataReader::new(bytes);

        for magic_byte in RLPR_MAGIC_BYTES {
            if reader.read_u8()? != magic_byte {
                return Err(std::io::Error::new(
                    ErrorKind::InvalidData,
                    "File is not a valid recording (wrong magic)",
                ));
            }
        }

        let are_we_big_endian = cfg!(target_endian = "big");
        let is_file_big_endian = reader.read_bool()?;
        if is_file_big_endian != are_we_big_endian {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                "File has wrong endianness",
            ));
        }

        let version = reader.read_u32()?;
        if version != RLPR_VERSION {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                format!("RLPR version mismatch (expected: {RLPR_VERSION}, got: {version})"),
            ));
        }

        let info = unsafe { reader.read_struct_unsafe::<RecordingInfo>() }?;
        let num_cars = info.num_cars as usize;
        if num_cars > RLPR_MAX_CARS {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                format!("RLPR recording has too many cars (max: {RLPR_MAX_CARS}, got: {num_cars})"),
            ));
        }

        let num_ticks = reader.read_u32()?;
        let mut ticks = Vec::with_capacity(num_ticks as usize);

        for _ in 0..num_ticks {
            let mut car_records = Vec::with_capacity(num_cars);
            for _ in 0..num_cars {
                let car_record = unsafe { reader.read_struct_unsafe::<CarRecord>() }?;
                car_records.push(car_record);
            }
            let ball_record = unsafe { reader.read_struct_unsafe::<PhysRecord>() }?;
            ticks.push(TickRecord {
                car_records,
                ball_record,
            });
        }

        if reader.num_bytes_left() > 0 {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                format!(
                    "RLPR recording still has {} bytes left after reading all ticks",
                    reader.num_bytes_left()
                ),
            ));
        }

        let stride = Self::detect_stride(&ticks);
        Ok(Self {
            name: name.to_string(),
            info,
            ticks,
            stride,
        })
    }

    /// Detect the recording tick rate relative to the sim's 120 Hz.
    ///
    /// Compares actual position deltas between consecutive ticks against
    /// the expected delta (velocity × TICK_TIME).  Returns the stride
    /// needed to down-sample to 120 Hz equivalent.
    ///
    /// - stride=1 → recording is at 120 Hz (normal)
    /// - stride=2 → recording is at 240 Hz (skip every other tick)
    fn detect_stride(ticks: &[TickRecord]) -> usize {
        let mut ratios: Vec<f32> = Vec::new();
        for i in 0..ticks.len().saturating_sub(1) {
            if ticks[i].car_records.is_empty() {
                continue;
            }
            let vel: Vec3A = ticks[i].car_records[0].phys.lin_vel.into();
            let speed = vel.length();
            if speed < 100.0 {
                continue; // skip stationary / very slow ticks
            }
            let pos_curr: Vec3A = ticks[i].car_records[0].phys.pos.into();
            let pos_next: Vec3A = ticks[i + 1].car_records[0].phys.pos.into();
            let actual_delta = (pos_next - pos_curr).length();
            if actual_delta < 0.01 {
                continue; // skip duplicate frames
            }
            let expected_delta = speed * TICK_TIME;
            ratios.push(expected_delta / actual_delta);
            if ratios.len() >= 60 {
                break;
            }
        }

        if ratios.len() < 5 {
            return 1; // not enough data, assume 120 Hz
        }

        // Use median for robustness against outliers
        ratios.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let median = ratios[ratios.len() / 2];
        let stride = median.round() as usize;

        stride.clamp(1, 8)
    }
}
