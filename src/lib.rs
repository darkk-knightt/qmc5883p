//! A platform agnostic driver to interface with the QMC5883P magnetometer.
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/~0.2

#![deny(missing_docs)]
#![no_std]

extern crate embedded_hal as hal;
use hal::blocking::i2c::{Write, WriteRead};

const I2C_ADDRESS: u8 = 0x2c;

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Copy, Clone)]
#[repr(u8)]
enum Register {
    CHIP_ID = 0,
    DATA_OUT_X_L = 1,
    DATA_OUT_X_H = 2,
    DATA_OUT_Y_L = 3,
    DATA_OUT_Y_H = 4,
    DATA_OUT_Z_L = 5,
    DATA_OUT_Z_H = 6,
    STATUS = 9,
    CONTROL1 = 10,
    CONTROL2 = 11,
    SIGN = 0x29,
}

const STATUS_OVL: u8 = 0b010;
const STATUS_DRDY: u8 = 0b001;

const MODE_CONTINUOUS: u8 = 0b11;
const MODE_NORMAL: u8 = 0b01;

const CTRL2_SOFT_RST: u8 = 0x08;

/// Update frequency; 10Hz recommended for low power consumption.
#[repr(u8)]
pub enum OutputDataRate {
    /// 10Hz update rate.
    Rate10Hz = 0,
    /// 50Hz update rate.
    Rate50Hz = 0b0100,
    /// 100Hz update rate.
    Rate100Hz = 0b1000,
    /// 200Hz update rate.
    Rate200Hz = 0b1100,
}

/// Oversampling ratio; controls bandwidth of internal digital filter.
/// Larger oversampling gets less in-band noise but higher power consumption.
#[repr(u8)]
pub enum OversampleRatio {
    /// Sample Ratio 1
    Ratio1 = 3 << 4,
    /// Sample Ratio 2
    Ratio2 = 2 << 4,
    /// Sample Ratio 3
    Ratio4 = 1 << 4,
    /// Sample Ratio 4
    Ratio8 = 0,
}
/// The downsampling rate controls how many internal samples are averaged together before
/// providing a final reading.
#[repr(u8)]
pub enum DownsamplingRate{
    /// Sample Size 8
    Rate8 = 3 << 6,
    /// Sample Size 4
    Rate4 = 2 << 6,
    /// Sample Size 2
    Rate2 = 1 << 6,
    /// Sample Size 1
    Rate1 = 0,
}
/// Field range of magnetic sensor.
#[repr(u8)]
pub enum FieldRange {
    /// ± 2 gauss
    Range2Gauss = 0b1100,
    /// ± 8 gauss
    Range8Gauss = 0b1000,
    /// ± 12 gauss
    Range12Gauss = 0b0100,
    /// ± 30 gauss
    Range30Gauss = 0b0000,
}

/// QMC5883P Error
#[derive(Debug, Copy, Clone)]
pub enum Error<E> {
    /// CHIP_ID returned invalid value (returned value is argument).
    InvalidDevice(u8),
    /// Read taken from magnetometer before ready.
    NotReady,
    /// Reading overflowed.
    Overflow,
    /// Underlying I2C bus error.
    BusError(E),
}

impl<E> core::convert::From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::BusError(e)
    }
}

/// QMC5883P driver
pub struct QMC5883P<I2C> {
    i2c: I2C,
}

impl<I2C, E> QMC5883P<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
{
    /// Creates a new QMC5883P device from an I2C peripheral; begins with a soft reset.
    pub fn new(i2c: I2C) -> Result<Self, Error<E>> {
        let mut dev = QMC5883P { i2c: i2c };
        let id = dev.read_u8(Register::CHIP_ID)?;
        if id != 0x80 {
            return Err(Error::InvalidDevice(id));
        }
        dev.reset()?;
        Ok(dev)
    }

    /// Soft reset the device.
    pub fn reset(&mut self) -> Result<(), E> {
        self.write_u8(Register::CONTROL2, CTRL2_SOFT_RST)?;
        Ok(())
    }

    /// Set the device field range.
    pub fn set_field_range(&mut self, rng: FieldRange) -> Result<(), E> {
        let ctrl2 = self.read_u8(Register::CONTROL2)?;
        let v = (ctrl2 & !(FieldRange::Range8Gauss as u8)) | (rng as u8);
        self.write_u8(Register::CONTROL2, v)
    }

    /// Set the device oversampling rate.
    pub fn set_oversample(&mut self, osr: OversampleRatio) -> Result<(), E> {
        let ctrl1 = self.read_u8(Register::CONTROL1)?;
        let v = (ctrl1 & !(OversampleRatio::Ratio8 as u8)) | (osr as u8);
        self.write_u8(Register::CONTROL1, v)
    }

    /// Set the device oversampling rate.
    pub fn set_downsample(&mut self, dsr: DownsamplingRate) -> Result<(), E> {
        let ctrl1 = self.read_u8(Register::CONTROL1)?;
        let v = (ctrl1 & !(DownsamplingRate::Rate1 as u8)) | (dsr as u8);
        self.write_u8(Register::CONTROL1, v)
    }

    /// Set the device output data rate.
    pub fn set_output_data_rate(&mut self, odr: OutputDataRate) -> Result<(), E> {
        let ctrl1 = self.read_u8(Register::CONTROL1)?;
        let v = (ctrl1 & !(OutputDataRate::Rate200Hz as u8)) | (odr as u8);
        self.write_u8(Register::CONTROL1, v)
    }
    
    /// Put device in normal mode.
    pub fn normal(&mut self) -> Result<(), E> {
        self.write_u8(Register::SIGN,0x06);
        let ctrl1 = self.read_u8(Register::CONTROL1)?;
        self.write_u8(Register::CONTROL1, (ctrl1 & !MODE_CONTINUOUS) | MODE_NORMAL)
    }

    /// Put device in continous mode.
    pub fn continuous(&mut self) -> Result<(), E> {
        self.write_u8(Register::SIGN,0x06);
        self.write_u8(Register::CONTROL2,0x08);
        let ctrl1 = self.read_u8(Register::CONTROL1)?;
        self.write_u8(Register::CONTROL1, ctrl1 | MODE_CONTINUOUS)
    }

    /// Put device in standby mode.
    pub fn standby(&mut self) -> Result<(), E> {
        let ctrl1 = self.read_u8(Register::CONTROL1)?;
        self.write_u8(Register::CONTROL1, ctrl1 & !MODE_CONTINUOUS)
    }

        /// Read raw (x,y,z) from magnetometer.
    pub fn mag(&mut self) -> Result<(i16, i16, i16), Error<E>> {
        let mut status_buf: &mut [u8; 1] = &mut [0; 1];
        self.i2c
            .write_read(I2C_ADDRESS, &[Register::STATUS as u8], status_buf)?;
        let status = status_buf[0];
        if (status & STATUS_DRDY) == 0 {
            return Err(Error::NotReady);
        } else if (status & STATUS_OVL) != 0 {
            return Err(Error::Overflow);
        }
        let buf: &mut [u8; 6] = &mut [0; 6];
        self.i2c
            .write_read(I2C_ADDRESS, &[Register::DATA_OUT_X_L as u8], buf)?;
        let x = ((buf[1] as i16) << 8) | (buf[0] as i16);
        let y = ((buf[3] as i16) << 8) | (buf[2] as i16);
        let z = ((buf[5] as i16) << 8) | (buf[4] as i16);
        Ok((x, y, z))
    }

    fn read_u8(&mut self, reg: Register) -> Result<u8, E> {
        let buf: &mut [u8; 1] = &mut [0];
        self.i2c.write_read(I2C_ADDRESS, &[reg as u8], buf)?;
        Ok(buf[0])
    }

    fn write_u8(&mut self, reg: Register, v: u8) -> Result<(), E> {
        self.i2c.write(I2C_ADDRESS, &[reg as u8, v])
    }
}
