# `QMC5883P`

An embedded rust `no_std` driver for the QMC5883P magnetometer chip.

## Usage

Include [library](https://crates.io/crates/qmc5883p) as a dependency in your Cargo.toml:

```
[dependencies.qmc5883p]
version = "<version>"
```

To use the sensor, call `QMC5883P::new` with an embedded-hal i2c device:
```rust
extern crate qmc5883p;

// Create the sensor in soft-reset mode.
let mut dev = QMC5883P::new(i2c_dev).unwrap();
// Enable data collection.
dev.continuous().unwrap();
// Get magnetometer (x,y,z) measurement.
let (x, y, z) = dev.mag().unwrap();
```

## Documentation

API documentation is generated on [docs.rs](https://docs.rs/qmc5883p).

## License

Licensed under AGPL-3.0.

