[![codecov](https://codecov.io/github/awalland/uhf-rfid/graph/badge.svg?token=FSE4ZYPOIE)](https://codecov.io/github/awalland/uhf-rfid)

# uhf-rfid

A Rust driver for M5Stack UHF RFID readers with support for multiple transport backends.

## Features

- **ESP32 UART support** - Native UART transport for ESP32 using `esp-idf-hal`
- **Desktop serial support** - Serial port transport using the `serialport` crate
- **Full EPC Gen2 support** - Tag polling, reading, writing, locking, and killing
- **Advanced configuration** - Region settings, RF link profiles, frequency hopping, and more
- **Vendor extensions** - NXP UCODE and Impinj Monza specific commands

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
uhf-rfid = "0.1"
```

Enable the appropriate feature for your platform:

```toml
# For desktop/Linux serial port
uhf-rfid = { version = "0.1", features = ["serial"] }

# For ESP32
uhf-rfid = { version = "0.1", features = ["uart-esp32"] }
```

## Usage

### Desktop (Serial Port)

```rust
use uhf_rfid::{UhfRfid, SerialTransport};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let transport = SerialTransport::new("/dev/ttyUSB0", 115200)?;
    let mut rfid = UhfRfid::new(transport);

    // Get firmware version
    let version = rfid.get_firmware_version()?;
    println!("Firmware: {}", version);

    // Poll for a single tag
    if let Some(tag) = rfid.single_poll()? {
        println!("Found tag: {} (RSSI: {})", tag.epc, tag.rssi);
    }

    // Poll for multiple tags
    let tags = rfid.multiple_poll(100)?;
    for tag in tags {
        println!("Tag: {}", tag.epc);
    }

    Ok(())
}
```

### ESP32 (UART)

```rust
use esp_idf_svc::hal::peripherals::Peripherals;
use uhf_rfid::{UhfRfid, UartTransport};

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();

    let peripherals = Peripherals::take()?;

    let transport = UartTransport::new(
        peripherals.uart1,
        peripherals.pins.gpio17, // TX
        peripherals.pins.gpio16, // RX
        115200,
    )?;

    let mut rfid = UhfRfid::new(transport);

    // Get firmware version
    let version = rfid.get_firmware_version()?;
    println!("Firmware: {}", version);

    // Poll for tags
    if let Some(tag) = rfid.single_poll()? {
        println!("Found tag: {} (RSSI: {})", tag.epc, tag.rssi);
    }

    Ok(())
}
```

## Supported Operations

- **Polling**: Single and multiple tag inventory
- **Memory access**: Read/write tag memory banks (EPC, TID, User, Reserved)
- **Security**: Lock and kill tags
- **Configuration**: TX power, region, channel, frequency hopping, baud rate
- **Advanced**: Select filtering, query parameters, RF link profiles
- **Vendor-specific**: NXP EAS, read protect; Impinj Monza QT

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.
