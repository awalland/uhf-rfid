//! UHF RFID reader driver with support for multiple transport backends.
//!
//! # Features
//!
//! - `uart-esp32` - UART transport for ESP32 using esp-idf-svc
//! - `serial` - Serial port transport for desktop using serialport crate
//!
//! # Example
//!
//! ```ignore
//! use uhf_rfid::{UhfRfid, SerialTransport};
//!
//! let transport = SerialTransport::new("/dev/ttyUSB0", 115200)?;
//! let mut rfid = UhfRfid::new(transport);
//!
//! if let Some(tag) = rfid.single_poll()? {
//!     println!("Found tag: {}", tag.epc);
//! }
//! ```

mod reader;
mod transport;
mod types;

#[cfg(feature = "uart-esp32")]
mod uart;

#[cfg(feature = "serial")]
mod serial;

// Re-exports
pub use reader::UhfRfid;
pub use transport::RfidTransport;
pub use types::{TagInfo, UhfError};

#[cfg(feature = "uart-esp32")]
pub use uart::UartTransport;

#[cfg(feature = "serial")]
pub use serial::SerialTransport;

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::RefCell;

    /// Dummy transport for testing protocol logic without hardware
    struct DummyTransport;

    impl RfidTransport for DummyTransport {
        type Error = std::io::Error;

        fn write(&mut self, _data: &[u8]) -> Result<usize, Self::Error> {
            Ok(0)
        }

        fn read(&mut self, _buf: &mut [u8], _timeout_ms: u32) -> Result<usize, Self::Error> {
            Ok(0)
        }

        fn clear_input(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    /// Mock transport that returns predefined responses
    struct MockTransport {
        response: RefCell<Vec<u8>>,
    }

    impl MockTransport {
        fn new(response: Vec<u8>) -> Self {
            Self {
                response: RefCell::new(response),
            }
        }
    }

    impl RfidTransport for MockTransport {
        type Error = std::io::Error;

        fn write(&mut self, _data: &[u8]) -> Result<usize, Self::Error> {
            Ok(_data.len())
        }

        fn read(&mut self, buf: &mut [u8], _timeout_ms: u32) -> Result<usize, Self::Error> {
            let response = self.response.borrow();
            let len = response.len().min(buf.len());
            buf[..len].copy_from_slice(&response[..len]);
            Ok(len)
        }

        fn clear_input(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    // ===================
    // create_command tests
    // ===================

    #[test]
    fn test_create_command_with_param() {
        let result = UhfRfid::<DummyTransport>::create_command(0x03, &[0x01]);
        assert_eq!(result, [0xBB, 0x00, 0x03, 0x00, 0x01, 0x01, 0x05, 0x7E]);
    }

    #[test]
    fn test_create_command_no_param() {
        let result = UhfRfid::<DummyTransport>::create_command(0x22, &[]);
        assert_eq!(result, [0xBB, 0x00, 0x22, 0x00, 0x00, 0x22, 0x7E]);
    }

    #[test]
    fn test_create_multiple_poll_command() {
        // Multiple poll with count 10,000 (0x2710)
        let result = UhfRfid::<DummyTransport>::create_command(0x27, &[0x22, 0x27, 0x10]);
        assert_eq!(result, [0xBB, 0x00, 0x27, 0x00, 0x03, 0x22, 0x27, 0x10, 0x83, 0x7E]);
    }

    #[test]
    fn test_create_get_tx_power_command() {
        let result = UhfRfid::<DummyTransport>::create_command(0xB7, &[]);
        assert_eq!(result, [0xBB, 0x00, 0xB7, 0x00, 0x00, 0xB7, 0x7E]);
    }

    #[test]
    fn test_create_set_tx_power_command() {
        // Set power to 20 dBm (2000 = 0x07D0)
        let result = UhfRfid::<DummyTransport>::create_command(0xB6, &[0x07, 0xD0]);
        assert_eq!(result, [0xBB, 0x00, 0xB6, 0x00, 0x02, 0x07, 0xD0, 0x8F, 0x7E]);
    }

    // ===================
    // get_firmware_version tests
    // ===================

    #[test]
    fn test_get_firmware_version_valid() {
        // Response: BB 01 03 00 0A "V1.0.0" checksum 7E
        let response = vec![0xBB, 0x01, 0x03, 0x00, 0x0A, 0x00, b'V', b'1', b'.', b'0', b'.', b'0', 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let version = rfid.get_firmware_version().unwrap();
        assert_eq!(version, "V1.0.0");
    }

    #[test]
    fn test_get_firmware_version_invalid_header() {
        let response = vec![0xAA, 0x01, 0x03, 0x00, 0x02, 0x00, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(matches!(rfid.get_firmware_version(), Err(UhfError::InvalidResponse(_))));
    }

    #[test]
    fn test_get_firmware_version_too_short() {
        let response = vec![0xBB, 0x01, 0x03];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(matches!(rfid.get_firmware_version(), Err(UhfError::InvalidResponse(_))));
    }

    // ===================
    // single_poll tests
    // ===================

    #[test]
    fn test_single_poll_tag_found() {
        // Tag response with 12-byte EPC
        // BB 02 22 00 11 C8 00 00 E2 00 68 16 00 00 00 60 12 34 56 78 checksum 7E
        let response = vec![
            0xBB, 0x02, 0x22, 0x00, 0x11, // header, type, cmd, len MSB, len LSB (17 bytes)
            0xC8, // RSSI = 200
            0x00, 0x00, // PC
            0xE2, 0x00, 0x68, 0x16, 0x00, 0x00, 0x00, 0x60, 0x12, 0x34, 0x56, 0x78, // 12-byte EPC
            0x00, 0x7E, // checksum, end
        ];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let tag = rfid.single_poll().unwrap();
        assert!(tag.is_some());
        let tag = tag.unwrap();
        assert_eq!(tag.rssi, 0xC8);
        assert_eq!(tag.epc, "E20068160000006012345678");
    }

    #[test]
    fn test_single_poll_no_tag() {
        // Notification response (no tag found)
        let response = vec![0xBB, 0x01, 0x22, 0x00, 0x01, 0x00, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let tag = rfid.single_poll().unwrap();
        assert!(tag.is_none());
    }

    #[test]
    fn test_single_poll_response_too_short() {
        let response = vec![0xBB, 0x02, 0x22, 0x00];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let tag = rfid.single_poll().unwrap();
        assert!(tag.is_none());
    }

    #[test]
    fn test_single_poll_invalid_header() {
        let response = vec![0xAA, 0x02, 0x22, 0x00, 0x11, 0xC8, 0x00, 0x00, 0xE2, 0x00, 0x68, 0x16, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(matches!(rfid.single_poll(), Err(UhfError::InvalidResponse(_))));
    }

    // ===================
    // get_tx_power tests
    // ===================

    #[test]
    fn test_get_tx_power_valid() {
        // Response for 20 dBm (2000 = 0x07D0)
        let response = vec![0xBB, 0x01, 0xB7, 0x00, 0x02, 0x07, 0xD0, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let power = rfid.get_tx_power().unwrap();
        assert_eq!(power, 20);
    }

    #[test]
    fn test_get_tx_power_26dbm() {
        // Response for 26 dBm (2600 = 0x0A28)
        let response = vec![0xBB, 0x01, 0xB7, 0x00, 0x02, 0x0A, 0x28, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let power = rfid.get_tx_power().unwrap();
        assert_eq!(power, 26);
    }

    #[test]
    fn test_get_tx_power_invalid_response() {
        // Wrong command byte
        let response = vec![0xBB, 0x01, 0xB6, 0x00, 0x02, 0x07, 0xD0, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(matches!(rfid.get_tx_power(), Err(UhfError::InvalidResponse(_))));
    }

    #[test]
    fn test_get_tx_power_too_short() {
        let response = vec![0xBB, 0x01, 0xB7, 0x00];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(matches!(rfid.get_tx_power(), Err(UhfError::InvalidResponse(_))));
    }

    // ===================
    // set_tx_power tests
    // ===================

    #[test]
    fn test_set_tx_power_valid() {
        // Success response
        let response = vec![0xBB, 0x01, 0xB6, 0x00, 0x01, 0x00, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(rfid.set_tx_power(20).is_ok());
    }

    #[test]
    fn test_set_tx_power_min_valid() {
        let response = vec![0xBB, 0x01, 0xB6, 0x00, 0x01, 0x00, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(rfid.set_tx_power(18).is_ok());
    }

    #[test]
    fn test_set_tx_power_max_valid() {
        let response = vec![0xBB, 0x01, 0xB6, 0x00, 0x01, 0x00, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(rfid.set_tx_power(26).is_ok());
    }

    #[test]
    fn test_set_tx_power_too_low() {
        let transport = MockTransport::new(vec![]);
        let mut rfid = UhfRfid::new(transport);

        let result = rfid.set_tx_power(17);
        assert!(matches!(result, Err(UhfError::InvalidParameter(_))));
    }

    #[test]
    fn test_set_tx_power_too_high() {
        let transport = MockTransport::new(vec![]);
        let mut rfid = UhfRfid::new(transport);

        let result = rfid.set_tx_power(27);
        assert!(matches!(result, Err(UhfError::InvalidParameter(_))));
    }

    #[test]
    fn test_set_tx_power_device_error() {
        // Error response (non-zero status)
        let response = vec![0xBB, 0x01, 0xB6, 0x00, 0x01, 0x01, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(matches!(rfid.set_tx_power(20), Err(UhfError::InvalidResponse(_))));
    }

    // ===================
    // multiple_poll tests
    // ===================

    #[test]
    fn test_multiple_poll_zero_count_error() {
        let transport = MockTransport::new(vec![]);
        let mut rfid = UhfRfid::new(transport);

        let result = rfid.multiple_poll(0);
        assert!(matches!(result, Err(UhfError::InvalidParameter(_))));
    }

    // ===================
    // bytes_to_hex tests
    // ===================

    #[test]
    fn test_bytes_to_hex() {
        use types::bytes_to_hex;
        assert_eq!(bytes_to_hex(&[0xDE, 0xAD, 0xBE, 0xEF]), "DEADBEEF");
        assert_eq!(bytes_to_hex(&[0x00, 0x01, 0x0A, 0xFF]), "00010AFF");
        assert_eq!(bytes_to_hex(&[]), "");
    }

    // ===================
    // TagInfo tests
    // ===================

    #[test]
    fn test_tag_info_equality() {
        let tag1 = TagInfo {
            epc: "E200".to_string(),
            rssi: 100,
        };
        let tag2 = TagInfo {
            epc: "E200".to_string(),
            rssi: 50, // Different RSSI
        };
        let tag3 = TagInfo {
            epc: "E300".to_string(),
            rssi: 100,
        };

        assert_eq!(tag1, tag2); // Same EPC, different RSSI -> equal
        assert_ne!(tag1, tag3); // Different EPC -> not equal
    }
}
