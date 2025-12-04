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
pub use types::{
    LockAction, LockPayload, LockTarget, MemoryBank, QtControl, QueryParams, QuerySel,
    QuerySession, QueryTarget, Region, RfLinkProfile, SelectAction, SelectMode, SelectParams,
    SelectTarget, TagInfo, UhfError,
};

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
    // poll_for_duration tests
    // ===================

    /// Mock transport that returns multiple responses in sequence for poll_for_duration tests
    struct MultiResponseMockTransport {
        responses: RefCell<Vec<Vec<u8>>>,
        read_count: RefCell<usize>,
    }

    impl MultiResponseMockTransport {
        fn new(responses: Vec<Vec<u8>>) -> Self {
            Self {
                responses: RefCell::new(responses),
                read_count: RefCell::new(0),
            }
        }
    }

    impl RfidTransport for MultiResponseMockTransport {
        type Error = std::io::Error;

        fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error> {
            Ok(data.len())
        }

        fn read(&mut self, buf: &mut [u8], _timeout_ms: u32) -> Result<usize, Self::Error> {
            let responses = self.responses.borrow();
            let mut count = self.read_count.borrow_mut();

            if *count >= responses.len() {
                return Ok(0);
            }

            let response = &responses[*count];
            let len = response.len().min(buf.len());
            buf[..len].copy_from_slice(&response[..len]);
            *count += 1;
            Ok(len)
        }

        fn clear_input(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    #[test]
    fn test_create_poll_for_duration_command() {
        // poll_for_duration uses 0xFFFF count for continuous polling
        let result = UhfRfid::<DummyTransport>::create_command(0x27, &[0x22, 0xFF, 0xFF]);
        assert_eq!(result[0], 0xBB);
        assert_eq!(result[2], 0x27); // MULTIPLE_POLL command
        assert_eq!(result[5], 0x22);
        assert_eq!(result[6], 0xFF); // count MSB
        assert_eq!(result[7], 0xFF); // count LSB
        assert_eq!(*result.last().unwrap(), 0x7E);
    }

    #[test]
    fn test_poll_for_duration_with_tags() {
        use std::time::Duration;

        // Simulate two tag responses followed by empty reads
        let tag1_response = vec![
            0xBB, 0x02, 0x22, 0x00, 0x11, // header
            0xC8, // RSSI = 200
            0x30, 0x00, // PC
            0xE2, 0x00, 0x00, 0x17, 0x22, 0x09, 0x01, 0x23, 0x19, 0x10, 0x01, 0x23, // EPC
            0x00, 0x7E, // checksum, end
        ];
        let tag2_response = vec![
            0xBB, 0x02, 0x22, 0x00, 0x11,
            0xB4, // RSSI = 180
            0x30, 0x00,
            0xE2, 0x00, 0x00, 0x17, 0x22, 0x09, 0x01, 0x23, 0x19, 0x10, 0x01, 0x24, // Different EPC
            0x00, 0x7E,
        ];

        let transport = MultiResponseMockTransport::new(vec![tag1_response, tag2_response]);
        let mut rfid = UhfRfid::new(transport);

        // Use a very short timeout since we're mocking
        let tags = rfid.poll_for_duration(Duration::from_millis(50)).unwrap();

        assert_eq!(tags.len(), 2);
        assert_eq!(tags[0].rssi, 0xC8);
        assert_eq!(tags[1].rssi, 0xB4);
    }

    #[test]
    fn test_poll_for_duration_with_callback() {
        use std::time::Duration;

        let tag_response = vec![
            0xBB, 0x02, 0x22, 0x00, 0x11,
            0xC8,
            0x30, 0x00,
            0xE2, 0x00, 0x00, 0x17, 0x22, 0x09, 0x01, 0x23, 0x19, 0x10, 0x01, 0x23,
            0x00, 0x7E,
        ];

        let transport = MultiResponseMockTransport::new(vec![tag_response]);
        let mut rfid = UhfRfid::new(transport);

        let mut callback_count = 0;
        let count = rfid
            .poll_for_duration_with_callback(Duration::from_millis(50), |_tag| {
                callback_count += 1;
            })
            .unwrap();

        assert_eq!(count, 1);
        assert_eq!(callback_count, 1);
    }

    #[test]
    fn test_poll_for_duration_empty() {
        use std::time::Duration;

        // No tags found
        let transport = MultiResponseMockTransport::new(vec![]);
        let mut rfid = UhfRfid::new(transport);

        let tags = rfid.poll_for_duration(Duration::from_millis(50)).unwrap();
        assert!(tags.is_empty());
    }

    #[test]
    fn test_poll_for_duration_restarts_on_end_notification() {
        use std::time::Duration;

        // End-of-poll notification followed by a tag (simulating restart)
        let end_notification = vec![
            0xBB, 0x01, 0xFF, 0x00, 0x01, 0x15, 0x00, 0x7E,
        ];
        let tag_response = vec![
            0xBB, 0x02, 0x22, 0x00, 0x11,
            0xC8,
            0x30, 0x00,
            0xE2, 0x00, 0x00, 0x17, 0x22, 0x09, 0x01, 0x23, 0x19, 0x10, 0x01, 0x23,
            0x00, 0x7E,
        ];

        let transport = MultiResponseMockTransport::new(vec![end_notification, tag_response]);
        let mut rfid = UhfRfid::new(transport);

        // Should continue polling after end notification and find the tag
        let tags = rfid.poll_for_duration(Duration::from_millis(50)).unwrap();
        assert_eq!(tags.len(), 1);
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

    // ===================
    // stop_multiple_poll tests
    // ===================

    #[test]
    fn test_create_stop_multiple_poll_command() {
        let result = UhfRfid::<DummyTransport>::create_command(0x28, &[]);
        assert_eq!(result, [0xBB, 0x00, 0x28, 0x00, 0x00, 0x28, 0x7E]);
    }

    #[test]
    fn test_stop_multiple_poll_valid() {
        // Success response per protocol
        let response = vec![0xBB, 0x01, 0x28, 0x00, 0x01, 0x00, 0x2A, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(rfid.stop_multiple_poll().is_ok());
    }

    #[test]
    fn test_stop_multiple_poll_invalid_response() {
        let response = vec![0xBB, 0x01, 0x28, 0x00, 0x01, 0x01, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(matches!(rfid.stop_multiple_poll(), Err(UhfError::InvalidResponse(_))));
    }

    // ===================
    // set_select_param tests
    // ===================

    #[test]
    fn test_create_set_select_param_command() {
        // Example from protocol: Target=000, Action=000, MemBank=01 -> 0x01
        // Ptr=0x00000020, MaskLen=0x60 (96 bits = 12 bytes), Truncate=0x00
        // Mask: 0x30751FEB705C5904E3D50D70
        let result = UhfRfid::<DummyTransport>::create_command(
            0x0C,
            &[
                0x01, // SelParam
                0x00, 0x00, 0x00, 0x20, // Ptr
                0x60, // MaskLen (96 bits)
                0x00, // Truncate disabled
                0x30, 0x75, 0x1F, 0xEB, 0x70, 0x5C, 0x59, 0x04, 0xE3, 0xD5, 0x0D, 0x70, // Mask
            ],
        );
        // Verify header and end markers
        assert_eq!(result[0], 0xBB);
        assert_eq!(result[1], 0x00);
        assert_eq!(result[2], 0x0C);
        assert_eq!(*result.last().unwrap(), 0x7E);
    }

    #[test]
    fn test_set_select_param_valid() {
        use types::{MemoryBank, SelectAction, SelectParams, SelectTarget};

        let response = vec![0xBB, 0x01, 0x0C, 0x00, 0x01, 0x00, 0x0E, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let params = SelectParams {
            target: SelectTarget::S0,
            action: SelectAction::Action0,
            mem_bank: MemoryBank::Epc,
            pointer: 0x20,
            mask: vec![0x30, 0x75, 0x1F, 0xEB],
            truncate: false,
        };

        assert!(rfid.set_select_param(&params).is_ok());
    }

    #[test]
    fn test_set_select_param_mask_too_long() {
        use types::{MemoryBank, SelectAction, SelectParams, SelectTarget};

        let transport = MockTransport::new(vec![]);
        let mut rfid = UhfRfid::new(transport);

        let params = SelectParams {
            target: SelectTarget::S0,
            action: SelectAction::Action0,
            mem_bank: MemoryBank::Epc,
            pointer: 0x20,
            mask: vec![0u8; 33], // Too long
            truncate: false,
        };

        assert!(matches!(rfid.set_select_param(&params), Err(UhfError::InvalidParameter(_))));
    }

    // ===================
    // get_select_param tests
    // ===================

    #[test]
    fn test_create_get_select_param_command() {
        let result = UhfRfid::<DummyTransport>::create_command(0x0B, &[]);
        assert_eq!(result, [0xBB, 0x00, 0x0B, 0x00, 0x00, 0x0B, 0x7E]);
    }

    #[test]
    fn test_get_select_param_valid() {
        use types::{MemoryBank, SelectAction, SelectTarget};

        // Response with 4-byte mask
        let response = vec![
            0xBB, 0x01, 0x0B, 0x00, 0x0B, // header, type, cmd, len MSB, len LSB
            0x01, // SelParam: Target=0, Action=0, MemBank=1 (EPC)
            0x00, 0x00, 0x00, 0x20, // Ptr = 32
            0x20, // MaskLen = 32 bits (4 bytes)
            0x00, // Truncate disabled
            0xDE, 0xAD, 0xBE, 0xEF, // Mask
            0x00, 0x7E, // checksum, end
        ];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let params = rfid.get_select_param().unwrap();
        assert_eq!(params.target, SelectTarget::S0);
        assert_eq!(params.action, SelectAction::Action0);
        assert_eq!(params.mem_bank, MemoryBank::Epc);
        assert_eq!(params.pointer, 0x20);
        assert_eq!(params.mask, vec![0xDE, 0xAD, 0xBE, 0xEF]);
        assert!(!params.truncate);
    }

    #[test]
    fn test_get_select_param_invalid_header() {
        let response = vec![0xAA, 0x01, 0x0B, 0x00, 0x0B, 0x01, 0x00, 0x00, 0x00, 0x20, 0x20, 0x00, 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(matches!(rfid.get_select_param(), Err(UhfError::InvalidResponse(_))));
    }

    // ===================
    // set_select_mode tests
    // ===================

    #[test]
    fn test_create_set_select_mode_command() {
        // Disable select mode (0x01)
        let result = UhfRfid::<DummyTransport>::create_command(0x12, &[0x01]);
        assert_eq!(result, [0xBB, 0x00, 0x12, 0x00, 0x01, 0x01, 0x14, 0x7E]);
    }

    #[test]
    fn test_set_select_mode_valid() {
        // Response per protocol (uses 0x0C command in response)
        let response = vec![0xBB, 0x01, 0x0C, 0x00, 0x01, 0x00, 0x0E, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(rfid.set_select_mode(SelectMode::Disabled).is_ok());
    }

    #[test]
    fn test_set_select_mode_always() {
        let response = vec![0xBB, 0x01, 0x0C, 0x00, 0x01, 0x00, 0x0E, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(rfid.set_select_mode(SelectMode::Always).is_ok());
    }

    #[test]
    fn test_set_select_mode_non_polling() {
        let response = vec![0xBB, 0x01, 0x0C, 0x00, 0x01, 0x00, 0x0E, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(rfid.set_select_mode(SelectMode::NonPolling).is_ok());
    }

    #[test]
    fn test_set_select_mode_invalid_response() {
        let response = vec![0xBB, 0x01, 0x0C, 0x00, 0x01, 0x01, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(matches!(rfid.set_select_mode(SelectMode::Disabled), Err(UhfError::InvalidResponse(_))));
    }

    // ===================
    // Phase 2: Query Parameters tests
    // ===================

    #[test]
    fn test_query_params_to_bytes() {
        use types::{QueryParams, QuerySel, QuerySession, QueryTarget};

        let params = QueryParams {
            sel: QuerySel::All,
            session: QuerySession::S0,
            target: QueryTarget::A,
            q: 4,
        };
        let bytes = params.to_bytes();
        // DR=0, M=0, TRext=1, Sel=00, Session=00, Target=0, Q=4
        assert_eq!(bytes[0], 0x10); // 0001 0000
        assert_eq!(bytes[1], 0x20); // 0 0100 000 (target=0, q=4, padding=0)
    }

    #[test]
    fn test_query_params_from_bytes() {
        use types::{QueryParams, QuerySel, QuerySession, QueryTarget};

        let params = QueryParams::from_bytes([0x10, 0x20]);
        assert_eq!(params.sel, QuerySel::All);
        assert_eq!(params.session, QuerySession::S0);
        assert_eq!(params.target, QueryTarget::A);
        assert_eq!(params.q, 4);
    }

    #[test]
    fn test_query_params_roundtrip() {
        use types::{QueryParams, QuerySel, QuerySession, QueryTarget};

        let original = QueryParams {
            sel: QuerySel::Sl,
            session: QuerySession::S2,
            target: QueryTarget::B,
            q: 8,
        };
        let bytes = original.to_bytes();
        let restored = QueryParams::from_bytes(bytes);

        assert_eq!(restored.sel, original.sel);
        assert_eq!(restored.session, original.session);
        assert_eq!(restored.target, original.target);
        assert_eq!(restored.q, original.q);
    }

    #[test]
    fn test_create_get_query_param_command() {
        let result = UhfRfid::<DummyTransport>::create_command(0x0D, &[]);
        assert_eq!(result, [0xBB, 0x00, 0x0D, 0x00, 0x00, 0x0D, 0x7E]);
    }

    #[test]
    fn test_get_query_param_valid() {
        use types::{QuerySel, QuerySession, QueryTarget};

        // Response: BB 01 0D 00 02 10 20 checksum 7E
        let response = vec![0xBB, 0x01, 0x0D, 0x00, 0x02, 0x10, 0x20, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let params = rfid.get_query_param().unwrap();
        assert_eq!(params.sel, QuerySel::All);
        assert_eq!(params.session, QuerySession::S0);
        assert_eq!(params.target, QueryTarget::A);
        assert_eq!(params.q, 4);
    }

    #[test]
    fn test_get_query_param_invalid_response() {
        let response = vec![0xBB, 0x01, 0xAA, 0x00, 0x02, 0x10, 0x20, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(matches!(rfid.get_query_param(), Err(UhfError::InvalidResponse(_))));
    }

    #[test]
    fn test_create_set_query_param_command() {
        let result = UhfRfid::<DummyTransport>::create_command(0x0E, &[0x10, 0x20]);
        assert_eq!(result, [0xBB, 0x00, 0x0E, 0x00, 0x02, 0x10, 0x20, 0x40, 0x7E]);
    }

    #[test]
    fn test_set_query_param_valid() {
        use types::{QueryParams, QuerySel, QuerySession, QueryTarget};

        let response = vec![0xBB, 0x01, 0x0E, 0x00, 0x01, 0x00, 0x10, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let params = QueryParams {
            sel: QuerySel::All,
            session: QuerySession::S0,
            target: QueryTarget::A,
            q: 4,
        };

        assert!(rfid.set_query_param(&params).is_ok());
    }

    #[test]
    fn test_set_query_param_invalid_q() {
        use types::{QueryParams, QuerySel, QuerySession, QueryTarget};

        let transport = MockTransport::new(vec![]);
        let mut rfid = UhfRfid::new(transport);

        let params = QueryParams {
            sel: QuerySel::All,
            session: QuerySession::S0,
            target: QueryTarget::A,
            q: 16, // Invalid: max is 15
        };

        assert!(matches!(rfid.set_query_param(&params), Err(UhfError::InvalidParameter(_))));
    }

    // ===================
    // Phase 2: Region tests
    // ===================

    #[test]
    fn test_region_frequencies() {
        assert_eq!(Region::Us.base_frequency(), 902.25);
        assert_eq!(Region::Europe.base_frequency(), 865.1);
        assert_eq!(Region::China900.base_frequency(), 920.125);
    }

    #[test]
    fn test_region_channel_spacing() {
        assert_eq!(Region::Us.channel_spacing(), 0.5);
        assert_eq!(Region::Europe.channel_spacing(), 0.2);
    }

    #[test]
    fn test_region_frequency_from_channel() {
        // US: 902.25 + (10 * 0.5) = 907.25 MHz
        assert_eq!(Region::Us.frequency_from_channel(10), 907.25);
    }

    #[test]
    fn test_region_channel_from_frequency() {
        // US: (907.25 - 902.25) / 0.5 = 10
        assert_eq!(Region::Us.channel_from_frequency(907.25), 10);
    }

    #[test]
    fn test_region_try_from() {
        assert_eq!(Region::try_from(0x02).unwrap(), Region::Us);
        assert_eq!(Region::try_from(0x03).unwrap(), Region::Europe);
        assert!(Region::try_from(0xFF).is_err());
    }

    #[test]
    fn test_create_get_region_command() {
        let result = UhfRfid::<DummyTransport>::create_command(0x08, &[]);
        assert_eq!(result, [0xBB, 0x00, 0x08, 0x00, 0x00, 0x08, 0x7E]);
    }

    #[test]
    fn test_get_region_valid() {
        // Response: US region (0x02)
        let response = vec![0xBB, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let region = rfid.get_region().unwrap();
        assert_eq!(region, Region::Us);
    }

    #[test]
    fn test_get_region_europe() {
        let response = vec![0xBB, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let region = rfid.get_region().unwrap();
        assert_eq!(region, Region::Europe);
    }

    #[test]
    fn test_get_region_invalid_code() {
        let response = vec![0xBB, 0x01, 0x08, 0x00, 0x01, 0xFF, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(matches!(rfid.get_region(), Err(UhfError::InvalidResponse(_))));
    }

    #[test]
    fn test_create_set_region_command() {
        let result = UhfRfid::<DummyTransport>::create_command(0x07, &[0x02]);
        assert_eq!(result, [0xBB, 0x00, 0x07, 0x00, 0x01, 0x02, 0x0A, 0x7E]);
    }

    #[test]
    fn test_set_region_valid() {
        let response = vec![0xBB, 0x01, 0x07, 0x00, 0x01, 0x00, 0x09, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(rfid.set_region(Region::Us).is_ok());
    }

    #[test]
    fn test_set_region_invalid_response() {
        let response = vec![0xBB, 0x01, 0x07, 0x00, 0x01, 0x01, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(matches!(rfid.set_region(Region::Us), Err(UhfError::InvalidResponse(_))));
    }

    // ===================
    // Phase 2: Channel tests
    // ===================

    #[test]
    fn test_create_get_channel_command() {
        let result = UhfRfid::<DummyTransport>::create_command(0xAA, &[]);
        assert_eq!(result, [0xBB, 0x00, 0xAA, 0x00, 0x00, 0xAA, 0x7E]);
    }

    #[test]
    fn test_get_channel_valid() {
        let response = vec![0xBB, 0x01, 0xAA, 0x00, 0x01, 0x0A, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let channel = rfid.get_channel().unwrap();
        assert_eq!(channel, 10);
    }

    #[test]
    fn test_get_channel_invalid_response() {
        let response = vec![0xBB, 0x01, 0xAB, 0x00, 0x01, 0x0A, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(matches!(rfid.get_channel(), Err(UhfError::InvalidResponse(_))));
    }

    #[test]
    fn test_create_set_channel_command() {
        let result = UhfRfid::<DummyTransport>::create_command(0xAB, &[0x0A]);
        assert_eq!(result, [0xBB, 0x00, 0xAB, 0x00, 0x01, 0x0A, 0xB6, 0x7E]);
    }

    #[test]
    fn test_set_channel_valid() {
        let response = vec![0xBB, 0x01, 0xAB, 0x00, 0x01, 0x00, 0xAD, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(rfid.set_channel(10).is_ok());
    }

    #[test]
    fn test_set_channel_invalid_response() {
        let response = vec![0xBB, 0x01, 0xAB, 0x00, 0x01, 0x01, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(matches!(rfid.set_channel(10), Err(UhfError::InvalidResponse(_))));
    }

    // ===================
    // Phase 2: Auto Frequency Hopping tests
    // ===================

    #[test]
    fn test_create_set_auto_freq_hop_command_enabled() {
        let result = UhfRfid::<DummyTransport>::create_command(0xAD, &[0xFF]);
        assert_eq!(result, [0xBB, 0x00, 0xAD, 0x00, 0x01, 0xFF, 0xAD, 0x7E]);
    }

    #[test]
    fn test_create_set_auto_freq_hop_command_disabled() {
        let result = UhfRfid::<DummyTransport>::create_command(0xAD, &[0x00]);
        assert_eq!(result, [0xBB, 0x00, 0xAD, 0x00, 0x01, 0x00, 0xAE, 0x7E]);
    }

    #[test]
    fn test_set_auto_freq_hop_enabled() {
        let response = vec![0xBB, 0x01, 0xAD, 0x00, 0x01, 0x00, 0xAF, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(rfid.set_auto_freq_hop(true).is_ok());
    }

    #[test]
    fn test_set_auto_freq_hop_disabled() {
        let response = vec![0xBB, 0x01, 0xAD, 0x00, 0x01, 0x00, 0xAF, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(rfid.set_auto_freq_hop(false).is_ok());
    }

    // ===================
    // Phase 2: Insert Channel tests
    // ===================

    #[test]
    fn test_create_insert_channel_command() {
        let result = UhfRfid::<DummyTransport>::create_command(0xA9, &[0x05]);
        assert_eq!(result, [0xBB, 0x00, 0xA9, 0x00, 0x01, 0x05, 0xAF, 0x7E]);
    }

    #[test]
    fn test_insert_channel_valid() {
        let response = vec![0xBB, 0x01, 0xA9, 0x00, 0x01, 0x00, 0xAB, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(rfid.insert_channel(5).is_ok());
    }

    // ===================
    // Phase 2: Continuous Carrier tests
    // ===================

    #[test]
    fn test_create_set_continuous_carrier_command_enabled() {
        let result = UhfRfid::<DummyTransport>::create_command(0xB0, &[0xFF]);
        assert_eq!(result, [0xBB, 0x00, 0xB0, 0x00, 0x01, 0xFF, 0xB0, 0x7E]);
    }

    #[test]
    fn test_create_set_continuous_carrier_command_disabled() {
        let result = UhfRfid::<DummyTransport>::create_command(0xB0, &[0x00]);
        assert_eq!(result, [0xBB, 0x00, 0xB0, 0x00, 0x01, 0x00, 0xB1, 0x7E]);
    }

    #[test]
    fn test_set_continuous_carrier_enabled() {
        let response = vec![0xBB, 0x01, 0xB0, 0x00, 0x01, 0x00, 0xB2, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(rfid.set_continuous_carrier(true).is_ok());
    }

    #[test]
    fn test_set_continuous_carrier_disabled() {
        let response = vec![0xBB, 0x01, 0xB0, 0x00, 0x01, 0x00, 0xB2, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(rfid.set_continuous_carrier(false).is_ok());
    }

    // ===================
    // Phase 2: Baud Rate tests
    // ===================

    #[test]
    fn test_create_set_baud_rate_command() {
        // Baud rate index 1 = 115200
        let result = UhfRfid::<DummyTransport>::create_command(0x11, &[0x01]);
        assert_eq!(result, [0xBB, 0x00, 0x11, 0x00, 0x01, 0x01, 0x13, 0x7E]);
    }

    #[test]
    fn test_set_baud_rate_valid() {
        let response = vec![0xBB, 0x01, 0x11, 0x00, 0x01, 0x00, 0x13, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(rfid.set_baud_rate(1).is_ok());
    }

    #[test]
    fn test_set_baud_rate_invalid_index() {
        let transport = MockTransport::new(vec![]);
        let mut rfid = UhfRfid::new(transport);

        assert!(matches!(rfid.set_baud_rate(3), Err(UhfError::InvalidParameter(_))));
    }

    #[test]
    fn test_set_baud_rate_all_valid_indices() {
        for i in 0..=2 {
            let response = vec![0xBB, 0x01, 0x11, 0x00, 0x01, 0x00, 0x13, 0x7E];
            let transport = MockTransport::new(response);
            let mut rfid = UhfRfid::new(transport);

            assert!(rfid.set_baud_rate(i).is_ok(), "Baud rate index {} should be valid", i);
        }
    }

    // ===================
    // Phase 3: Tag Memory Operations tests
    // ===================

    #[test]
    fn test_create_read_tag_data_command() {
        // Read 2 words from EPC bank at word address 2, no password
        let result = UhfRfid::<DummyTransport>::create_command(
            0x39,
            &[0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x02],
        );
        assert_eq!(result[0], 0xBB);
        assert_eq!(result[1], 0x00);
        assert_eq!(result[2], 0x39);
        assert_eq!(*result.last().unwrap(), 0x7E);
    }

    #[test]
    fn test_read_tag_data_valid() {
        // Response with 4 bytes of data (2 words)
        let response = vec![
            0xBB, 0x02, 0x39, 0x00, 0x04, // header, type=tag, cmd, len
            0xDE, 0xAD, 0xBE, 0xEF, // data
            0x00, 0x7E, // checksum, end
        ];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let data = rfid
            .read_tag_data(&[0, 0, 0, 0], MemoryBank::Epc, 2, 2)
            .unwrap();
        assert_eq!(data, vec![0xDE, 0xAD, 0xBE, 0xEF]);
    }

    #[test]
    fn test_read_tag_data_zero_word_count() {
        let transport = MockTransport::new(vec![]);
        let mut rfid = UhfRfid::new(transport);

        let result = rfid.read_tag_data(&[0, 0, 0, 0], MemoryBank::Epc, 0, 0);
        assert!(matches!(result, Err(UhfError::InvalidParameter(_))));
    }

    #[test]
    fn test_read_tag_data_error_response() {
        // Error response
        let response = vec![0xBB, 0x01, 0x39, 0x00, 0x01, 0x10, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let result = rfid.read_tag_data(&[0, 0, 0, 0], MemoryBank::Epc, 2, 2);
        assert!(matches!(result, Err(UhfError::InvalidResponse(_))));
    }

    #[test]
    fn test_create_write_tag_data_command() {
        // Write 2 words to EPC bank at word address 2, no password
        let result = UhfRfid::<DummyTransport>::create_command(
            0x49,
            &[0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x02, 0xDE, 0xAD, 0xBE, 0xEF],
        );
        assert_eq!(result[0], 0xBB);
        assert_eq!(result[1], 0x00);
        assert_eq!(result[2], 0x49);
        assert_eq!(*result.last().unwrap(), 0x7E);
    }

    #[test]
    fn test_write_tag_data_valid() {
        let response = vec![0xBB, 0x01, 0x49, 0x00, 0x01, 0x00, 0x4B, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let result = rfid.write_tag_data(
            &[0, 0, 0, 0],
            MemoryBank::Epc,
            2,
            &[0xDE, 0xAD, 0xBE, 0xEF],
        );
        assert!(result.is_ok());
    }

    #[test]
    fn test_write_tag_data_empty_data() {
        let transport = MockTransport::new(vec![]);
        let mut rfid = UhfRfid::new(transport);

        let result = rfid.write_tag_data(&[0, 0, 0, 0], MemoryBank::Epc, 0, &[]);
        assert!(matches!(result, Err(UhfError::InvalidParameter(_))));
    }

    #[test]
    fn test_write_tag_data_odd_length() {
        let transport = MockTransport::new(vec![]);
        let mut rfid = UhfRfid::new(transport);

        let result = rfid.write_tag_data(&[0, 0, 0, 0], MemoryBank::Epc, 0, &[0xDE, 0xAD, 0xBE]);
        assert!(matches!(result, Err(UhfError::InvalidParameter(_))));
    }

    #[test]
    fn test_write_tag_data_too_long() {
        let transport = MockTransport::new(vec![]);
        let mut rfid = UhfRfid::new(transport);

        let data = vec![0u8; 66]; // 65 bytes, exceeds max
        let result = rfid.write_tag_data(&[0, 0, 0, 0], MemoryBank::Epc, 0, &data);
        assert!(matches!(result, Err(UhfError::InvalidParameter(_))));
    }

    #[test]
    fn test_write_tag_data_error_response() {
        let response = vec![0xBB, 0x01, 0x49, 0x00, 0x01, 0x10, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let result = rfid.write_tag_data(&[0, 0, 0, 0], MemoryBank::Epc, 2, &[0xDE, 0xAD]);
        assert!(matches!(result, Err(UhfError::InvalidResponse(_))));
    }

    #[test]
    fn test_lock_payload_to_bytes_user_lock() {
        use types::{LockAction, LockPayload, LockTarget};

        let payload = LockPayload {
            target: LockTarget::User,
            action: LockAction::Lock,
        };
        let bytes = payload.to_bytes();
        // User is at shift 0, mask=0x03, action=0x01
        // payload = (0x03 << 10) | 0x01 = 0x0C01
        assert_eq!(bytes, [0x00, 0x0C, 0x01]);
    }

    #[test]
    fn test_lock_payload_to_bytes_epc_permlock() {
        use types::{LockAction, LockPayload, LockTarget};

        let payload = LockPayload {
            target: LockTarget::Epc,
            action: LockAction::PermLock,
        };
        let bytes = payload.to_bytes();
        // EPC is at shift 4, mask=0x30, action=0x30 (PermLock=3)
        // payload = (0x30 << 10) | 0x30 = 0xC030
        assert_eq!(bytes, [0x00, 0xC0, 0x30]);
    }

    #[test]
    fn test_create_lock_tag_command() {
        // Lock with password and lock payload
        let result = UhfRfid::<DummyTransport>::create_command(
            0x82,
            &[0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x01],
        );
        assert_eq!(result[0], 0xBB);
        assert_eq!(result[1], 0x00);
        assert_eq!(result[2], 0x82);
        assert_eq!(*result.last().unwrap(), 0x7E);
    }

    #[test]
    fn test_lock_tag_valid() {
        use types::{LockAction, LockPayload, LockTarget};

        let response = vec![0xBB, 0x01, 0x82, 0x00, 0x01, 0x00, 0x84, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let payload = LockPayload {
            target: LockTarget::User,
            action: LockAction::Lock,
        };
        let result = rfid.lock_tag(&[0, 0, 0, 0], &payload);
        assert!(result.is_ok());
    }

    #[test]
    fn test_lock_tag_error_response() {
        use types::{LockAction, LockPayload, LockTarget};

        let response = vec![0xBB, 0x01, 0x82, 0x00, 0x01, 0x10, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let payload = LockPayload {
            target: LockTarget::User,
            action: LockAction::Lock,
        };
        let result = rfid.lock_tag(&[0, 0, 0, 0], &payload);
        assert!(matches!(result, Err(UhfError::InvalidResponse(_))));
    }

    #[test]
    fn test_create_kill_tag_command() {
        let result = UhfRfid::<DummyTransport>::create_command(0x65, &[0x12, 0x34, 0x56, 0x78]);
        assert_eq!(result[0], 0xBB);
        assert_eq!(result[1], 0x00);
        assert_eq!(result[2], 0x65);
        assert_eq!(result[5], 0x12);
        assert_eq!(result[6], 0x34);
        assert_eq!(result[7], 0x56);
        assert_eq!(result[8], 0x78);
        assert_eq!(*result.last().unwrap(), 0x7E);
    }

    #[test]
    fn test_kill_tag_valid() {
        let response = vec![0xBB, 0x01, 0x65, 0x00, 0x01, 0x00, 0x67, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let result = rfid.kill_tag(&[0x12, 0x34, 0x56, 0x78]);
        assert!(result.is_ok());
    }

    #[test]
    fn test_kill_tag_zero_password() {
        let transport = MockTransport::new(vec![]);
        let mut rfid = UhfRfid::new(transport);

        let result = rfid.kill_tag(&[0, 0, 0, 0]);
        assert!(matches!(result, Err(UhfError::InvalidParameter(_))));
    }

    #[test]
    fn test_kill_tag_error_response() {
        let response = vec![0xBB, 0x01, 0x65, 0x00, 0x01, 0x10, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let result = rfid.kill_tag(&[0x12, 0x34, 0x56, 0x78]);
        assert!(matches!(result, Err(UhfError::InvalidResponse(_))));
    }

    // ===================
    // Phase 4: Advanced/Vendor Commands tests
    // ===================

    #[test]
    fn test_rf_link_profile_try_from() {
        assert_eq!(RfLinkProfile::try_from(0xD0).unwrap(), RfLinkProfile::Fm0_40kHz);
        assert_eq!(RfLinkProfile::try_from(0xD1).unwrap(), RfLinkProfile::Fm0_400kHz);
        assert_eq!(RfLinkProfile::try_from(0xD2).unwrap(), RfLinkProfile::Miller4_250kHz);
        assert!(RfLinkProfile::try_from(0xFF).is_err());
    }

    #[test]
    fn test_qt_control_to_byte() {
        use types::QtControl;

        let qt = QtControl {
            short_range: false,
            persistence: false,
        };
        assert_eq!(qt.to_byte(), 0x00);

        let qt = QtControl {
            short_range: true,
            persistence: false,
        };
        assert_eq!(qt.to_byte(), 0x01);

        let qt = QtControl {
            short_range: false,
            persistence: true,
        };
        assert_eq!(qt.to_byte(), 0x02);

        let qt = QtControl {
            short_range: true,
            persistence: true,
        };
        assert_eq!(qt.to_byte(), 0x03);
    }

    #[test]
    fn test_create_inventory_buffer_command() {
        let result = UhfRfid::<DummyTransport>::create_command(0x18, &[0x22, 0x00, 0x0A]);
        assert_eq!(result[0], 0xBB);
        assert_eq!(result[2], 0x18);
        assert_eq!(*result.last().unwrap(), 0x7E);
    }

    #[test]
    fn test_inventory_buffer_valid() {
        let response = vec![0xBB, 0x01, 0x18, 0x00, 0x01, 0x00, 0x1A, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(rfid.inventory_buffer(10).is_ok());
    }

    #[test]
    fn test_inventory_buffer_zero_count() {
        let transport = MockTransport::new(vec![]);
        let mut rfid = UhfRfid::new(transport);

        let result = rfid.inventory_buffer(0);
        assert!(matches!(result, Err(UhfError::InvalidParameter(_))));
    }

    #[test]
    fn test_create_clear_buffer_command() {
        let result = UhfRfid::<DummyTransport>::create_command(0x2A, &[]);
        assert_eq!(result, [0xBB, 0x00, 0x2A, 0x00, 0x00, 0x2A, 0x7E]);
    }

    #[test]
    fn test_clear_buffer_valid() {
        let response = vec![0xBB, 0x01, 0x2A, 0x00, 0x01, 0x00, 0x2C, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(rfid.clear_buffer().is_ok());
    }

    #[test]
    fn test_get_buffer_data_empty() {
        let response = vec![0xBB, 0x01, 0x29, 0x00, 0x01, 0x00, 0x2B, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let tags = rfid.get_buffer_data().unwrap();
        assert!(tags.is_empty());
    }

    #[test]
    fn test_create_get_rf_link_profile_command() {
        let result = UhfRfid::<DummyTransport>::create_command(0x6A, &[]);
        assert_eq!(result, [0xBB, 0x00, 0x6A, 0x00, 0x00, 0x6A, 0x7E]);
    }

    #[test]
    fn test_get_rf_link_profile_valid() {
        let response = vec![0xBB, 0x01, 0x6A, 0x00, 0x01, 0xD0, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let profile = rfid.get_rf_link_profile().unwrap();
        assert_eq!(profile, RfLinkProfile::Fm0_40kHz);
    }

    #[test]
    fn test_create_set_rf_link_profile_command() {
        let result = UhfRfid::<DummyTransport>::create_command(0x69, &[0xD1]);
        assert_eq!(result[0], 0xBB);
        assert_eq!(result[2], 0x69);
        assert_eq!(result[5], 0xD1);
        assert_eq!(*result.last().unwrap(), 0x7E);
    }

    #[test]
    fn test_set_rf_link_profile_valid() {
        let response = vec![0xBB, 0x01, 0x69, 0x00, 0x01, 0x00, 0x6B, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(rfid.set_rf_link_profile(RfLinkProfile::Fm0_400kHz).is_ok());
    }

    #[test]
    fn test_create_get_reader_sensitivity_command() {
        let result = UhfRfid::<DummyTransport>::create_command(0xF1, &[]);
        assert_eq!(result, [0xBB, 0x00, 0xF1, 0x00, 0x00, 0xF1, 0x7E]);
    }

    #[test]
    fn test_get_reader_sensitivity_valid() {
        let response = vec![0xBB, 0x01, 0xF1, 0x00, 0x01, 0x0A, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let sensitivity = rfid.get_reader_sensitivity().unwrap();
        assert_eq!(sensitivity, 0x0A);
    }

    #[test]
    fn test_create_set_reader_sensitivity_command() {
        let result = UhfRfid::<DummyTransport>::create_command(0xF0, &[0x10]);
        assert_eq!(result[0], 0xBB);
        assert_eq!(result[2], 0xF0);
        assert_eq!(result[5], 0x10);
        assert_eq!(*result.last().unwrap(), 0x7E);
    }

    #[test]
    fn test_set_reader_sensitivity_valid() {
        let response = vec![0xBB, 0x01, 0xF0, 0x00, 0x01, 0x00, 0xF2, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(rfid.set_reader_sensitivity(16).is_ok());
    }

    #[test]
    fn test_create_block_permalock_command() {
        let result = UhfRfid::<DummyTransport>::create_command(
            0xD3,
            &[0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x01, 0xFF, 0xFF],
        );
        assert_eq!(result[0], 0xBB);
        assert_eq!(result[2], 0xD3);
        assert_eq!(*result.last().unwrap(), 0x7E);
    }

    #[test]
    fn test_block_permalock_valid() {
        let response = vec![0xBB, 0x01, 0xD3, 0x00, 0x01, 0x00, 0xD5, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let result = rfid.block_permalock(&[0, 0, 0, 0], MemoryBank::User, 0, 1, 0xFFFF);
        assert!(result.is_ok());
    }

    #[test]
    fn test_create_nxp_read_protect_command() {
        let result = UhfRfid::<DummyTransport>::create_command(0xE1, &[0x00, 0x00, 0x00, 0x00]);
        assert_eq!(result[0], 0xBB);
        assert_eq!(result[2], 0xE1);
        assert_eq!(*result.last().unwrap(), 0x7E);
    }

    #[test]
    fn test_nxp_read_protect_valid() {
        let response = vec![0xBB, 0x01, 0xE1, 0x00, 0x01, 0x00, 0xE3, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(rfid.nxp_read_protect(&[0, 0, 0, 0]).is_ok());
    }

    #[test]
    fn test_nxp_reset_read_protect_valid() {
        let response = vec![0xBB, 0x01, 0xE2, 0x00, 0x01, 0x00, 0xE4, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(rfid.nxp_reset_read_protect(&[0, 0, 0, 0]).is_ok());
    }

    #[test]
    fn test_nxp_change_eas_enable() {
        let response = vec![0xBB, 0x01, 0xE3, 0x00, 0x01, 0x00, 0xE5, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(rfid.nxp_change_eas(&[0, 0, 0, 0], true).is_ok());
    }

    #[test]
    fn test_nxp_change_eas_disable() {
        let response = vec![0xBB, 0x01, 0xE3, 0x00, 0x01, 0x00, 0xE5, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(rfid.nxp_change_eas(&[0, 0, 0, 0], false).is_ok());
    }

    #[test]
    fn test_nxp_eas_alarm_detected() {
        // Tag response indicates EAS detected
        let response = vec![0xBB, 0x02, 0xE4, 0x00, 0x01, 0x00, 0xE6, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let result = rfid.nxp_eas_alarm().unwrap();
        assert!(result);
    }

    #[test]
    fn test_nxp_eas_alarm_not_detected() {
        // Notification response indicates no EAS
        let response = vec![0xBB, 0x01, 0xE4, 0x00, 0x01, 0x00, 0xE6, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let result = rfid.nxp_eas_alarm().unwrap();
        assert!(result);
    }

    #[test]
    fn test_nxp_change_config_valid() {
        let response = vec![0xBB, 0x01, 0xE0, 0x00, 0x01, 0x00, 0xE2, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        assert!(rfid.nxp_change_config(&[0, 0, 0, 0], 0x1234).is_ok());
    }

    #[test]
    fn test_impinj_monza_qt_read() {
        use types::QtControl;

        let response = vec![0xBB, 0x01, 0xE5, 0x00, 0x02, 0x00, 0x03, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let qt = QtControl {
            short_range: false,
            persistence: false,
        };
        let result = rfid.impinj_monza_qt(&[0, 0, 0, 0], &qt, true).unwrap();
        assert_eq!(result, 0x03);
    }

    #[test]
    fn test_impinj_monza_qt_write() {
        use types::QtControl;

        let response = vec![0xBB, 0x01, 0xE5, 0x00, 0x02, 0x00, 0x00, 0x7E];
        let transport = MockTransport::new(response);
        let mut rfid = UhfRfid::new(transport);

        let qt = QtControl {
            short_range: true,
            persistence: true,
        };
        let result = rfid.impinj_monza_qt(&[0, 0, 0, 0], &qt, false);
        assert!(result.is_ok());
    }
}
