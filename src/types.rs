//! Types for RFID operations

/// Information about a detected RFID tag
#[derive(Debug, Clone)]
pub struct TagInfo {
    pub epc: String,
    pub rssi: u8,
}

impl PartialEq for TagInfo {
    fn eq(&self, other: &Self) -> bool {
        self.epc == other.epc
    }
}

/// Errors that can occur during RFID operations
#[derive(Debug)]
pub enum UhfError {
    /// Transport layer error (UART, serial, etc.)
    Transport(String),
    /// Invalid parameter passed to a function
    InvalidParameter(String),
    /// Invalid response received from the reader
    InvalidResponse(String),
}

/// Convert bytes to uppercase hex string
pub(crate) fn bytes_to_hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{:02X}", b)).collect()
}
