/// Trait for RFID reader communication backends.
/// Implement this trait for different transports (UART, serial port, etc.)
pub trait RfidTransport {
    /// Error type for transport operations
    type Error: std::fmt::Debug;

    /// Write data to the transport
    fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error>;

    /// Read data from the transport with a timeout in milliseconds
    fn read(&mut self, buf: &mut [u8], timeout_ms: u32) -> Result<usize, Self::Error>;

    /// Clear the input buffer
    fn clear_input(&mut self) -> Result<(), Self::Error>;
}
