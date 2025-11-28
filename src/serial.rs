//! Serial port transport for desktop using serialport crate

use crate::transport::RfidTransport;
use std::time::Duration;

pub struct SerialTransport {
    port: Box<dyn serialport::SerialPort>,
}

impl SerialTransport {
    pub fn new(port_name: &str, baud_rate: u32) -> Result<Self, serialport::Error> {
        let port = serialport::new(port_name, baud_rate)
            .timeout(Duration::from_secs(1))
            .open()?;
        std::thread::sleep(Duration::from_millis(500));
        port.clear(serialport::ClearBuffer::Input)?;

        Ok(Self { port })
    }
}

impl RfidTransport for SerialTransport {
    type Error = std::io::Error;

    fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error> {
        std::io::Write::write(&mut self.port, data)
    }

    fn read(&mut self, buf: &mut [u8], timeout_ms: u32) -> Result<usize, Self::Error> {
        self.port
            .set_timeout(Duration::from_millis(timeout_ms as u64))
            .map_err(|e| std::io::Error::other(e))?;
        std::io::Read::read(&mut self.port, buf)
    }

    fn clear_input(&mut self) -> Result<(), Self::Error> {
        self.port
            .clear(serialport::ClearBuffer::Input)
            .map_err(|e| std::io::Error::other(e))
    }
}
