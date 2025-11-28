//! UART transport for ESP32 using esp-idf-svc

use crate::transport::RfidTransport;
use esp_idf_svc::hal::gpio::{self, InputPin, OutputPin};
use esp_idf_svc::hal::peripheral::Peripheral;
use esp_idf_svc::hal::uart::{self, UartDriver};
use std::time::Duration;

pub struct UartTransport<'a> {
    uart: UartDriver<'a>,
}

impl<'a> UartTransport<'a> {
    pub fn new(
        uart: impl Peripheral<P = impl uart::Uart> + 'a,
        tx: impl Peripheral<P = impl OutputPin> + 'a,
        rx: impl Peripheral<P = impl InputPin> + 'a,
        baud_rate: u32,
    ) -> Result<Self, esp_idf_svc::sys::EspError> {
        let config = uart::config::Config::default().baudrate(baud_rate.into());
        let uart = UartDriver::new(
            uart,
            tx,
            rx,
            Option::<gpio::Gpio0>::None,
            Option::<gpio::Gpio0>::None,
            &config,
        )?;

        std::thread::sleep(Duration::from_millis(500));
        uart.clear_rx()?;

        Ok(Self { uart })
    }
}

impl RfidTransport for UartTransport<'_> {
    type Error = esp_idf_svc::sys::EspError;

    fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error> {
        self.uart.write(data)
    }

    fn read(&mut self, buf: &mut [u8], timeout_ms: u32) -> Result<usize, Self::Error> {
        self.uart.read(buf, timeout_ms)
    }

    fn clear_input(&mut self) -> Result<(), Self::Error> {
        self.uart.clear_rx()
    }
}
