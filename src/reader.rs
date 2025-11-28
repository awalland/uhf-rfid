use log::{debug, error, warn};
use std::time::{Duration, Instant};

use crate::transport::RfidTransport;
use crate::types::{TagInfo, UhfError, bytes_to_hex};

pub struct UhfRfid<T: RfidTransport> {
    transport: T,
}

impl<T: RfidTransport> UhfRfid<T> {
    // Protocol constants
    const HEADER: u8 = 0xBB;
    const END: u8 = 0x7E;
    const CMD_TYPE: u8 = 0x00;
    const RESP_TYPE_NOTIFICATION: u8 = 0x01;
    const RESP_TYPE_TAG: u8 = 0x02;

    // Command codes
    const GET_FIRMWARE: u8 = 0x03;
    const SINGLE_POLL: u8 = 0x22;
    const MULTIPLE_POLL: u8 = 0x27;
    const GET_TX_POWER: u8 = 0xB7;
    const SET_TX_POWER: u8 = 0xB6;

    /// Create a new RFID reader instance with the given transport
    pub fn new(transport: T) -> Self {
        Self { transport }
    }

    /// Get firmware version
    pub fn get_firmware_version(&mut self) -> Result<String, UhfError> {
        self.exec(&Self::create_command(Self::GET_FIRMWARE, &[0x01]))
            .and_then(|response| Self::parse_firmware_version(&response))
    }

    /// Poll for a single RFID tag
    pub fn single_poll(&mut self) -> Result<Option<TagInfo>, UhfError> {
        self.exec(&Self::create_command(Self::SINGLE_POLL, &[]))
            .and_then(|response| Self::parse_tag(&response))
    }

    /// Poll for multiple RFID tags with a callback for each tag
    pub fn multiple_poll_with_callback<F>(&mut self, count: u16, mut callback: F) -> Result<usize, UhfError>
    where
        F: FnMut(TagInfo),
    {
        if count == 0 {
            return Err(UhfError::InvalidParameter("Poll count must be at least 1".into()));
        }

        let count_msb = (count >> 8) as u8;
        let count_lsb = (count & 0xFF) as u8;

        self.transport
            .clear_input()
            .map_err(|e| UhfError::Transport(format!("{:?}", e)))?;
        self.transport
            .write(&Self::create_command(
                Self::MULTIPLE_POLL,
                &[0x22, count_msb, count_lsb],
            ))
            .map_err(|e| UhfError::Transport(format!("{:?}", e)))?;
        std::thread::sleep(Duration::from_millis(100));

        let mut tag_count = 0;
        let start = Instant::now();
        let max_wait = Duration::from_secs(3);
        let mut buffer = Vec::new();

        loop {
            let mut temp_buf = [0u8; 256];

            match self.transport.read(&mut temp_buf, 50) {
                Ok(bytes_read) if bytes_read > 0 => {
                    buffer.extend_from_slice(&temp_buf[..bytes_read]);

                    while let Some(frame_end) = buffer.iter().position(|&b| b == Self::END) {
                        if let Some(frame_start) = buffer[..frame_end].iter().rposition(|&b| b == Self::HEADER) {
                            let frame = &buffer[frame_start..=frame_end];

                            if frame.len() >= 8
                                && frame[1] == Self::RESP_TYPE_NOTIFICATION
                                && frame[2] == 0xFF
                                && frame[5] == 0x15
                            {
                                buffer.drain(..=frame_end);
                                return Ok(tag_count);
                            }

                            match Self::parse_tag(frame) {
                                Ok(Some(tag)) => {
                                    callback(tag);
                                    tag_count += 1;
                                }
                                Ok(None) => {}
                                Err(e) => {
                                    warn!("Failed to parse frame: {:?}", e);
                                }
                            }

                            buffer.drain(..=frame_end);
                        } else {
                            buffer.drain(..=frame_end);
                        }
                    }
                }
                Ok(_) => {
                    if start.elapsed() > max_wait {
                        break;
                    }
                    std::thread::sleep(Duration::from_millis(50));
                }
                Err(_) => {
                    if start.elapsed() > max_wait {
                        break;
                    }
                    std::thread::sleep(Duration::from_millis(50));
                }
            }
        }

        Ok(tag_count)
    }

    /// Poll for multiple RFID tags
    pub fn multiple_poll(&mut self, count: u16) -> Result<Vec<TagInfo>, UhfError> {
        let mut tags = Vec::new();
        self.multiple_poll_with_callback(count, |tag| tags.push(tag))?;
        Ok(tags)
    }

    /// Get current transmit power in dBm
    pub fn get_tx_power(&mut self) -> Result<u16, UhfError> {
        let response = self.exec(&Self::create_command(Self::GET_TX_POWER, &[]))?;

        if response.len() >= 8
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::GET_TX_POWER
            && response[3] == 0x00
            && response[4] == 0x02
        {
            let power_raw = ((response[5] as u16) << 8) | (response[6] as u16);
            let power_dbm = power_raw / 100;
            Ok(power_dbm)
        } else {
            Err(UhfError::InvalidResponse("Failed to get transmit power".into()))
        }
    }

    /// Set transmit power (18-26 dBm valid range)
    pub fn set_tx_power(&mut self, power_dbm: u16) -> Result<(), UhfError> {
        const MIN_POWER_DBM: u16 = 18;
        const MAX_POWER_DBM: u16 = 26;

        if power_dbm < MIN_POWER_DBM {
            return Err(UhfError::InvalidParameter(format!(
                "Transmit power too low: {} dBm (minimum: {} dBm)",
                power_dbm, MIN_POWER_DBM
            )));
        }

        if power_dbm > MAX_POWER_DBM {
            return Err(UhfError::InvalidParameter(format!(
                "Transmit power too high: {} dBm (maximum: {} dBm)",
                power_dbm, MAX_POWER_DBM
            )));
        }

        let power = power_dbm * 100;
        let power_msb = (power >> 8) as u8;
        let power_lsb = (power & 0xFF) as u8;

        let response = self.exec(&Self::create_command(Self::SET_TX_POWER, &[power_msb, power_lsb]))?;

        if response.len() >= 7
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::SET_TX_POWER
            && response[5] == 0x00
        {
            Ok(())
        } else {
            Err(UhfError::InvalidResponse("Failed to set transmit power".into()))
        }
    }

    fn parse_tag(response: &[u8]) -> Result<Option<TagInfo>, UhfError> {
        if response.len() < 12 {
            return Ok(None);
        }

        if response[0] == Self::HEADER && response[1] == Self::RESP_TYPE_TAG {
            let data_length = response[4] as usize;
            let rssi = response[5];

            let epc_start = 8;
            let epc_end = epc_start + data_length.saturating_sub(5);

            if epc_end > response.len() {
                return Err(UhfError::InvalidResponse(format!(
                    "Invalid tag response: data_length claims {} bytes but response only has {} bytes",
                    data_length,
                    response.len()
                )));
            }

            let epc_bytes = &response[epc_start..epc_end];
            Ok(Some(TagInfo {
                epc: bytes_to_hex(epc_bytes),
                rssi,
            }))
        } else if response[0] == Self::HEADER {
            Ok(None)
        } else {
            Err(UhfError::InvalidResponse(format!(
                "Invalid response header: {:02X?}",
                response
            )))
        }
    }

    fn parse_firmware_version(response: &[u8]) -> Result<String, UhfError> {
        if response.len() > 6 && response[0] == Self::HEADER && response[1] == Self::RESP_TYPE_NOTIFICATION {
            let version_bytes = &response[6..response.len() - 2];
            Ok(String::from_utf8_lossy(version_bytes).to_string())
        } else {
            Err(UhfError::InvalidResponse("Invalid firmware response".into()))
        }
    }

    fn exec(&mut self, cmd: &[u8]) -> Result<Vec<u8>, UhfError> {
        self.transport
            .clear_input()
            .map_err(|e| UhfError::Transport(format!("{:?}", e)))?;
        debug!("Sending command: {:02X?}", cmd);
        let written = self
            .transport
            .write(cmd)
            .map_err(|e| UhfError::Transport(format!("{:?}", e)))?;
        debug!("Wrote {} bytes", written);
        std::thread::sleep(Duration::from_millis(200));

        let mut response = vec![0u8; 100];
        match self.transport.read(&mut response, 500) {
            Ok(bytes_read) => {
                response.truncate(bytes_read);
                debug!("Received {} bytes: {:02X?}", bytes_read, response);
                Ok(response)
            }
            Err(e) => {
                error!("Read error: {:?}", e);
                Err(UhfError::Transport(format!("{:?}", e)))
            }
        }
    }

    pub(crate) fn create_command(command: u8, params: &[u8]) -> Vec<u8> {
        let param_len = params.len() as u16;
        let msb = (param_len >> 8) as u8;
        let lsb = (param_len & 0xFF) as u8;

        let checksum = [Self::CMD_TYPE, command, msb, lsb]
            .iter()
            .chain(params.iter())
            .fold(0u8, |acc, &b| acc.wrapping_add(b));

        let mut cmd = vec![Self::HEADER, Self::CMD_TYPE, command, msb, lsb];
        cmd.extend_from_slice(params);
        cmd.push(checksum);
        cmd.push(Self::END);
        cmd
    }
}
