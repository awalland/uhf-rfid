use log::{debug, error, warn};
use std::time::{Duration, Instant};

use crate::transport::RfidTransport;
use crate::types::{
    bytes_to_hex, LockPayload, MemoryBank, QtControl, QueryParams, Region, RfLinkProfile,
    SelectAction, SelectMode, SelectParams, SelectTarget, TagInfo, UhfError,
};

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
    const STOP_MULTIPLE_POLL: u8 = 0x28;
    const GET_SELECT_PARAM: u8 = 0x0B;
    const SET_SELECT_PARAM: u8 = 0x0C;
    const SET_SELECT_MODE: u8 = 0x12;
    const GET_TX_POWER: u8 = 0xB7;
    const SET_TX_POWER: u8 = 0xB6;
    const SET_REGION: u8 = 0x07;
    const GET_REGION: u8 = 0x08;
    const GET_QUERY_PARAM: u8 = 0x0D;
    const SET_QUERY_PARAM: u8 = 0x0E;
    const SET_BAUD_RATE: u8 = 0x11;
    const INSERT_CHANNEL: u8 = 0xA9;
    const GET_CHANNEL: u8 = 0xAA;
    const SET_CHANNEL: u8 = 0xAB;
    const SET_AUTO_FREQ_HOP: u8 = 0xAD;
    const SET_CONTINUOUS_CARRIER: u8 = 0xB0;
    const READ_TAG_DATA: u8 = 0x39;
    const WRITE_TAG_DATA: u8 = 0x49;
    const LOCK_TAG: u8 = 0x82;
    const KILL_TAG: u8 = 0x65;
    const INVENTORY_BUFFER: u8 = 0x18;
    const GET_BUFFER_DATA: u8 = 0x29;
    const CLEAR_BUFFER: u8 = 0x2A;
    const GET_RF_LINK_PROFILE: u8 = 0x6A;
    const SET_RF_LINK_PROFILE: u8 = 0x69;
    const BLOCK_PERMALOCK: u8 = 0xD3;
    const NXP_CHANGE_CONFIG: u8 = 0xE0;
    const NXP_READ_PROTECT: u8 = 0xE1;
    const NXP_RESET_READ_PROTECT: u8 = 0xE2;
    const NXP_CHANGE_EAS: u8 = 0xE3;
    const NXP_EAS_ALARM: u8 = 0xE4;
    const IMPINJ_MONZA_QT: u8 = 0xE5;
    const SET_READER_SENSITIVITY: u8 = 0xF0;
    const GET_READER_SENSITIVITY: u8 = 0xF1;

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
    ///
    /// # Arguments
    /// * `rounds` - Number of inventory rounds to perform (not the number of tags to find)
    /// * `callback` - Function called for each discovered tag
    ///
    /// # Returns
    /// Total number of tags discovered across all rounds
    pub fn multiple_poll_with_callback<F>(&mut self, rounds: u16, mut callback: F) -> Result<usize, UhfError>
    where
        F: FnMut(TagInfo),
    {
        if rounds == 0 {
            return Err(UhfError::InvalidParameter("Poll rounds must be at least 1".into()));
        }

        let rounds_msb = (rounds >> 8) as u8;
        let rounds_lsb = (rounds & 0xFF) as u8;

        self.transport
            .clear_input()
            .map_err(|e| UhfError::Transport(format!("{:?}", e)))?;
        self.transport
            .write(&Self::create_command(
                Self::MULTIPLE_POLL,
                &[0x22, rounds_msb, rounds_lsb],
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
    ///
    /// # Arguments
    /// * `rounds` - Number of inventory rounds to perform (not the number of tags to find)
    ///
    /// # Returns
    /// Vector of all tags discovered across all rounds
    pub fn multiple_poll(&mut self, rounds: u16) -> Result<Vec<TagInfo>, UhfError> {
        let mut tags = Vec::new();
        self.multiple_poll_with_callback(rounds, |tag| tags.push(tag))?;
        Ok(tags)
    }

    /// Poll for RFID tags for a specified duration
    ///
    /// This starts continuous polling (0xFFFF rounds) and collects tags until
    /// the timeout expires, then stops polling.
    ///
    /// # Arguments
    /// * `timeout` - How long to poll for tags
    ///
    /// # Returns
    /// Vector of all tags discovered during the timeout period
    pub fn poll_for_duration(&mut self, timeout: Duration) -> Result<Vec<TagInfo>, UhfError> {
        let mut tags = Vec::new();
        self.poll_for_duration_with_callback(timeout, |tag| tags.push(tag))?;
        Ok(tags)
    }

    /// Poll for RFID tags for a specified duration with a callback
    ///
    /// This starts continuous polling (0xFFFF rounds) and calls the callback
    /// for each tag discovered until the timeout expires, then stops polling.
    ///
    /// # Arguments
    /// * `timeout` - How long to poll for tags
    /// * `callback` - Function called for each discovered tag
    ///
    /// # Returns
    /// Total number of tags discovered
    pub fn poll_for_duration_with_callback<F>(
        &mut self,
        timeout: Duration,
        mut callback: F,
    ) -> Result<usize, UhfError>
    where
        F: FnMut(TagInfo),
    {
        // Start continuous polling with max count
        self.transport
            .clear_input()
            .map_err(|e| UhfError::Transport(format!("{:?}", e)))?;
        self.transport
            .write(&Self::create_command(
                Self::MULTIPLE_POLL,
                &[0x22, 0xFF, 0xFF], // 0xFFFF = 65535 rounds (continuous)
            ))
            .map_err(|e| UhfError::Transport(format!("{:?}", e)))?;

        let mut tag_count = 0;
        let start = Instant::now();
        let mut buffer = Vec::new();

        // Read tags until timeout
        while start.elapsed() < timeout {
            let mut temp_buf = [0u8; 256];

            match self.transport.read(&mut temp_buf, 50) {
                Ok(bytes_read) if bytes_read > 0 => {
                    buffer.extend_from_slice(&temp_buf[..bytes_read]);

                    while let Some(frame_end) = buffer.iter().position(|&b| b == Self::END) {
                        if let Some(frame_start) =
                            buffer[..frame_end].iter().rposition(|&b| b == Self::HEADER)
                        {
                            let frame = &buffer[frame_start..=frame_end];

                            // Check for end-of-poll notification - restart polling
                            if frame.len() >= 8
                                && frame[1] == Self::RESP_TYPE_NOTIFICATION
                                && frame[2] == 0xFF
                                && frame[5] == 0x15
                            {
                                buffer.drain(..=frame_end);
                                // Restart polling if we still have time
                                if start.elapsed() < timeout {
                                    let _ = self.transport.write(&Self::create_command(
                                        Self::MULTIPLE_POLL,
                                        &[0x22, 0xFF, 0xFF],
                                    ));
                                }
                                continue;
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
                    std::thread::sleep(Duration::from_millis(10));
                }
                Err(_) => {
                    std::thread::sleep(Duration::from_millis(10));
                }
            }
        }

        // Stop polling
        let _ = self.transport.write(&Self::create_command(Self::STOP_MULTIPLE_POLL, &[]));

        // Drain any remaining responses
        std::thread::sleep(Duration::from_millis(100));
        let mut drain_buf = [0u8; 256];
        while self.transport.read(&mut drain_buf, 50).unwrap_or(0) > 0 {}

        Ok(tag_count)
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

    /// Stop multiple polling operation immediately
    ///
    /// Note: This function is intentionally not exposed publicly. With a blocking serial
    /// transport, there's no safe way to call this while `multiple_poll` is running,
    /// and we are reading all the tags discovered during the run,
    /// since that function holds `&mut self` for the entire duration. To stop early,
    /// use a small poll count and loop externally, or use `single_poll` in a loop.
    #[allow(dead_code)]
    pub(crate) fn stop_multiple_poll(&mut self) -> Result<(), UhfError> {
        let response = self.exec(&Self::create_command(Self::STOP_MULTIPLE_POLL, &[]))?;

        if response.len() >= 7
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::STOP_MULTIPLE_POLL
            && response[5] == 0x00
        {
            Ok(())
        } else {
            Err(UhfError::InvalidResponse("Failed to stop multiple polling".into()))
        }
    }

    /// Set Select parameters for filtering specific tags
    ///
    /// This sets the Select parameters and automatically enables Select mode (0x02).
    /// Use this to filter operations to specific tags based on EPC, TID, or other memory.
    pub fn set_select_param(&mut self, params: &SelectParams) -> Result<(), UhfError> {
        // Validate mask length (max 255 bits = 31.875 bytes, protocol uses bytes)
        if params.mask.len() > 32 {
            return Err(UhfError::InvalidParameter(
                "Mask length exceeds maximum of 32 bytes".into(),
            ));
        }

        // Build SelParam byte: Target (3 bits) | Action (3 bits) | MemBank (2 bits)
        let sel_param = ((params.target as u8) << 5)
            | ((params.action as u8) << 2)
            | (params.mem_bank as u8);

        // Pointer is 4 bytes (32 bits), in bits not words
        let ptr_bytes = params.pointer.to_be_bytes();

        // Mask length in bits
        let mask_len_bits = (params.mask.len() * 8) as u8;

        // Truncate flag: 0x00 = disabled, 0x80 = enabled
        let truncate = if params.truncate { 0x80 } else { 0x00 };

        // Build parameter array
        let mut cmd_params = Vec::with_capacity(7 + params.mask.len());
        cmd_params.push(sel_param);
        cmd_params.extend_from_slice(&ptr_bytes);
        cmd_params.push(mask_len_bits);
        cmd_params.push(truncate);
        cmd_params.extend_from_slice(&params.mask);

        let response = self.exec(&Self::create_command(Self::SET_SELECT_PARAM, &cmd_params))?;

        if response.len() >= 7
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::SET_SELECT_PARAM
            && response[5] == 0x00
        {
            Ok(())
        } else {
            Err(UhfError::InvalidResponse("Failed to set select parameters".into()))
        }
    }

    /// Get current Select parameters
    pub fn get_select_param(&mut self) -> Result<SelectParams, UhfError> {
        let response = self.exec(&Self::create_command(Self::GET_SELECT_PARAM, &[]))?;

        // Minimum response: header + type + cmd + len(2) + sel_param + ptr(4) + mask_len + truncate + checksum + end
        if response.len() < 14
            || response[0] != Self::HEADER
            || response[1] != Self::RESP_TYPE_NOTIFICATION
            || response[2] != Self::GET_SELECT_PARAM
        {
            return Err(UhfError::InvalidResponse("Invalid select parameter response".into()));
        }

        let sel_param = response[5];
        let target = match (sel_param >> 5) & 0x07 {
            0 => SelectTarget::S0,
            1 => SelectTarget::S1,
            2 => SelectTarget::S2,
            3 => SelectTarget::S3,
            4 => SelectTarget::Sl,
            _ => SelectTarget::S0,
        };
        let action = match (sel_param >> 2) & 0x07 {
            0 => SelectAction::Action0,
            1 => SelectAction::Action1,
            2 => SelectAction::Action2,
            3 => SelectAction::Action3,
            4 => SelectAction::Action4,
            5 => SelectAction::Action5,
            6 => SelectAction::Action6,
            7 => SelectAction::Action7,
            _ => SelectAction::Action0,
        };
        let mem_bank = match sel_param & 0x03 {
            0 => MemoryBank::Reserved,
            1 => MemoryBank::Epc,
            2 => MemoryBank::Tid,
            3 => MemoryBank::User,
            _ => MemoryBank::Epc,
        };

        let pointer = u32::from_be_bytes([response[6], response[7], response[8], response[9]]);
        let mask_len_bits = response[10];
        let truncate = response[11] == 0x80;

        // Calculate mask length in bytes
        let mask_len_bytes = ((mask_len_bits + 7) / 8) as usize;

        // Extract mask data
        let mask_start = 12;
        let mask_end = mask_start + mask_len_bytes;

        if response.len() < mask_end + 2 {
            return Err(UhfError::InvalidResponse("Response too short for mask data".into()));
        }

        let mask = response[mask_start..mask_end].to_vec();

        Ok(SelectParams {
            target,
            action,
            mem_bank,
            pointer,
            mask,
            truncate,
        })
    }

    /// Set Select mode
    ///
    /// - `Always` (0x00): Send Select command before every tag operation
    /// - `Disabled` (0x01): Do not send Select command
    /// - `NonPolling` (0x02): Send Select only before Read, Write, Lock, Kill (not polling)
    pub fn set_select_mode(&mut self, mode: SelectMode) -> Result<(), UhfError> {
        let response = self.exec(&Self::create_command(Self::SET_SELECT_MODE, &[mode as u8]))?;

        // Response uses command 0x0C (SET_SELECT_PARAM) per protocol spec
        if response.len() >= 7
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[5] == 0x00
        {
            Ok(())
        } else {
            Err(UhfError::InvalidResponse("Failed to set select mode".into()))
        }
    }

    // ========================================================================
    // Phase 2: Configuration Commands
    // ========================================================================

    /// Get current Query parameters
    pub fn get_query_param(&mut self) -> Result<QueryParams, UhfError> {
        let response = self.exec(&Self::create_command(Self::GET_QUERY_PARAM, &[]))?;

        if response.len() >= 9
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::GET_QUERY_PARAM
            && response[3] == 0x00
            && response[4] == 0x02
        {
            Ok(QueryParams::from_bytes([response[5], response[6]]))
        } else {
            Err(UhfError::InvalidResponse("Failed to get query parameters".into()))
        }
    }

    /// Set Query parameters
    ///
    /// These parameters control the EPC Gen2 Query command used during inventory.
    pub fn set_query_param(&mut self, params: &QueryParams) -> Result<(), UhfError> {
        if params.q > 15 {
            return Err(UhfError::InvalidParameter(
                "Q value must be 0-15".into(),
            ));
        }

        let bytes = params.to_bytes();
        let response = self.exec(&Self::create_command(Self::SET_QUERY_PARAM, &bytes))?;

        if response.len() >= 7
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::SET_QUERY_PARAM
            && response[5] == 0x00
        {
            Ok(())
        } else {
            Err(UhfError::InvalidResponse("Failed to set query parameters".into()))
        }
    }

    /// Get current operating region
    pub fn get_region(&mut self) -> Result<Region, UhfError> {
        let response = self.exec(&Self::create_command(Self::GET_REGION, &[]))?;

        if response.len() >= 8
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::GET_REGION
            && response[3] == 0x00
            && response[4] == 0x01
        {
            Region::try_from(response[5])
                .map_err(|_| UhfError::InvalidResponse(format!("Unknown region code: 0x{:02X}", response[5])))
        } else {
            Err(UhfError::InvalidResponse("Failed to get region".into()))
        }
    }

    /// Set operating region
    ///
    /// This configures the frequency band the reader operates in.
    /// The region must match local regulations.
    pub fn set_region(&mut self, region: Region) -> Result<(), UhfError> {
        let response = self.exec(&Self::create_command(Self::SET_REGION, &[region as u8]))?;

        if response.len() >= 7
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::SET_REGION
            && response[5] == 0x00
        {
            Ok(())
        } else {
            Err(UhfError::InvalidResponse("Failed to set region".into()))
        }
    }

    /// Get current operating channel index
    pub fn get_channel(&mut self) -> Result<u8, UhfError> {
        let response = self.exec(&Self::create_command(Self::GET_CHANNEL, &[]))?;

        if response.len() >= 8
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::GET_CHANNEL
            && response[3] == 0x00
            && response[4] == 0x01
        {
            Ok(response[5])
        } else {
            Err(UhfError::InvalidResponse("Failed to get channel".into()))
        }
    }

    /// Set operating channel index
    ///
    /// The valid channel range depends on the configured region.
    /// Use `Region::frequency_from_channel()` to calculate the actual frequency.
    pub fn set_channel(&mut self, channel: u8) -> Result<(), UhfError> {
        let response = self.exec(&Self::create_command(Self::SET_CHANNEL, &[channel]))?;

        if response.len() >= 7
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::SET_CHANNEL
            && response[5] == 0x00
        {
            Ok(())
        } else {
            Err(UhfError::InvalidResponse("Failed to set channel".into()))
        }
    }

    /// Set automatic frequency hopping mode
    ///
    /// When enabled, the reader automatically hops between channels.
    /// When disabled, the reader uses a fixed channel set via `set_channel()`.
    pub fn set_auto_freq_hop(&mut self, enabled: bool) -> Result<(), UhfError> {
        let param = if enabled { 0xFF } else { 0x00 };
        let response = self.exec(&Self::create_command(Self::SET_AUTO_FREQ_HOP, &[param]))?;

        if response.len() >= 7
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::SET_AUTO_FREQ_HOP
            && response[5] == 0x00
        {
            Ok(())
        } else {
            Err(UhfError::InvalidResponse("Failed to set auto frequency hopping".into()))
        }
    }

    /// Insert a channel into the frequency hopping table
    ///
    /// This adds a channel to the list of channels used during frequency hopping.
    pub fn insert_channel(&mut self, channel: u8) -> Result<(), UhfError> {
        let response = self.exec(&Self::create_command(Self::INSERT_CHANNEL, &[channel]))?;

        if response.len() >= 7
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::INSERT_CHANNEL
            && response[5] == 0x00
        {
            Ok(())
        } else {
            Err(UhfError::InvalidResponse("Failed to insert channel".into()))
        }
    }

    /// Set continuous carrier transmission
    ///
    /// When enabled, the reader transmits a continuous unmodulated carrier.
    /// This is primarily used for testing and regulatory compliance verification.
    pub fn set_continuous_carrier(&mut self, enabled: bool) -> Result<(), UhfError> {
        let param = if enabled { 0xFF } else { 0x00 };
        let response = self.exec(&Self::create_command(Self::SET_CONTINUOUS_CARRIER, &[param]))?;

        if response.len() >= 7
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::SET_CONTINUOUS_CARRIER
            && response[5] == 0x00
        {
            Ok(())
        } else {
            Err(UhfError::InvalidResponse("Failed to set continuous carrier".into()))
        }
    }

    /// Set communication baud rate
    ///
    /// Supported baud rates:
    /// - 0: 38400 bps
    /// - 1: 115200 bps (default)
    /// - 2: 9600 bps
    ///
    /// Note: After changing the baud rate, the transport must be reconfigured
    /// to use the new rate before further communication is possible.
    pub fn set_baud_rate(&mut self, rate_index: u8) -> Result<(), UhfError> {
        if rate_index > 2 {
            return Err(UhfError::InvalidParameter(
                "Baud rate index must be 0 (38400), 1 (115200), or 2 (9600)".into(),
            ));
        }

        let response = self.exec(&Self::create_command(Self::SET_BAUD_RATE, &[rate_index]))?;

        if response.len() >= 7
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::SET_BAUD_RATE
            && response[5] == 0x00
        {
            Ok(())
        } else {
            Err(UhfError::InvalidResponse("Failed to set baud rate".into()))
        }
    }

    // ========================================================================
    // Phase 3: Tag Memory Operations
    // ========================================================================

    /// Read data from a tag's memory bank
    ///
    /// # Arguments
    /// * `access_password` - 4-byte access password (use [0,0,0,0] for no password)
    /// * `mem_bank` - Memory bank to read from
    /// * `word_ptr` - Starting word address (1 word = 2 bytes)
    /// * `word_count` - Number of words to read
    ///
    /// # Returns
    /// The raw bytes read from the tag memory
    pub fn read_tag_data(
        &mut self,
        access_password: &[u8; 4],
        mem_bank: MemoryBank,
        word_ptr: u8,
        word_count: u8,
    ) -> Result<Vec<u8>, UhfError> {
        if word_count == 0 {
            return Err(UhfError::InvalidParameter(
                "Word count must be at least 1".into(),
            ));
        }

        let mut params = Vec::with_capacity(7);
        params.extend_from_slice(access_password);
        params.push(mem_bank as u8);
        params.push(word_ptr);
        params.push(word_count);

        let response = self.exec(&Self::create_command(Self::READ_TAG_DATA, &params))?;

        // Response format: BB 02 39 00 LL [data...] checksum 7E
        // or error: BB 01 39 00 01 EE checksum 7E
        if response.len() >= 8
            && response[0] == Self::HEADER
            && response[2] == Self::READ_TAG_DATA
        {
            if response[1] == Self::RESP_TYPE_TAG {
                // Success - extract data
                let data_len = ((response[3] as usize) << 8) | (response[4] as usize);
                let data_start = 5;
                let data_end = data_start + data_len;

                if response.len() >= data_end + 2 {
                    Ok(response[data_start..data_end].to_vec())
                } else {
                    Err(UhfError::InvalidResponse("Response too short for data".into()))
                }
            } else if response[1] == Self::RESP_TYPE_NOTIFICATION {
                // Error response
                let error_code = if response.len() > 5 { response[5] } else { 0xFF };
                Err(UhfError::InvalidResponse(format!(
                    "Read failed with error code: 0x{:02X}",
                    error_code
                )))
            } else {
                Err(UhfError::InvalidResponse("Unexpected response type".into()))
            }
        } else {
            Err(UhfError::InvalidResponse("Invalid read response".into()))
        }
    }

    /// Write data to a tag's memory bank
    ///
    /// # Arguments
    /// * `access_password` - 4-byte access password (use [0,0,0,0] for no password)
    /// * `mem_bank` - Memory bank to write to
    /// * `word_ptr` - Starting word address (1 word = 2 bytes)
    /// * `data` - Data to write (must be an even number of bytes)
    ///
    /// # Note
    /// Writing to EPC bank at word address 1 (PC + EPC) is common for re-encoding tags.
    pub fn write_tag_data(
        &mut self,
        access_password: &[u8; 4],
        mem_bank: MemoryBank,
        word_ptr: u8,
        data: &[u8],
    ) -> Result<(), UhfError> {
        if data.is_empty() {
            return Err(UhfError::InvalidParameter("Data cannot be empty".into()));
        }
        if data.len() % 2 != 0 {
            return Err(UhfError::InvalidParameter(
                "Data length must be even (word-aligned)".into(),
            ));
        }
        if data.len() > 64 {
            return Err(UhfError::InvalidParameter(
                "Data length exceeds maximum of 64 bytes".into(),
            ));
        }

        let word_count = (data.len() / 2) as u8;

        let mut params = Vec::with_capacity(7 + data.len());
        params.extend_from_slice(access_password);
        params.push(mem_bank as u8);
        params.push(word_ptr);
        params.push(word_count);
        params.extend_from_slice(data);

        let response = self.exec(&Self::create_command(Self::WRITE_TAG_DATA, &params))?;

        if response.len() >= 7
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::WRITE_TAG_DATA
            && response[5] == 0x00
        {
            Ok(())
        } else if response.len() >= 6 && response[5] != 0x00 {
            Err(UhfError::InvalidResponse(format!(
                "Write failed with error code: 0x{:02X}",
                response[5]
            )))
        } else {
            Err(UhfError::InvalidResponse("Failed to write tag data".into()))
        }
    }

    /// Lock a tag's memory or password area
    ///
    /// # Arguments
    /// * `access_password` - 4-byte access password (use [0,0,0,0] for no password)
    /// * `lock_payload` - Specifies which area to lock and how
    ///
    /// # Warning
    /// Permanent lock operations are irreversible!
    pub fn lock_tag(
        &mut self,
        access_password: &[u8; 4],
        lock_payload: &LockPayload,
    ) -> Result<(), UhfError> {
        let lock_bytes = lock_payload.to_bytes();

        let mut params = Vec::with_capacity(7);
        params.extend_from_slice(access_password);
        params.extend_from_slice(&lock_bytes);

        let response = self.exec(&Self::create_command(Self::LOCK_TAG, &params))?;

        if response.len() >= 7
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::LOCK_TAG
            && response[5] == 0x00
        {
            Ok(())
        } else if response.len() >= 6 && response[5] != 0x00 {
            Err(UhfError::InvalidResponse(format!(
                "Lock failed with error code: 0x{:02X}",
                response[5]
            )))
        } else {
            Err(UhfError::InvalidResponse("Failed to lock tag".into()))
        }
    }

    /// Kill (permanently disable) a tag
    ///
    /// # Arguments
    /// * `kill_password` - 4-byte kill password (must be non-zero and match tag's kill password)
    ///
    /// # Warning
    /// This operation is irreversible! The tag will be permanently disabled.
    pub fn kill_tag(&mut self, kill_password: &[u8; 4]) -> Result<(), UhfError> {
        // Kill password must be non-zero per EPC Gen2 spec
        if kill_password == &[0, 0, 0, 0] {
            return Err(UhfError::InvalidParameter(
                "Kill password must be non-zero".into(),
            ));
        }

        let response = self.exec(&Self::create_command(Self::KILL_TAG, kill_password))?;

        if response.len() >= 7
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::KILL_TAG
            && response[5] == 0x00
        {
            Ok(())
        } else if response.len() >= 6 && response[5] != 0x00 {
            Err(UhfError::InvalidResponse(format!(
                "Kill failed with error code: 0x{:02X}",
                response[5]
            )))
        } else {
            Err(UhfError::InvalidResponse("Failed to kill tag".into()))
        }
    }

    // ========================================================================
    // Phase 4: Advanced/Vendor Commands
    // ========================================================================

    /// Start inventory and store results in reader buffer
    ///
    /// This command starts tag inventory and stores the results in the reader's
    /// internal buffer. Use `get_buffer_data()` to retrieve results and
    /// `clear_buffer()` to clear the buffer.
    ///
    /// # Arguments
    /// * `rounds` - Number of inventory rounds to perform
    pub fn inventory_buffer(&mut self, rounds: u16) -> Result<(), UhfError> {
        if rounds == 0 {
            return Err(UhfError::InvalidParameter(
                "Inventory rounds must be at least 1".into(),
            ));
        }

        let rounds_msb = (rounds >> 8) as u8;
        let rounds_lsb = (rounds & 0xFF) as u8;

        let response = self.exec(&Self::create_command(
            Self::INVENTORY_BUFFER,
            &[0x22, rounds_msb, rounds_lsb],
        ))?;

        if response.len() >= 7
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::INVENTORY_BUFFER
            && response[5] == 0x00
        {
            Ok(())
        } else {
            Err(UhfError::InvalidResponse("Failed to start inventory buffer".into()))
        }
    }

    /// Get tag data stored in reader buffer
    ///
    /// Returns the tags stored in the buffer from a previous `inventory_buffer()` call.
    pub fn get_buffer_data(&mut self) -> Result<Vec<TagInfo>, UhfError> {
        let response = self.exec(&Self::create_command(Self::GET_BUFFER_DATA, &[]))?;

        if response.len() < 7
            || response[0] != Self::HEADER
            || response[1] != Self::RESP_TYPE_NOTIFICATION
            || response[2] != Self::GET_BUFFER_DATA
        {
            return Err(UhfError::InvalidResponse("Invalid buffer response".into()));
        }

        let data_len = ((response[3] as usize) << 8) | (response[4] as usize);
        if data_len == 1 && response[5] == 0x00 {
            // Empty buffer
            return Ok(Vec::new());
        }

        // Parse tags from buffer - format varies by reader
        // This is a simplified implementation
        let mut tags = Vec::new();
        let mut offset = 5;

        while offset + 4 < response.len() - 2 {
            // Try to parse tag entries
            // Format: RSSI (1) + PC (2) + EPC (variable)
            if response[offset] == 0x00 || offset >= data_len + 5 {
                break;
            }

            let rssi = response[offset];
            let epc_len = if offset + 3 < response.len() {
                // Assume 12-byte EPC by default
                12usize.min(response.len() - offset - 3)
            } else {
                break;
            };

            if offset + 3 + epc_len <= response.len() {
                let epc_bytes = &response[offset + 3..offset + 3 + epc_len];
                tags.push(TagInfo {
                    epc: bytes_to_hex(epc_bytes),
                    rssi,
                });
                offset += 3 + epc_len;
            } else {
                break;
            }
        }

        Ok(tags)
    }

    /// Clear the reader's tag buffer
    pub fn clear_buffer(&mut self) -> Result<(), UhfError> {
        let response = self.exec(&Self::create_command(Self::CLEAR_BUFFER, &[]))?;

        if response.len() >= 7
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::CLEAR_BUFFER
            && response[5] == 0x00
        {
            Ok(())
        } else {
            Err(UhfError::InvalidResponse("Failed to clear buffer".into()))
        }
    }

    /// Get current RF link profile
    pub fn get_rf_link_profile(&mut self) -> Result<RfLinkProfile, UhfError> {
        let response = self.exec(&Self::create_command(Self::GET_RF_LINK_PROFILE, &[]))?;

        if response.len() >= 8
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::GET_RF_LINK_PROFILE
            && response[3] == 0x00
            && response[4] == 0x01
        {
            RfLinkProfile::try_from(response[5]).map_err(|_| {
                UhfError::InvalidResponse(format!(
                    "Unknown RF link profile: 0x{:02X}",
                    response[5]
                ))
            })
        } else {
            Err(UhfError::InvalidResponse("Failed to get RF link profile".into()))
        }
    }

    /// Set RF link profile
    ///
    /// This configures the modulation and data rate settings.
    pub fn set_rf_link_profile(&mut self, profile: RfLinkProfile) -> Result<(), UhfError> {
        let response = self.exec(&Self::create_command(
            Self::SET_RF_LINK_PROFILE,
            &[profile as u8],
        ))?;

        if response.len() >= 7
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::SET_RF_LINK_PROFILE
            && response[5] == 0x00
        {
            Ok(())
        } else {
            Err(UhfError::InvalidResponse("Failed to set RF link profile".into()))
        }
    }

    /// Get current reader sensitivity
    ///
    /// Returns the sensitivity value (higher = less sensitive, lower = more sensitive).
    pub fn get_reader_sensitivity(&mut self) -> Result<u8, UhfError> {
        let response = self.exec(&Self::create_command(Self::GET_READER_SENSITIVITY, &[]))?;

        if response.len() >= 8
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::GET_READER_SENSITIVITY
            && response[3] == 0x00
            && response[4] == 0x01
        {
            Ok(response[5])
        } else {
            Err(UhfError::InvalidResponse("Failed to get reader sensitivity".into()))
        }
    }

    /// Set reader sensitivity
    ///
    /// # Arguments
    /// * `sensitivity` - Sensitivity value (valid range typically 0-31, check reader specs)
    pub fn set_reader_sensitivity(&mut self, sensitivity: u8) -> Result<(), UhfError> {
        let response = self.exec(&Self::create_command(
            Self::SET_READER_SENSITIVITY,
            &[sensitivity],
        ))?;

        if response.len() >= 7
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::SET_READER_SENSITIVITY
            && response[5] == 0x00
        {
            Ok(())
        } else {
            Err(UhfError::InvalidResponse("Failed to set reader sensitivity".into()))
        }
    }

    /// Block Permalock - permanently lock memory blocks
    ///
    /// # Arguments
    /// * `access_password` - 4-byte access password
    /// * `mem_bank` - Memory bank (User bank only, typically)
    /// * `block_ptr` - Starting block number
    /// * `block_range` - Number of blocks to lock
    /// * `mask` - 16-bit mask specifying which blocks to permalock
    ///
    /// # Warning
    /// This operation is irreversible!
    pub fn block_permalock(
        &mut self,
        access_password: &[u8; 4],
        mem_bank: MemoryBank,
        block_ptr: u8,
        block_range: u8,
        mask: u16,
    ) -> Result<(), UhfError> {
        let mask_msb = (mask >> 8) as u8;
        let mask_lsb = (mask & 0xFF) as u8;

        let mut params = Vec::with_capacity(9);
        params.extend_from_slice(access_password);
        params.push(mem_bank as u8);
        params.push(block_ptr);
        params.push(block_range);
        params.push(mask_msb);
        params.push(mask_lsb);

        let response = self.exec(&Self::create_command(Self::BLOCK_PERMALOCK, &params))?;

        if response.len() >= 7
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::BLOCK_PERMALOCK
            && response[5] == 0x00
        {
            Ok(())
        } else if response.len() >= 6 && response[5] != 0x00 {
            Err(UhfError::InvalidResponse(format!(
                "Block permalock failed with error code: 0x{:02X}",
                response[5]
            )))
        } else {
            Err(UhfError::InvalidResponse("Failed to block permalock".into()))
        }
    }

    // NXP-specific commands

    /// NXP Read Protect - enable read protection on NXP UCODE tags
    ///
    /// # Arguments
    /// * `access_password` - 4-byte access password
    pub fn nxp_read_protect(&mut self, access_password: &[u8; 4]) -> Result<(), UhfError> {
        let response = self.exec(&Self::create_command(Self::NXP_READ_PROTECT, access_password))?;

        if response.len() >= 7
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::NXP_READ_PROTECT
            && response[5] == 0x00
        {
            Ok(())
        } else if response.len() >= 6 && response[5] != 0x00 {
            Err(UhfError::InvalidResponse(format!(
                "NXP Read Protect failed with error code: 0x{:02X}",
                response[5]
            )))
        } else {
            Err(UhfError::InvalidResponse("Failed to enable NXP read protect".into()))
        }
    }

    /// NXP Reset Read Protect - disable read protection on NXP UCODE tags
    ///
    /// # Arguments
    /// * `access_password` - 4-byte access password
    pub fn nxp_reset_read_protect(&mut self, access_password: &[u8; 4]) -> Result<(), UhfError> {
        let response = self.exec(&Self::create_command(
            Self::NXP_RESET_READ_PROTECT,
            access_password,
        ))?;

        if response.len() >= 7
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::NXP_RESET_READ_PROTECT
            && response[5] == 0x00
        {
            Ok(())
        } else if response.len() >= 6 && response[5] != 0x00 {
            Err(UhfError::InvalidResponse(format!(
                "NXP Reset Read Protect failed with error code: 0x{:02X}",
                response[5]
            )))
        } else {
            Err(UhfError::InvalidResponse("Failed to reset NXP read protect".into()))
        }
    }

    /// NXP Change EAS - enable or disable EAS (Electronic Article Surveillance) on NXP tags
    ///
    /// # Arguments
    /// * `access_password` - 4-byte access password
    /// * `enabled` - true to enable EAS, false to disable
    pub fn nxp_change_eas(&mut self, access_password: &[u8; 4], enabled: bool) -> Result<(), UhfError> {
        let mut params = Vec::with_capacity(5);
        params.extend_from_slice(access_password);
        params.push(if enabled { 0x01 } else { 0x00 });

        let response = self.exec(&Self::create_command(Self::NXP_CHANGE_EAS, &params))?;

        if response.len() >= 7
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::NXP_CHANGE_EAS
            && response[5] == 0x00
        {
            Ok(())
        } else if response.len() >= 6 && response[5] != 0x00 {
            Err(UhfError::InvalidResponse(format!(
                "NXP Change EAS failed with error code: 0x{:02X}",
                response[5]
            )))
        } else {
            Err(UhfError::InvalidResponse("Failed to change NXP EAS".into()))
        }
    }

    /// NXP EAS Alarm - check for EAS alarm on NXP tags
    ///
    /// Returns true if an EAS-enabled tag is detected.
    pub fn nxp_eas_alarm(&mut self) -> Result<bool, UhfError> {
        let response = self.exec(&Self::create_command(Self::NXP_EAS_ALARM, &[]))?;

        if response.len() >= 7 && response[0] == Self::HEADER {
            if response[1] == Self::RESP_TYPE_TAG {
                // Tag with EAS detected
                Ok(true)
            } else if response[1] == Self::RESP_TYPE_NOTIFICATION {
                // No EAS tag detected or error
                Ok(response[5] == 0x00)
            } else {
                Err(UhfError::InvalidResponse("Unexpected response type".into()))
            }
        } else {
            Err(UhfError::InvalidResponse("Failed to check EAS alarm".into()))
        }
    }

    /// NXP Change Config - modify NXP UCODE tag configuration
    ///
    /// # Arguments
    /// * `access_password` - 4-byte access password
    /// * `config_word` - 16-bit configuration word
    pub fn nxp_change_config(
        &mut self,
        access_password: &[u8; 4],
        config_word: u16,
    ) -> Result<(), UhfError> {
        let config_msb = (config_word >> 8) as u8;
        let config_lsb = (config_word & 0xFF) as u8;

        let mut params = Vec::with_capacity(6);
        params.extend_from_slice(access_password);
        params.push(config_msb);
        params.push(config_lsb);

        let response = self.exec(&Self::create_command(Self::NXP_CHANGE_CONFIG, &params))?;

        if response.len() >= 7
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::NXP_CHANGE_CONFIG
            && response[5] == 0x00
        {
            Ok(())
        } else if response.len() >= 6 && response[5] != 0x00 {
            Err(UhfError::InvalidResponse(format!(
                "NXP Change Config failed with error code: 0x{:02X}",
                response[5]
            )))
        } else {
            Err(UhfError::InvalidResponse("Failed to change NXP config".into()))
        }
    }

    /// Impinj Monza QT - configure QT settings on Impinj Monza tags
    ///
    /// # Arguments
    /// * `access_password` - 4-byte access password
    /// * `qt_control` - QT control settings
    /// * `read` - true to read current QT settings, false to write
    pub fn impinj_monza_qt(
        &mut self,
        access_password: &[u8; 4],
        qt_control: &QtControl,
        read: bool,
    ) -> Result<u8, UhfError> {
        let rw_flag = if read { 0x00 } else { 0x01 };

        let mut params = Vec::with_capacity(6);
        params.extend_from_slice(access_password);
        params.push(rw_flag);
        params.push(qt_control.to_byte());

        let response = self.exec(&Self::create_command(Self::IMPINJ_MONZA_QT, &params))?;

        if response.len() >= 8
            && response[0] == Self::HEADER
            && response[1] == Self::RESP_TYPE_NOTIFICATION
            && response[2] == Self::IMPINJ_MONZA_QT
            && response[5] == 0x00
        {
            Ok(response[6])
        } else if response.len() >= 6 && response[5] != 0x00 {
            Err(UhfError::InvalidResponse(format!(
                "Impinj Monza QT failed with error code: 0x{:02X}",
                response[5]
            )))
        } else {
            Err(UhfError::InvalidResponse("Failed Impinj Monza QT operation".into()))
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
