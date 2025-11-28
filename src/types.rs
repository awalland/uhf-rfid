//! Types for RFID operations

/// Information about a detected RFID tag
#[derive(Debug, Clone)]
pub struct TagInfo {
    pub epc: String,
    pub rssi: u8,
}

/// Memory bank selection for tag operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MemoryBank {
    /// Reserved memory bank (RFU)
    Reserved = 0x00,
    /// EPC memory bank
    Epc = 0x01,
    /// TID memory bank
    Tid = 0x02,
    /// User memory bank
    User = 0x03,
}

/// Target flag for Select command (per EPC Gen2)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum SelectTarget {
    /// Inventoried S0
    #[default]
    S0 = 0x00,
    /// Inventoried S1
    S1 = 0x01,
    /// Inventoried S2
    S2 = 0x02,
    /// Inventoried S3
    S3 = 0x03,
    /// SL flag
    Sl = 0x04,
}

/// Action for Select command (per EPC Gen2)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum SelectAction {
    /// Match: assert SL or inventoried→A, Non-match: deassert SL or inventoried→B
    #[default]
    Action0 = 0x00,
    /// Match: assert SL or inventoried→A, Non-match: do nothing
    Action1 = 0x01,
    /// Match: do nothing, Non-match: deassert SL or inventoried→B
    Action2 = 0x02,
    /// Match: negate SL or invert, Non-match: do nothing
    Action3 = 0x03,
    /// Match: deassert SL or inventoried→B, Non-match: assert SL or inventoried→A
    Action4 = 0x04,
    /// Match: deassert SL or inventoried→B, Non-match: do nothing
    Action5 = 0x05,
    /// Match: do nothing, Non-match: assert SL or inventoried→A
    Action6 = 0x06,
    /// Match: do nothing, Non-match: negate SL or invert
    Action7 = 0x07,
}

/// Select mode configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum SelectMode {
    /// Send Select command before every tag operation
    Always = 0x00,
    /// Do not send Select command before tag operations
    #[default]
    Disabled = 0x01,
    /// Send Select only before non-polling operations (Read, Write, Lock, Kill)
    NonPolling = 0x02,
}

/// Parameters for the Select command
#[derive(Debug, Clone)]
pub struct SelectParams {
    /// Target session/flag
    pub target: SelectTarget,
    /// Action to perform
    pub action: SelectAction,
    /// Memory bank to match against
    pub mem_bank: MemoryBank,
    /// Bit pointer (starting bit position in memory bank)
    pub pointer: u32,
    /// Mask data to match
    pub mask: Vec<u8>,
    /// Whether to enable truncation
    pub truncate: bool,
}

/// Operating region for the reader
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Region {
    /// China 900 MHz band
    China900 = 0x01,
    /// US band
    Us = 0x02,
    /// Europe band
    Europe = 0x03,
    /// China 800 MHz band
    China800 = 0x04,
    /// South Korea band
    Korea = 0x06,
}

impl Region {
    /// Get the base frequency for this region in MHz
    pub fn base_frequency(&self) -> f64 {
        match self {
            Region::China900 => 920.125,
            Region::Us => 902.25,
            Region::Europe => 865.1,
            Region::China800 => 840.125,
            Region::Korea => 917.1,
        }
    }

    /// Get the channel spacing for this region in MHz
    pub fn channel_spacing(&self) -> f64 {
        match self {
            Region::China900 => 0.25,
            Region::Us => 0.5,
            Region::Europe => 0.2,
            Region::China800 => 0.25,
            Region::Korea => 0.2,
        }
    }

    /// Calculate channel index from frequency
    pub fn channel_from_frequency(&self, freq_mhz: f64) -> u8 {
        ((freq_mhz - self.base_frequency()) / self.channel_spacing()) as u8
    }

    /// Calculate frequency from channel index
    pub fn frequency_from_channel(&self, channel: u8) -> f64 {
        (channel as f64) * self.channel_spacing() + self.base_frequency()
    }
}

impl TryFrom<u8> for Region {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x01 => Ok(Region::China900),
            0x02 => Ok(Region::Us),
            0x03 => Ok(Region::Europe),
            0x04 => Ok(Region::China800),
            0x06 => Ok(Region::Korea),
            _ => Err(()),
        }
    }
}

/// Query parameters for tag inventory (per EPC Gen2)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QueryParams {
    /// Sel field: which tags respond to Query
    pub sel: QuerySel,
    /// Session (S0-S3)
    pub session: QuerySession,
    /// Target (A or B)
    pub target: QueryTarget,
    /// Q value (0-15) - controls slot count
    pub q: u8,
}

impl Default for QueryParams {
    fn default() -> Self {
        Self {
            sel: QuerySel::All,
            session: QuerySession::S0,
            target: QueryTarget::A,
            q: 4,
        }
    }
}

impl QueryParams {
    /// Encode query parameters to 2-byte protocol format
    pub fn to_bytes(&self) -> [u8; 2] {
        // Format: DR(1) | M(2) | TRext(1) | Sel(2) | Session(2) | Target(1) | Q(4) | padding(3)
        // DR = 0 (DR=8), M = 0 (M=1), TRext = 1 (use pilot tone)
        let byte0 = 0x10 | ((self.sel as u8) << 2) | (self.session as u8);
        let byte1 = ((self.target as u8) << 7) | ((self.q & 0x0F) << 3);
        [byte0, byte1]
    }

    /// Decode query parameters from 2-byte protocol format
    pub fn from_bytes(bytes: [u8; 2]) -> Self {
        let sel = match (bytes[0] >> 2) & 0x03 {
            0 | 1 => QuerySel::All,
            2 => QuerySel::NotSl,
            3 => QuerySel::Sl,
            _ => QuerySel::All,
        };
        let session = match bytes[0] & 0x03 {
            0 => QuerySession::S0,
            1 => QuerySession::S1,
            2 => QuerySession::S2,
            3 => QuerySession::S3,
            _ => QuerySession::S0,
        };
        let target = if (bytes[1] >> 7) & 0x01 == 0 {
            QueryTarget::A
        } else {
            QueryTarget::B
        };
        let q = (bytes[1] >> 3) & 0x0F;

        Self {
            sel,
            session,
            target,
            q,
        }
    }
}

/// Sel field for Query command
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum QuerySel {
    /// All tags respond
    #[default]
    All = 0x00,
    /// Only tags with SL deasserted respond
    NotSl = 0x02,
    /// Only tags with SL asserted respond
    Sl = 0x03,
}

/// Session for Query command
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum QuerySession {
    #[default]
    S0 = 0x00,
    S1 = 0x01,
    S2 = 0x02,
    S3 = 0x03,
}

/// Target flag for Query command
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum QueryTarget {
    #[default]
    A = 0x00,
    B = 0x01,
}

impl PartialEq for TagInfo {
    fn eq(&self, other: &Self) -> bool {
        self.epc == other.epc
    }
}

/// Lock action for tag memory operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum LockAction {
    /// Unlock (read/write accessible without password)
    Unlock = 0x00,
    /// Lock (password required for read/write)
    Lock = 0x01,
    /// Permanent unlock (cannot be locked)
    PermUnlock = 0x02,
    /// Permanent lock (password always required, cannot be unlocked)
    PermLock = 0x03,
}

/// Lock target for specifying which memory area to lock
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum LockTarget {
    /// User memory bank
    User = 0x01,
    /// TID memory bank
    Tid = 0x02,
    /// EPC memory bank
    Epc = 0x03,
    /// Access password
    AccessPassword = 0x04,
    /// Kill password
    KillPassword = 0x05,
}

/// Lock mask and action payload
#[derive(Debug, Clone)]
pub struct LockPayload {
    /// Which memory area to apply the lock action to
    pub target: LockTarget,
    /// The lock action to perform
    pub action: LockAction,
}

impl LockPayload {
    /// Encode lock payload to 3-byte protocol format (mask and action)
    pub fn to_bytes(&self) -> [u8; 3] {
        // The lock payload format per EPC Gen2:
        // Mask (10 bits) + Action (10 bits) = 20 bits = 2.5 bytes, padded to 3 bytes
        //
        // Bits map to memory areas (2 bits per area):
        // Kill pwd | Access pwd | EPC | TID | User
        //
        // Mask bits: 1 = apply action, 0 = don't change
        // Action bits per area: bit1=permalock, bit0=lock
        //   00 = unlock, 01 = lock, 10 = perm unlock, 11 = perm lock

        let shift = match self.target {
            LockTarget::User => 0,
            LockTarget::Tid => 2,
            LockTarget::Epc => 4,
            LockTarget::AccessPassword => 6,
            LockTarget::KillPassword => 8,
        };

        // Set mask bits for target (2 bits)
        let mask: u16 = 0x03 << shift;

        // Set action bits for target
        let action: u16 = (self.action as u16) << shift;

        // Combine into 20-bit payload (mask in high 10 bits, action in low 10 bits)
        // Format: [mask_hi, mask_lo|action_hi, action_lo]
        let payload: u32 = ((mask as u32) << 10) | (action as u32);

        [
            ((payload >> 16) & 0xFF) as u8,
            ((payload >> 8) & 0xFF) as u8,
            (payload & 0xFF) as u8,
        ]
    }
}

/// RF link profile for modulation settings
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RfLinkProfile {
    /// FM0 40KHz link frequency
    Fm0_40kHz = 0xD0,
    /// FM0 400KHz link frequency
    Fm0_400kHz = 0xD1,
    /// Miller 4, 250KHz link frequency
    Miller4_250kHz = 0xD2,
    /// Miller 4, 300KHz link frequency
    Miller4_300kHz = 0xD3,
    /// Miller 2, 40KHz link frequency (Dense Reader Mode)
    Miller2_40kHzDrm = 0xD4,
}

impl TryFrom<u8> for RfLinkProfile {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0xD0 => Ok(RfLinkProfile::Fm0_40kHz),
            0xD1 => Ok(RfLinkProfile::Fm0_400kHz),
            0xD2 => Ok(RfLinkProfile::Miller4_250kHz),
            0xD3 => Ok(RfLinkProfile::Miller4_300kHz),
            0xD4 => Ok(RfLinkProfile::Miller2_40kHzDrm),
            _ => Err(()),
        }
    }
}

/// QT control settings for Impinj Monza tags
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QtControl {
    /// Short Range mode: reduces backscatter strength
    pub short_range: bool,
    /// QT persistence: false = temporary, true = permanent
    pub persistence: bool,
}

impl QtControl {
    /// Encode to protocol byte
    pub fn to_byte(&self) -> u8 {
        let mut byte = 0u8;
        if self.short_range {
            byte |= 0x01;
        }
        if self.persistence {
            byte |= 0x02;
        }
        byte
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
