# M5Stack UHF RFID Unit (JRD-4035) Documentation

## Device Information

- **Model**: JRD-4035
- **Manufacturer**: M5Stack
- **SKU**: U107
- **Protocol**: Serial UART communication
- **Baud Rate**: 115200 bps
- **Standard**: EPCglobal UHF Class 1 Gen 2 / ISO 18000-6C

## Documentation Files

- `Unit-RFID-UHF-Protocol-EN.pdf` - Official protocol specification from M5Stack

## Protocol Overview

### Frame Structure

All commands follow this structure:
```
[Header] [Type] [Command] [PL_High] [PL_Low] [Parameters...] [Checksum] [End]
```

- **Header**: `0xBB` (Start byte)
- **Type**: `0x00` for commands, `0x02` for responses
- **Command**: Specific command code
- **PL**: Parameter Length (2 bytes, high/low)
- **Parameters**: Command-specific data
- **Checksum**: Sum of bytes from Type to last Parameter byte (1 byte)
- **End**: `0x7E` (End byte)

### Checksum Calculation

```
Checksum = (Type + Command + PL_High + PL_Low + Parameter[0] + ... + Parameter[n-1]) & 0xFF
```

## Known Commands

Based on the working Python implementation:

### Get Hardware Version
```
Command: BB 00 03 00 01 00 04 7E
```
- Type: `0x00`
- Command: `0x03`
- Parameter Length: `0x0001` (1 byte)
- Parameter: `0x00` (hardware)
- Checksum: `0x04`

**Response**: ASCII string in bytes 5 to (len-1)

### Get Firmware Version
```
Command: BB 00 03 00 01 01 05 7E
```
- Type: `0x00`
- Command: `0x03`
- Parameter Length: `0x0001` (1 byte)
- Parameter: `0x01` (firmware)
- Checksum: `0x05`

**Response**: ASCII string in bytes 5 to (len-1)

### Single Poll (Read One Tag)
```
Command: BB 00 22 00 00 22 7E
```
- Type: `0x00`
- Command: `0x22`
- Parameter Length: `0x0000` (no parameters)
- Checksum: `0x22`

**Response Format**:
```
BB 02 22 00 [len] [rssi] [ant] [freq] [EPC data...] [checksum] 7E
```
- Byte 0-1: Header (`0xBB 0x02`)
- Byte 2: Command echo (`0x22`)
- Byte 3: Always `0x00`
- Byte 4: Data length (total length including RSSI, antenna, frequency, EPC)
- Byte 5: RSSI (signal strength)
- Byte 6: Antenna number
- Byte 7: Frequency index
- Byte 8+: EPC data (length = data_len - 5)
- Last-1: Checksum
- Last: End byte (`0x7E`)

### Set TX Power
```
Command: BB 00 B6 00 02 [power_high] [power_low] [checksum] 7E
```
- Type: `0x00`
- Command: `0xB6`
- Parameter Length: `0x0002` (2 bytes)
- Parameters: Power in dBm * 100 (e.g., 26 dBm = 2600 = `0x0A 0x28`)
- Valid range: 18-26 dBm (1800-2600)
- Checksum: `(0x00 + 0xB6 + 0x00 + 0x02 + power_high + power_low) & 0xFF`

## Resources

- **Official Documentation**: https://docs.m5stack.com/en/unit/uhf_rfid
- **GitHub Repository**: https://github.com/m5stack/M5Unit-UHF-RFID
- **Product Page**: https://shop.m5stack.com/products/uhf-rfid-unit-jrd-4035

## Additional Notes

- The module has a built-in ceramic antenna
- Maximum transmission power: 100mW
- Effective reading distance: >1.5 meters
- Supports multiple polling modes and tag operations (read, write, lock)
- For complete command set, refer to the official protocol PDF
