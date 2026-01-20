# Copilot Instructions for ADS1X58

## Project Overview
**ADS1X58** is an Arduino-compatible library for communicating with the ADS1258 (24-bit) and ADS1158 (16-bit) analog-to-digital converters via SPI. The library abstracts low-level SPI protocol details into configuration register management and data reading operations.

## Architecture & Key Components

### Core Class: `ADS1X58`
Located in [src/ADS1X58.h](src/ADS1X58.h) and [src/ADS1X58.cpp](src/ADS1X58.cpp)

- **Constructor**: Accepts SPIClass pointer, chip-select (CS) pin, data-ready (DRDY) pin, and ADC type
- **Register-based control**: All device configuration happens via register read/write operations
- **Three-tier register API**:
  - `readRegister()` - read a single register value
  - `writeRegister()` - write a single register value
  - `updateRegister()` - read-modify-write pattern using bit masks (preferred for partial updates)

### Configuration System
The device uses **masked enums** for configuration. Each enum represents a bitfield with:
- Named states (e.g., `AUTO_SCAN`, `FIXED_CHANNEL`)
- A `MASK` value indicating which bits this setting affects

**Pattern example** (CONFIG0 register):
```cpp
enum class ADS1X58_CHAN_MODE {
    AUTO_SCAN      = 0b00100000,  // default
    FIXED_CHANNEL  = 0b00000000,
    MASK           = 0b00100000   // bit 5 controls this setting
};
```

Use `updateRegister()` with enum values to safely modify individual settings without affecting others.

### Device Configurations
Two independent configuration registers (CONFIG0 and CONFIG1):
- **CONFIG0** (0x00): Reset timer, channel mode, bypass mode, clock output, chop, status
- **CONFIG1** (0x01): Idle mode, delay, sensor bias current, data rate

Each register's settings are mapped to specific bit positions. Consult comments in the header for bit layout and default values.

## Development Patterns

### SPI Protocol Details
- **Bit rate**: 4 MHz (hardcoded in `mySPISettings`)
- **Mode**: SPI_MODE0 (CPOL=0, CPHA=0)
- **Byte order**: MSB first
- **Command format**: Upper 3 bits = command type, lower 5 bits = register address or parameter

**Command types** (upper 3 bits of first byte):
- `0b000` - Direct data read
- `0b001` - Data read with command
- `0b010` - Register read
- `0b011` - Register write
- `0b100` - Pulse convert
- `0b110` - Reset

### Register Access Pattern
All register operations follow this sequence:
1. Pull CS low
2. Begin SPI transaction (establishes clock settings)
3. Transfer command byte + operand byte(s)
4. Receive response byte(s)
5. End transaction
6. Pull CS high

The `readRegister()`, `writeRegister()`, and `updateRegister()` methods encapsulate this pattern.

### Handling Multiple Devices
The library is designed for single-device communication (no multi-register access via `CMD_MUL_EN` is currently implemented, though the constant is defined). Adding multi-device support would extend the register API.

## Important Conventions

1. **Bit manipulation**: Always use the `MASK` field from configuration enums when updating registers. The `updateRegister()` method handles the mask application.

2. **Register addresses**: Use named constants (`REG_CONFIG0`, `REG_MUXSCH`, etc.) rather than raw hex values for clarity.

3. **Default values**: Documented in enum comments (e.g., "default" next to common settings). Always verify defaults match your application needs.

4. **Dual ADC support**: Constructor accepts `ADC_TYPE` parameter (ADS1158 or ADS1258). Currently, type detection uses `REG_ID`, but pin configuration and register interpretation differ between types.

5. **GPIO and system monitoring**: `REG_GPIOD` (0x08) and `REG_SYSRED` (0x06) allow GPIO and system parameter access; these are less common but available.

## Common Tasks

- **Configure channel scanning**: Set `ADS1X58_CHAN_MODE::AUTO_SCAN` in CONFIG0, then select channels via `REG_MUXDIF` (differential pairs) or `REG_MUXSG0`/`REG_MUXSG1` (single-ended).
- **Change data rate**: Use `ADS1X58_DATA_RATE` enum with `updateRegister()` on CONFIG1.
- **Enter low-power mode**: Set `ADS1X58_IDLE_MODE::IDLMOD_SLEEP` in CONFIG1.
- **Reset device**: Send `CMD_RESET` command byte via SPI (not currently wrapped in a method).

## Known Issues & TODOs

- Header file has incomplete method declaration: `writeRegister()` is missing closing parenthesis (syntax error).
- Examples folder is empty; consider adding initialization and data-reading examples.
- No higher-level abstractions for channel configuration or data conversion.
