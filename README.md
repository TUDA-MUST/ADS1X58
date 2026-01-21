# ADS1X58
Arduino library for TI ADS1258 (24-bit) and ADS1158 (16-bit) SPI ADCs. Provides register-level access plus helpers for common configuration and measurements.

## Features
- Supports ADS1258 (24-bit) and ADS1158 (16-bit)
- Read/Write/Update register helpers (read-modify-write)
- Config setters for CONFIG0/CONFIG1 (channel mode, bypass, chop, clock out, status byte, idle mode, delay, bias current, data rate)
- Single read or pulse conversion flows
- Optional status byte handling

## Quick Start
```cpp
#include <ADS1X58.h>

// SPI instance, chip select, vref volts
ADS1X58 adc(&SPI, ADC_TYPE::ADS1258, 10, 2.5f);

void setup() {
	SPI.begin();
	adc.setStatusByte(ADS1X58::STAT_EN); // include status byte in reads
	adc.setChannelMode(ADS1X58::MUXMODE_AUTO_SCAN);
	adc.setDataRate(ADS1X58::DRATE_11); // fastest
}

void loop() {
	float v = adc.readVoltage();
	// use v ...
}
```
## ADC Measurements
### Conversion Modes
- **Auto-Scan Mode**: The ADC cycles through enabled channels automatically. Configure using `setChannelMode(MUXMODE_AUTO_SCAN)`, then enable channels via the MUX registers (e.g., write `MUXSG0/MUXSG1` for single-ended, or `MUXDIF` for differential).
- **Fixed-Channel Mode**: The ADC repeatedly converts a single channel. Configure using `setChannelMode(MUXMODE_FIXED_CHANNEL)` and select the input in `REG_MUXSCH` (AINP on bits [7:4], AINN on bits [3:0]).

Example:
```cpp
adc.setChannelMode(ADS1X58::MUXMODE_AUTO_SCAN);
// Enable AIN0..AIN15 single-ended in auto-scan (helper enables all)
adc.enableSingleEndedInputsOnly();

adc.setChannelMode(ADS1X58::MUXMODE_FIXED_CHANNEL);
// Select AIN0 (P) vs AIN1 (N): write to REG_MUXSCH via updateRegister()
// adc.updateRegister(ADS1X58::REG_MUXSCH, ADS1X58::MASK_MUXSCH_AINP|ADS1X58::MASK_MUXSCH_AINN, (0x0<<4)|0x1);
```

### Trigger Conversion
- **Start Pin**: Set the Start Pin to HIGH to trigger conversion. Will continuously take new samples if left HIGH. If ADC is in Auto-Scan mode it will automatically index the next channel. Use DRDY and call `readChannelDataCommand()` or `readChannelDataDirect()` to fetch the latest result or look at the status byte `NEW`, [see](#channel-data-format).
- **Pulse Conversion (single-shot)**: Send `CMD_PULSE_CONVERT` to start a single conversion; library helper: `startPulseConversion()`. After DRDY, read with one of the read modes.

### Read Data
- **Direct Data Read**:  Wait for DRDY and then use `readChannelDataDirect()`. No command byte (**faster**); just hold DIN steady for the first 3 SCLK edges, then clock out data. Risk of corruption if data is not read before the next DRDY.
- **Command Data Read**: Library uses this in `readChannelDataCommand()`. Can be read even during DRDY transistion (**safer**). Sends `CMD_DATA_READ_COMMAND` (with optional `CMD_MUL_EN`) and then clocks out data. 

## Channel Data Format
- Optional **status byte**, toggle with `setStatusByte(STAT_EN|STAT_DIS)`:
	- Bit7 `NEW` new data, Bit6 `OFV` overvoltage, Bit5 `SUPPLY` low AVDD warning, Bits[4:0] `CHID` channel ID
- **Data bytes**: MSB→LSB
	- ADS1258: 3 data bytes (24-bit two's complement); sign-extend from bit 23
	- ADS1158: 2 data bytes (16-bit two's complement); sign-extend from bit 15
- is handled via `ADS1X58_ChanData` struct

## Configuration Helpers
CONFIG0:
- `setInactivityResetTimer(SPIRST_LONG|SPIRST_SHORT)`
- `setChannelMode(MUXMODE_AUTO_SCAN|MUXMODE_FIXED_CHANNEL)`
- `setBypassMode(BYPAS_INTERNAL|BYPAS_EXTERNAL)`
- `setClockOutput(CLKENB_EN|CLKENB_DIS)`
- `setChopMode(CHOP_EN|CHOP_DIS)`
- `setStatusByte(STAT_EN|STAT_DIS)`

CONFIG1:
- `setIdleMode(IDLMOD_SLEEP|IDLMOD_STANDBY)`
- `setConversionDelay(DLY_0 ... DLY_48)`
- `setSensorBiasCurrent(SBCS_OFF|SBCS_SMALL|SBCS_LARGE)`
- `setDataRate(DRATE_00 ... DRATE_11)`

## Internal Measurements
Helpers exist to read internal measurements: `measVcc()`, `measGain()`, `measVref()`, `measTempC()`, `measOffset()`. These temporarily adjust configuration, perform a pulse conversion, validate CHID, then restore registers.

## License
MIT
