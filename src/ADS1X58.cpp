#include "ADS1X58.h"

/**
 * @brief Constructs an ADS1X58 ADC interface object with specified reference voltage.
 * 
 * Initializes the SPI communication settings, configures the chip select pin,
 * and sets the reference voltage for conversion calculations.
 * The CS pin is set to OUTPUT mode and driven HIGH (inactive) by default.
 * 
 * @param s Pointer to SPIClass instance for SPI communication
 * @param type ADC type (ADS1258 for 24-bit or ADS1158 for 16-bit)
 * @param csPin Chip select pin number
 * @param vref Reference voltage in volts (used for voltage conversion). Can also be set later using setVref().
 * @param SPIclock SPI clock frequency in Hz (default: 4000000)
 */
ADS1X58::ADS1X58(SPIClass *s, ADC_TYPE type, int csPin, float vref, int SPIclock)
    : _spi(s), csPin(csPin), adcType(type), vref(vref), mySPISettings(SPIclock, MSBFIRST, SPI_MODE0)
{
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
}

/**
 * @brief Constructs an ADS1X58 ADC interface object and auto-measures reference voltage.
 * 
 * Initializes the SPI communication settings, configures the chip select pin,
 * and automatically measures the internal reference voltage using measVref().
 * The CS pin is set to OUTPUT mode and driven HIGH (inactive) by default.
 * If the measurement fails, vref is set to 1.0V as a fallback.
 * 
 * @param s Pointer to SPIClass instance for SPI communication
 * @param type ADC type (ADS1258 for 24-bit or ADS1158 for 16-bit)
 * @param csPin Chip select pin number
 * @param SPIclock SPI clock frequency in Hz (default: 4000000)
 * 
 * @note Requires a short delay after construction to allow ADC to stabilize before measurement
 */
ADS1X58::ADS1X58(SPIClass *s, ADC_TYPE type, int csPin, int SPIclock)
    : _spi(s), csPin(csPin), adcType(type), vref(1.0f), mySPISettings(SPIclock, MSBFIRST, SPI_MODE0)
{
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
    
    // Allow ADC to stabilize before measuring vref
    delay(50);
    
    // Measure and set vref automatically
    float measuredVref = measVref();
    if (measuredVref != WRONG_FLOAT && measuredVref > 0.0f) {
        vref = measuredVref;
    }
}

/**
 * @brief Waits for the ADS1X58 ADC to complete a conversion and updates channel data or times out.
 * 
 * This function blocks until the conversion status of the specified channel is complete.
 * It polls the device status and updates the provided channel data structure with the
 * conversion result.
 * 
 * @param chanData Pointer to an ADS1X58_ChanData structure that will be updated with the conversion status and result.
 * 
 * @return bool Returns true if the status check was successful and successfully read, false otherwise.
 * 
 * 
 * @see ADS1X58_ChanData
 */
bool ADS1X58::waitForReadChannelDataCommand(ADS1X58_ChanData* chanData)
{
    if (chanData == nullptr) return false;
    
    uint32_t startTime = millis();
    while (millis() - startTime < TIMEOUT_MS) {
        readChannelDataCommand(chanData);
        if (_isNewData(chanData)) return true;
        delayMicroseconds(100);
    }
    return false; // timeout
}

/**
 * @brief Checks if chanData is a result of new conversion.
 * 
 * @param chanData Pointer to the channel data structure to check. Must not be nullptr.
 * 
 * @return true if the channel data is valid, the read was successful, and new 
 *         status data is available; false otherwise.
 * 
 * @note Returns false if chanData is nullptr or if rawStatus equals 0xFF 
 *       (indicating invalid or unavailable status).
 * 
 * @see STATUS_NEW
 * @see ADS1X58_ChanData
 */
bool ADS1X58::_isNewData(ADS1X58_ChanData* chanData)
{
    // invalid or no status byte available
    if (chanData == nullptr || (chanData->rawStatus == 0xFF)) return false; 

    // new data available and successful read
    return chanData->success && (chanData->rawStatus & STATUS_NEW); 
}

/**
 * @brief Reads a single register value from the ADS1X58.
 * 
 * Performs a single register read operation via SPI using the CMD_REG_READ command.
 * 
 * @param regAddr Register address to read (use REG_* constants)
 * @return uint8_t The 8-bit value stored in the specified register
 */
uint8_t ADS1X58::readRegister(uint8_t regAddr)
{
    _spi->beginTransaction(mySPISettings);
    digitalWrite(csPin, LOW);
    
    // Send read command with register address
    _spi->transfer(CMD_REG_READ | regAddr);
    uint8_t value = _spi->transfer(0x00);
    
    digitalWrite(csPin, HIGH);
    _spi->endTransaction();

    return value;
}

/**
 * @brief Reads multiple consecutive registers from the ADS1X58 in a single transaction.
 * 
 * Uses the CMD_MUL_EN bit to enable multiple register access, reading consecutive
 * registers starting from the specified address.
 * 
 * @param startRegAddr Starting register address
 * @param values Pointer to buffer where register values will be stored
 * @param count Number of consecutive registers to read
 * @return uint8_t* Pointer to the buffer containing the read values
 */
uint8_t* ADS1X58::readRegistersMultiple(uint8_t startRegAddr, uint8_t* values, uint8_t count)
{
    _spi->beginTransaction(mySPISettings);
    digitalWrite(csPin, LOW);
    
    // Send read command with MUL bit set for consecutive register access
    _spi->transfer(CMD_REG_READ | CMD_MUL_EN | startRegAddr);
    
    // Read all consecutive registers in one transaction
    for (uint8_t i = 0; i < count; i++) {
        values[i] = _spi->transfer(0x00);
    }
    
    digitalWrite(csPin, HIGH);
    _spi->endTransaction();
    
    return values;  // returns pointer to the buffer
}

/**
 * @brief Writes a value to a single register in the ADS1X58.
 * 
 * Performs a single register write operation via SPI using the CMD_REG_WRITE command.
 * 
 * @param regAddr Register address to write (use REG_* constants)
 * @param val 8-bit value to write to the register
 */
void ADS1X58::writeRegister(uint8_t regAddr, uint8_t val)
{
    _spi->beginTransaction(mySPISettings);
    digitalWrite(csPin, LOW);
    
    // Send write command with register address
    _spi->transfer(CMD_REG_WRITE | regAddr);
    _spi->transfer(val);
    
    digitalWrite(csPin, HIGH);
    _spi->endTransaction();
}

/**
 * @brief Writes values to multiple consecutive registers in a single transaction.
 * 
 * Uses the CMD_MUL_EN bit to enable multiple register access, writing to consecutive
 * registers starting from the specified address.
 * 
 * @param startRegAddr Starting register address
 * @param values Pointer to array of values to write
 * @param count Number of consecutive registers to write
 */
void ADS1X58::writeRegistersMultiple(uint8_t startRegAddr, const uint8_t* values, uint8_t count)
{
    _spi->beginTransaction(mySPISettings);
    digitalWrite(csPin, LOW);
    
    // Send write command with MUL bit set for consecutive register access
    _spi->transfer(CMD_REG_WRITE | CMD_MUL_EN | startRegAddr);
    
    // Write all consecutive registers in one transaction
    for (uint8_t i = 0; i < count; i++) {
        _spi->transfer(values[i]);
    }
    
    digitalWrite(csPin, HIGH);
    _spi->endTransaction();
}

/**
 * @brief Updates specific bits in a register using read-modify-write pattern.
 * 
 * Reads the current register value, applies the mask to preserve unaffected bits,
 * updates the masked bits with the new value, and writes the result back.
 * Pattern: new_value = (current & ~mask) | (val & mask)
 * 
 * @param regAddr Register address to update
 * @param mask Bit mask indicating which bits to modify (1 = modify, 0 = preserve)
 * @param val New value for the masked bits (needs to be at corresponding positions)
 * @return uint8_t The previous register value before modification
 */
uint8_t ADS1X58::updateRegister(uint8_t regAddr, uint8_t mask, uint8_t val)
{
    uint8_t current = readRegister(regAddr);
    uint8_t updated = (current & ~mask) | (val & mask);
    writeRegister(regAddr, updated);
    return current;  // returns the previous register value
}

/**
 * @brief Configures the ADC for single-ended input mode on all 16 channels only.
 * 
 * Sets AUTO_SCAN mode and enables all single-ended inputs (AIN0-AIN15) by
 * configuring MUXSG0 and MUXSG1 registers. Clears differential inputs (MUXDIF)
 * and system readings (SYSRED).
 */
void ADS1X58::enableAllSingleEndedInputs()
{
    // Enable AUTO_SCAN mode (should be default anyway, but just to be sure)
    updateRegister(REG_CONFIG0, MASK_CONFIG0_MUXMOD, MUXMODE_AUTO_SCAN);
    
    // Enable all 16 single-ended inputs: MUXSG0 contains AIN7-AIN0 and MUXSG1 contains AIN15-AIN8
    uint8_t configValues[5] = {0x00, 0x00, 0xFF, 0xFF, 0x00};  // MUXSCH, MUXDIF, MUXSG0, MUXSG1, SYSRED
    writeRegistersMultiple(REG_MUXSCH, configValues, 5);
}

/**
 * @brief Disables all input channels by clearing MUXSCH, MUXDIF, MUXSG0, MUXSG1, and SYSRED registers.
 * 
 * This effectively prevents any channel from being selected for measurement.
 */
void ADS1X58::disableAllChannels()
{
    // Disable all channels by clearing MUXSCH, MUXDIF, MUXSG0, MUXSG1, and SYSRED
    uint8_t configValues[5] = {0x00, 0x00, 0x00, 0x00, 0x00};  // MUXSCH, MUXDIF, MUXSG0, MUXSG1, SYSRED
    writeRegistersMultiple(REG_MUXSCH, configValues, 5);
}

/**
 * @brief Sets the inactivity reset timer duration.
 * 
 * Configures the SPIRST bit in CONFIG0 register.
 * Controls how long the device waits before resetting the channel multiplexer
 * during auto-scan mode inactivity.
 * 
 * @param setting Inactivity timer setting (SPIRST_LONG for 4096 fclk, SPIRST_SHORT for shorter duration)
 */
void ADS1X58::setInactivityResetTimer(ADS1X58_SPIRST setting)
{
    updateRegister(REG_CONFIG0, MASK_CONFIG0_SPIRST, setting);
}

/**
 * @brief Sets the channel scanning mode.
 * 
 * Configures the MUXMOD bit in CONFIG0 register.
 * Selects between auto-scan mode (cycles through enabled channels) and
 * fixed-channel mode (reads single specified channel repeatedly).
 * 
 * @param setting Channel mode setting (MUXMODE_AUTO_SCAN or MUXMODE_FIXED_CHANNEL)
 */
void ADS1X58::setChannelMode(ADS1X58_MUXMODE setting)
{
    updateRegister(REG_CONFIG0, MASK_CONFIG0_MUXMOD, setting);
}

/**
 * @brief Sets the multiplexer bypass configuration.
 * 
 * Configures the BYPAS bit in CONFIG0 register.
 * Selects between internal MUX (routes channels to ADC input) and
 * external bypass (routes external analog source directly to ADC).
 * 
 * @param setting Bypass setting (BYPAS_INTERNAL for internal MUX, BYPAS_EXTERNAL for external)
 */
void ADS1X58::setBypassMode(ADS1X58_BYPAS setting)
{
    updateRegister(REG_CONFIG0, MASK_CONFIG0_BYPAS, setting);
}

/**
 * @brief Enables or disables the clock output on the CLKOUT pin.
 * 
 * Configures the CLKENB bit in CONFIG0 register.
 * When enabled, the internal clock is output on the CLKOUT pin for external synchronization.
 * When disabled, the CLKOUT pin remains inactive.
 * 
 * @param setting Clock output setting (CLKENB_EN to enable, CLKENB_DIS to disable)
 */
void ADS1X58::setClockOutput(ADS1X58_CLKENB setting)
{
    updateRegister(REG_CONFIG0, MASK_CONFIG0_CLKENB, setting);
}

/**
 * @brief Enables or disables the chopping function.
 * 
 * Configures the CHOP bit in CONFIG0 register.
 * Chopping reduces offset and 1/f noise by alternately sampling the analog input
 * and its inverse, then subtracting. Can reduce noise at the cost of conversion speed.
 * 
 * @param setting Chopping setting (CHOP_EN to enable, CHOP_DIS to disable)
 */
void ADS1X58::setChopMode(ADS1X58_CHOP setting)
{
    updateRegister(REG_CONFIG0, MASK_CONFIG0_CHOP, setting);
}

/**
 * @brief Sets the status byte enable/disable in ADC output.
 * 
 * Configures the STAT bit in CONFIG0 register and updates the internal statusByteEnabled flag.
 * When enabled (STAT_EN), each data read will include a status byte containing channel ID,
 * new data flag, overvoltage flag, and supply warning.
 * When disabled (STAT_DIS), data reads will only contain conversion data without status information.
 * 
 * @param setting Status byte setting (STAT_EN to enable, STAT_DIS to disable)
 */
void ADS1X58::setStatusByte(ADS1X58_STAT setting)
{
    updateRegister(REG_CONFIG0, MASK_CONFIG0_STAT, setting);
    statusByteEnabled = (setting == STAT_EN);
}

/**
 * @brief Sets the device idle/power consumption mode.
 * 
 * Configures the IDLMOD bit in CONFIG1 register.
 * Selects between sleep mode (lowest power consumption, longer wake time)
 * and standby mode (intermediate power consumption and wake time).
 * 
 * @param setting Idle mode setting (IDLMOD_SLEEP for low power, IDLMOD_STANDBY for standby)
 */
void ADS1X58::setIdleMode(ADS1X58_IDLMOD setting)
{
    updateRegister(REG_CONFIG1, MASK_CONFIG1_IDLMOD, setting);
}

/**
 * @brief Sets the conversion delay time.
 * 
 * Configures the DLY bits in CONFIG1 register.
 * Controls the delay between pulse conversions in auto-scan mode.
 * Delay is specified in units of 128/fclk periods (fclk typically 16 MHz).
 * 
 * @param setting Delay setting (DLY_0 for no delay, DLY_1 through DLY_48 for progressive delays)
 */
void ADS1X58::setDelay(ADS1X58_DLY setting)
{
    updateRegister(REG_CONFIG1, MASK_CONFIG1_DLY, setting);
}

/**
 * @brief Sets the sensor bias current source magnitude.
 * 
 * Configures the SBCS bits in CONFIG1 register.
 * Provides selectable bias current to sensor elements for resistance measurement.
 * Allows choice between no bias (OFF), small bias (1.5 µA), or large bias (24 µA).
 * 
 * @param setting Sensor bias setting (SBCS_OFF to disable, SBCS_SMALL or SBCS_LARGE to enable)
 */
void ADS1X58::setSensorBiasCurrent(ADS1X58_SBCS setting)
{
    updateRegister(REG_CONFIG1, MASK_CONFIG1_SBCS, setting);
}

/**
 * @brief Sets the ADC conversion data rate.
 * 
 * Configures the DRATE bits in CONFIG1 register.
 * Controls the output data rate in samples per second (SPS).
 * Actual data rate depends on channel mode and chop settings.
 * Higher rates provide faster conversion but increased noise; lower rates reduce noise.
 * 
 * @param setting Data rate setting (DRATE_00 through DRATE_11, DRATE_11 is default 23739/125000 SPS)
 */
void ADS1X58::setDataRate(ADS1X58_DRATE setting)
{
    updateRegister(REG_CONFIG1, MASK_CONFIG1_DRATE, setting);
}

/**
 * @brief Reads ADC data bytes from SPI and stores the result in the provided channel data structure.
 * 
 * This method reads either 24-bit (ADS1258) or 16-bit (ADS1158) ADC conversion results from the SPI bus.
 * It optionally reads a status byte if enabled, performs sign extension for two's complement representation,
 * and stores the raw data value in the channel data structure.
 * 
 * @param chanData Pointer to the ADS1X58_ChanData structure where the read data will be stored.
 *                 If nullptr, the function returns without performing any operations.
 * 
 * @details
 * - If statusByteEnabled is true, reads the status byte first via SPI transfer.
 * - For ADS1258 (24-bit): Reads three bytes and performs sign extension on bit 23.
 * - For ADS1158 (16-bit): Reads two bytes and performs sign extension on bit 15.
 * - Sets chanData->success to true upon successful completion.
 * - Automatically handles two's complement sign extension based on ADC type.
 * 
 * @note The function assumes valid SPI communication and does not perform error checking
 *       beyond null pointer validation.
 * 
 * @see ADS1X58_ChanData
 * @see ADC_TYPE
 */
void ADS1X58::_readDataBytesToResult(ADS1X58_ChanData* chanData)
{
    if (chanData == nullptr) return; // nothing to do

    if (statusByteEnabled) chanData->rawStatus = _spi->transfer(0x00); 

    uint8_t b2 = 0, b1 = 0, b0 = 0;
    uint32_t code = 0;
    if (adcType == ADC_TYPE::ADS1258) { // ADS1258: 24-bit (three bytes)
        b2 = _spi->transfer(0x00);
        b1 = _spi->transfer(0x00);
        b0 = _spi->transfer(0x00);

        code = (uint32_t(b2) << 16) | (uint32_t(b1) << 8) | uint32_t(b0);
        if (b2 & 0x80) code |= 0xFF000000u; // sign-extend 24-bit (two complement)

    } else { // ADS1158: 16-bit (two bytes)
        b1 = _spi->transfer(0x00);
        b0 = _spi->transfer(0x00);

        code = (uint32_t(b1) << 8) | uint32_t(b0);
        if (b1 & 0x80) code |= 0xFFFF0000u; // sign-extend 16-bit (two complement)
    }
    chanData->rawData = static_cast<int32_t>(code); // cast to signed int32_t
    chanData->success = true;
}

/**
 * @brief Reads channel data using direct read mode.
 * 
 * Direct read mode is auto-detected by the device when DIN is held constant (all 0s or all 1s)
 * during the first 3 SCLK transitions. No command byte is sent. The user must monitor
 * DRDY externally before calling this function.
 * 
 * @param chanData Pointer to structure where conversion result will be stored
 * 
 * @note Requires external DRDY monitoring by the user
 * @see _readDataBytesToResult
 */
void ADS1X58::readChannelDataDirect(ADS1X58_ChanData* chanData)
{
    digitalWrite(csPin, LOW);
    _spi->beginTransaction(mySPISettings);
    
    // TODO: Verify if CMD_DATA_READ_DIRECT command byte is needed or if we can read directly
    // _spi->transfer(CMD_DATA_READ_DIRECT);
    _readDataBytesToResult(chanData);

    _spi->endTransaction();
    digitalWrite(csPin, HIGH);
}

/**
 * @brief Reads channel data using command read mode.
 * 
 * Sends CMD_DATA_READ_COMMAND with MUL_EN bit set, then reads the conversion data.
 * This method does not wait for DRDY or check for new data availability.
 * 
 * @param chanData Pointer to structure where conversion result will be stored
 * 
 * @see waitForReadChannelDataCommand for a version that polls for new data
 * @see _readDataBytesToResult
 */
void ADS1X58::readChannelDataCommand(ADS1X58_ChanData* chanData)
{
    digitalWrite(csPin, LOW);
    _spi->beginTransaction(mySPISettings);
    
    _spi->transfer(CMD_DATA_READ_COMMAND | CMD_MUL_EN);// Send data read command
    _readDataBytesToResult(chanData);

    _spi->endTransaction();
    digitalWrite(csPin, HIGH);
}

/**
 * @brief Converts ADC code to voltage.
 * 
 * Applies the conversion formula: voltage = (code / RANGE) * vref / gain
 * The range is ADC-type dependent (0x780000 for ADS1258, 0x7800 for ADS1158).
 * 
 * @param code Signed 32-bit ADC conversion code (two's complement)
 * @return float Voltage value in volts
 * 
 * @note _gain can be adjusted using compensateGain() for gain error compensation.
 */
float ADS1X58::codeToVoltage(int32_t code)
{
    // then convert to voltage based on reference voltage and ADC type
    if (adcType == ADC_TYPE::ADS1258) {
        return ((float) code / (float)ADS1258_RANGE) * vref / _gain;
    } else {
        return ((float) code / (float)ADS1158_RANGE) * vref / _gain;
    }
}

/**
 * @brief Reads channel data and converts to voltage in a single call.
 * 
 * Convenience function that performs readChannelDataCommand followed by
 * codeToVoltage conversion.
 * 
 * @return float Voltage value in volts
 */
float ADS1X58::readVoltage() {
    ADS1X58_ChanData chanData;
    readChannelDataCommand(&chanData);
    return codeToVoltage(chanData.rawData);
}

/**
 * @brief Reads channel data and converts to voltage in a single call using the direct command.
 * 
 * Convenience function that performs readChannelDataDirect followed by
 * codeToVoltage conversion.
 * 
 * @return float Voltage value in volts
 * @note Requires external DRDY monitoring by the user
 */
float ADS1X58::readVoltageDirect() {
    ADS1X58_ChanData chanData;
    readChannelDataDirect(&chanData);
    return codeToVoltage(chanData.rawData);
}

/**
 * @brief Performs internal system measurement (VCC, GAIN, VREF, TEMP, OFFSET).
 * 
 * Saves current device configuration, sets up the device for system measurement,
 * performs conversion, validates CHID, and restores original configuration.
 * This ensures measurements don't corrupt user settings.
 * 
 * @param chanData Pointer to structure where measurement result will be stored
 * @param sysredBitMask Bit mask for SYSRED register selecting measurement type
 * @param checkChid Expected CHID value for validation (default 0xFF = no check)
 * 
 * @note Disables chopping and enables status byte during measurement
 * @note Validates CHID is in range 0x18-0x1D (system measurement channels)
 */
void ADS1X58::_internalMeas(ADS1X58_ChanData* chanData, uint8_t sysredBitMask, uint8_t checkChid=0xFF)
{
    // Save current register values
    uint8_t savedRegs[5];
    readRegistersMultiple(REG_MUXSCH, savedRegs, 5);
    
    // Clear all MUX registers and activate the selectable bit in SYSRED
    uint8_t tempRegs[5] = {0x00, 0x00, 0x00, 0x00, sysredBitMask};
    writeRegistersMultiple(REG_MUXSCH, tempRegs, 5);
    
    // disable chopping and enable status byte
    uint8_t combined_setting = CHOP_DIS | STAT_EN;
    uint8_t savedRegConfig0 = updateRegister(REG_CONFIG0, MASK_CONFIG0_CHOP | MASK_CONFIG0_STAT, combined_setting);

    // Small delay to allow measurement to settle
    delay(10);

    startPulseConversion(); // easiest way to start conversion for single measurement
    waitForReadChannelDataCommand(chanData);

    // reset chopping and status byte to previous state
    updateRegister(REG_CONFIG0, MASK_CONFIG0_CHOP | MASK_CONFIG0_STAT, savedRegConfig0);

    // Restore all registers to original values
    writeRegistersMultiple(REG_MUXSCH, savedRegs, 5);
    
    // Validate CHID (bits [4:0]) is within 0x18–0x1D; otherwise mark as failed
    if (chanData.rawStatus != 0xFF) {
        uint8_t chid = chanData.rawStatus & STATUS_CHID;
        if (chid < 0x18 || chid > 0x1D) chanData.success = false; // invalid CHID
        if (checkChid != 0xFF && chid != checkChid) chanData.success = false; // CHID does not match expected
    }
}

/**
 * @brief Converts internal measurement code to physical units.
 * 
 * Applies ADC-type-specific conversion factors for VCC, GAIN, and VREF measurements.
 * TEMP and OFFSET return error value as they require special handling.
 * 
 * @param code Signed 32-bit measurement code
 * @param measType Type of internal measurement (VCC, GAIN, VREF) TEMP and OFFSET should be handled separately
 * @return float Converted value in appropriate units, or WRONG_FLOAT on error
 */
float ADS1X58::_internalCodeConversion(int32_t code, INTERNAL_TYPE_CONV measType)
{
    switch (measType){
    case INTERNAL_TYPE_CONV::TEMP: return WRONG_FLOAT; // TEMP handled differently
    case INTERNAL_TYPE_CONV::OFFSET: return WRONG_FLOAT; // OFFSET handled differently
    }

    float conversionFactor;
    if (adcType == ADC_TYPE::ADS1258) {
        switch (measType) {
            case INTERNAL_TYPE_CONV::VCC:  conversionFactor = 786432.0f;  break;
            case INTERNAL_TYPE_CONV::GAIN: conversionFactor = 7864320.0f; break;
            case INTERNAL_TYPE_CONV::VREF: conversionFactor = 786432.0f;  break;
        }
    } else { // ADS1158
        switch (measType) {
            case INTERNAL_TYPE_CONV::VCC:  conversionFactor = 3072.0f;   break;
            case INTERNAL_TYPE_CONV::GAIN: conversionFactor = 30720.0f;  break;
            case INTERNAL_TYPE_CONV::VREF: conversionFactor = 3072.0f;   break;
        }
    }
    
    return static_cast<float>(code) / conversionFactor;
}

/**
 * @brief Measures ADC internal offset voltage.
 * 
 * Performs internal offset measurement and converts to voltage.
 * 
 * @return float Offset voltage in volts, or WRONG_FLOAT on failure
 */
float ADS1X58::measOffset()
{
    ADS1X58_ChanData chanData;
    _internalMeas(&chanData, SYSRED_OFFSET, CHID_OFFSET);
    if (!chanData.success) return WRONG_FLOAT; // measurement failed
    return codeToVoltage(chanData.rawData);
}

/**
 * @brief Measures ADC supply voltage (VCC).
 * 
 * Performs internal VCC measurement using device's internal monitoring.
 * 
 * @return float VCC voltage in volts, or WRONG_FLOAT on failure
 */
float ADS1X58::measVcc()
{
    ADS1X58_ChanData chanData;
    _internalMeas(&chanData, SYSRED_VCC, CHID_VCC);
    if (!chanData.success) return WRONG_FLOAT; // measurement failed
    return _internalCodeConversion(chanData.rawData, INTERNAL_TYPE_CONV::VCC);
}

/**
 * @brief Measures ADC gain error.
 * 
 * Performs internal gain measurement to determine ADC gain accuracy.
 * 
 * @return float Gain value (unitless), or WRONG_FLOAT on failure
 */
float ADS1X58::measGain()
{
    ADS1X58_ChanData chanData;
    _internalMeas(&chanData, SYSRED_GAIN, CHID_GAIN);
    if (!chanData.success) return WRONG_FLOAT; // measurement failed
    return _internalCodeConversion(chanData.rawData, INTERNAL_TYPE_CONV::GAIN);
}

/**
 * @brief Sets gain compensation factor for voltage conversions.
 * 
 * Updates the internal gain factor used in codeToVoltage conversions.
 * Rejects zero or negative values.
 * 
 * @param newGain New gain value (must be positive)
 */
void ADS1X58::compensateGain(int newGain)
{  
    if (newGain == 0 || newGain < 0) return;
    _gain = static_cast<float>(newGain);
}

/**
 * @brief Measures ADC internal reference voltage (VREF).
 * 
 * Performs internal VREF measurement using device's internal monitoring.
 * 
 * @return float VREF voltage in volts, or WRONG_FLOAT on failure
 */
float ADS1X58::measVref()
{
    ADS1X58_ChanData chanData;
    _internalMeas(&chanData, SYSRED_REF, CHID_REF);
    if (!chanData.success) return WRONG_FLOAT; // measurement failed
    return _internalCodeConversion(chanData.rawData, INTERNAL_TYPE_CONV::VREF);
}

/**
 * @brief Measures ADC die temperature in degrees Celsius.
 * 
 * Performs internal temperature measurement and converts using the formula:
 * tempC = (tempMicroVolts - 168) / tempCoeff + 25
 * 
 * @param tempCoeff Temperature coefficient in µV/°C (default: 563 for PCB-mounted)
 *                  Use ADS1X58_TEMP_COEFF_FREE (394) for free-air operation (unlikely)
 * @return float Temperature in degrees Celsius, or WRONG_FLOAT on failure
 */
float ADS1X58::measTempC(float tempCoeff)
{
    ADS1X58_ChanData chanData;
    _internalMeas(&chanData, SYSRED_TEMP, CHID_TEMP);
    if (!chanData.success) return WRONG_FLOAT; // measurement failed
    float tempMicroVolts = codeToVoltage(chanData.rawData) / 1e6f; // convert to microVolts;
    return (tempMicroVolts - 168.0f) / tempCoeff + 25.0f; // per datasheet
}

/**
 * @brief Sets the reference voltage for the ADS1X58 ADC.
 * 
 * @param newVref The new reference voltage value in volts. Must be a positive value.
 *                If a non-positive value is provided, the method returns without
 *                updating the reference voltage.
 * 
 * @return void
 * 
 * @note This method silently ignores invalid (non-positive) input values.
 *       Consider using error handling if validation feedback is required.
 */
void ADS1X58::setVref(float newVref)
{
    if (newVref <= 0.0f) return; // invalid
    vref = newVref;
}

/**
 * @brief Enables or disables a single single-ended input channel in auto-scan mode.
 * 
 * Configures the multiplexer to enable or disable the specified single-ended input (AIN0-AIN15)
 * without affecting other channels.
 * 
 * @param channel Input channel number (0-15 corresponding to AIN0-AIN15).
 *                Invalid channels (>15) are ignored.
 * @param enable Set to 1 to enable the channel, 0 to disable it.
 * 
 * @note Ensure the ADC is set to auto-scan mode via setChannelMode(MUXMODE_AUTO_SCAN) before calling.
 * 
 * @example
 * adc.setSingleEndedChannel(5, 1);   // Enable AIN5
 * adc.setSingleEndedChannel(5, 0);   // Disable AIN5
 */
void ADS1X58::setSingleEndedChannel(uint8_t channel, bool state)
{
    if (channel > 15) return; // ignore invalid channels
    if (channel > 7) { // MUXSG1 (bits map to AIN15:AIN8)
        uint8_t bitPosition = channel - 8;
        updateRegister(REG_MUXSG1, 1 << bitPosition, (uint8_t)state << bitPosition);
    } else { // MUXSG0 (bits map to AIN7:AIN0)
        uint8_t bitPosition = channel;
        updateRegister(REG_MUXSG0, 1 << bitPosition, (uint8_t)state << bitPosition);
    }
}

/**
 * @brief Enables or disables a differential input pair in auto-scan mode.
 * 
 * Configures the multiplexer to enable or disable the specified differential input pair (DIFF0-DIFF7)
 * without affecting other differential pairs.
 * 
 * @param diffPair Differential pair number (0-7):
 *        - DIFF0: AIN0 (positive) vs AIN1 (negative)
 *        - DIFF1: AIN2 (positive) vs AIN3 (negative)
 *        - DIFF2: AIN4 (positive) vs AIN5 (negative)
 *        - DIFF3: AIN6 (positive) vs AIN7 (negative)
 *        - DIFF4: AIN8 (positive) vs AIN9 (negative)
 *        - DIFF5: AIN10 (positive) vs AIN11 (negative)
 *        - DIFF6: AIN12 (positive) vs AIN13 (negative)
 *        - DIFF7: AIN14 (positive) vs AIN15 (negative)
 * @param state Set to 1 to enable the differential pair, 0 to disable it.
 * 
 * @note Ensure the ADC is set to auto-scan mode via setChannelMode(MUXMODE_AUTO_SCAN) before calling.
 *       Invalid diffPair values (>7) are ignored.
 * 
 * @example
 * adc.setDifferentialChannel(0, 1);   // Enable DIFF0 (AIN0 vs AIN1)
 * adc.setDifferentialChannel(4, 0);   // Disable DIFF4 (AIN8 vs AIN9)
 */
void ADS1X58::setDifferentialChannel(uint8_t diffPair, bool state)
{
    if (diffPair > 7) return; // ignore invalid differential pairs
    
    // MUXDIF (all 8 differential pairs in one register, bits map to DIFF7:DIFF0)
    uint8_t bitPosition = diffPair;
    updateRegister(REG_MUXDIF, 1 << bitPosition, (uint8_t)state << bitPosition);
}

/**
 * @brief Configures differential input channels for fixed-channel mode measurement.
 * 
 * Sets up the MUXSCH register to measure a differential pair in fixed-channel mode.
 * The differential measurement (AINP - AINN) will be repeated until a different
 * channel pair is selected or the ADC switches to auto-scan mode.
 * 
 * @param AINP Positive input channel number (0-3 corresponding to AINP0-AINP3)
 *             Invalid channels (>3) are ignored.
 * @param AINN Negative input channel number (0-3 corresponding to AINN0-AINN3)
 *             Invalid channels (>3) are ignored.
 * @param state Set to 1 (or true) to enable measurement on this channel pair,
 *              0 (or false) to disable/clear the channel configuration.
 * 
 * @note Ensure the ADC is set to fixed-channel mode via setChannelMode(MUXMODE_FIXED_CHANNEL) before calling.
 *       AINP and AINN must be different channel numbers for proper differential measurement.
 * 
 * @example
 * adc.setChannelMode(MUXMODE_FIXED_CHANNEL);  // Switch to fixed-channel mode
 * adc.setFixedChannel(0, 1, 1);               // Measure AI0 (positive) vs AIN1 (negative)
 */
void ADS1X58::setFixedChannel(uint8_t AINP, uint8_t AINN, bool state)
{
    if (AINP > 3 || AINN > 3) return; // ignore invalid channels

    uint8_t AINP_position = AINP << 4; // upper nibble
    uint8_t AINN_position = AINN; // lower nibble
    updateRegister(REG_MUXSCH, 1 << AINP_position, (uint8_t)state << AINP_position);
    updateRegister(REG_MUXSCH, 1 << AINN_position, (uint8_t)state << AINN_position);
}

/**
 * @brief Converts a channel ID (CHID) to a human-readable channel name.
 *
 * Helps decode the CHID field from the status byte by returning a descriptive
 * string for differential, single-ended, and system measurement channels.
 * Unknown IDs return "UNKNOWN".
 */
const char* ADS1X58::chidToName(uint8_t chid)
{
    switch (chid) {
        case CHID_DIFF0:  return "DIFF0";  // AIN0-AIN1
        case CHID_DIFF1:  return "DIFF1";  // AIN2-AIN3
        case CHID_DIFF2:  return "DIFF2";  // AIN4-AIN5
        case CHID_DIFF3:  return "DIFF3";  // AIN6-AIN7
        case CHID_DIFF4:  return "DIFF4";  // AIN8-AIN9
        case CHID_DIFF5:  return "DIFF5";  // AIN10-AIN11
        case CHID_DIFF6:  return "DIFF6";  // AIN12-AIN13
        case CHID_DIFF7:  return "DIFF7";  // AIN14-AIN15

        case CHID_AIN0:   return "AIN0";
        case CHID_AIN1:   return "AIN1";
        case CHID_AIN2:   return "AIN2";
        case CHID_AIN3:   return "AIN3";
        case CHID_AIN4:   return "AIN4";
        case CHID_AIN5:   return "AIN5";
        case CHID_AIN6:   return "AIN6";
        case CHID_AIN7:   return "AIN7";
        case CHID_AIN8:   return "AIN8";
        case CHID_AIN9:   return "AIN9";
        case CHID_AIN10:  return "AIN10";
        case CHID_AIN11:  return "AIN11";
        case CHID_AIN12:  return "AIN12";
        case CHID_AIN13:  return "AIN13";
        case CHID_AIN14:  return "AIN14";
        case CHID_AIN15:  return "AIN15";

        case CHID_OFFSET: return "OFFSET";
        case CHID_VCC:    return "VCC";
        case CHID_TEMP:   return "TEMP";
        case CHID_GAIN:   return "GAIN";
        case CHID_REF:    return "REF";
        default:          return "UNKNOWN";
    }
}

/**
 * @brief Sets GPIO pin mode (input or output).
 *
 * Configures the GPIOC register to set individual GPIO pins (0-7) as either
 * input or output. Invalid pins (>7) are ignored.
 *
 * @param pin GPIO pin number (0-7)
 * @param mode GPIO mode setting (GPIO_OUTPUT for output, GPIO_INPUT for input)
 *
 * @example
 * adc.GPIOpinMode(0, GPIO_OUTPUT);  // Configure GPIO0 as output
 * adc.GPIOpinMode(1, GPIO_INPUT);   // Configure GPIO1 as input
 */
void ADS1X58::GPIOpinMode(uint8_t pin, ADS1X58_GPIO_MODE mode)
{
    if (pin > 7) return; // ignore invalid pins
    updateRegister(REG_GPIOC, 1 << pin, (uint8_t)mode << pin);
}

/**
 * @brief Sets GPIO pin output value (high or low).
 *
 * Configures the GPIOD register to set individual GPIO pins (0-7) output state.
 * Only affects pins configured as outputs. Invalid pins (>7) are ignored.
 *
 * @param pin GPIO pin number (0-7)
 * @param state GPIO state setting (GPIO_HIGH to set high, GPIO_LOW to set low)
 *
 * @example
 * adc.GPIOdigitalWrite(0, GPIO_HIGH);  // Set GPIO0 high
 * adc.GPIOdigitalWrite(0, GPIO_LOW);   // Set GPIO0 low
 */
void ADS1X58::GPIOdigitalWrite(uint8_t pin, ADS1X58_GPIO_STATE state)
{
    if (pin > 7) return; // ignore invalid pins
    updateRegister(REG_GPIOD, 1 << pin, (uint8_t)state << pin);
}

/**
 * @brief Reads GPIO pin input value.
 *
 * Reads the GPIOD register to get the current state of a GPIO pin (0-7).
 * Returns true for high, false for low. Invalid pins (>7) return false.
 *
 * @param pin GPIO pin number (0-7)
 * @return bool Current pin state (true for high, false for low)
 *
 * @example
 * bool pinState = adc.GPIOdigitalRead(1);  // Read GPIO1 state
 */
bool ADS1X58::GPIOdigitalRead(uint8_t pin)
{
    if (pin > 7) return false; // invalid pin
    uint8_t gpiod = readRegister(REG_GPIOD);
    return (gpiod >> pin) & 0b1; // just return the selected channel
}

/**
 * @brief Triggers a Single-Shot Conversion.
 * 
 * Sends CMD_PULSE_CONVERT to trigger a conversion.
 * Useful for single-shot measurements.
 * Conversion result must be read separately.
 */
void ADS1X58::startPulseConversion()
{
    digitalWrite(csPin, LOW);
    _spi->beginTransaction(mySPISettings);
    
    _spi->transfer(CMD_PULSE_CONVERT);
    
    _spi->endTransaction();
    digitalWrite(csPin, HIGH);
}




