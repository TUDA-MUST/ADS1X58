#include "ADS1X58.h"

volatile bool drdyIsReady = false; 

void drdyISR() 
{
	drdyIsReady = true; // true when low / falling
}

bool waitForDRDY()
{
    uint32_t startTime = millis();
    while (drdyIsReady == false) {
        if (millis() - startTime > TIMEOUT_MS) {
            return false; // timeout
        }
        delayMicroseconds(100);
    }
    return true;
}

/**
 * @brief Waits for the ADS1X58 ADC to complete a conversion and updates channel data or times out.
 * 
 * This function blocks until the conversion status of the specified channel is complete.
 * It polls the device status and updates the provided channel data structure with the
 * conversion result.
 * 
 * @param chanData Pointer to an ADS1X58_ChanData structure that contains channel
 *                 configuration and will be updated with the conversion status and result.
 * 
 * @return bool Returns true if the status check was successful and successfully read, false otherwise.
 * 
 * 
 * @see ADS1X58_ChanData
 */
bool waitForReadChannelDataCommand(ADS1X58_ChanData* chanData)
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
 * @brief Checks if new status data is ready for a given channel.
 * 
 * Verifies that the channel data is valid and contains new data that was 
 * successfully read. The function performs two checks:
 * 1. Ensures the channel pointer is valid and a status byte is available
 * 2. Confirms the read operation was successful and new data is present
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
bool _isNewData(ADS1X58_ChanData* chanData)
{
    // invalid or no status byte available
    if (chanData == nullptr || (chanData->rawStatus == 0xFF)) return false; 

    // new data available and successful read
    return chanData->success && (chanData->rawStatus & STATUS_NEW); 
}

ADS1X58::ADS1X58(SPIClass *s, int cs, int drdy, ADC_TYPE type, SPIclock)
    : _spi(s), csPin(cs), drdyPin(drdy), adcType(type), mySPISettings(SPIclock, MSBFIRST, SPI_MODE0)
{
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);

    if (drdyPin!=-1) { // allows for the user to not use DRDY pin
        pinMode(drdyPin, INPUT);
        attachInterrupt(digitalPinToInterrupt(drdyPin), drdyISR, FALLING); 
    }
}

uint8_t ADS1X58::readRegister(uint8_t reg)
{
    _spi->beginTransaction(mySPISettings);
    digitalWrite(csPin, LOW);
    
    // Send read command with register address
    _spi->transfer(CMD_REG_READ | reg);
    uint8_t value = _spi->transfer(0x00);
    
    digitalWrite(csPin, HIGH);
    _spi->endTransaction();

    return value;
}

uint8_t* ADS1X58::readRegistersMultiple(uint8_t startReg, uint8_t* buffer, uint8_t count)
{
    _spi->beginTransaction(mySPISettings);
    digitalWrite(csPin, LOW);
    
    // Send read command with MUL bit set for consecutive register access
    _spi->transfer(CMD_REG_READ | CMD_MUL_EN | startReg);
    
    // Read all consecutive registers in one transaction
    for (uint8_t i = 0; i < count; i++) {
        buffer[i] = _spi->transfer(0x00);
    }
    
    digitalWrite(csPin, HIGH);
    _spi->endTransaction();
    
    return buffer;  // returns pointer to the buffer
}

void ADS1X58::writeRegister(uint8_t reg, uint8_t val)
{
    _spi->beginTransaction(mySPISettings);
    digitalWrite(csPin, LOW);
    
    // Send write command with register address
    _spi->transfer(CMD_REG_WRITE | reg);
    _spi->transfer(val);
    
    digitalWrite(csPin, HIGH);
    _spi->endTransaction();
}

void ADS1X58::writeRegistersMultiple(uint8_t startReg, const uint8_t* values, uint8_t count)
{
    digitalWrite(csPin, LOW);
    _spi->beginTransaction(mySPISettings);
    
    // Send write command with MUL bit set for consecutive register access
    _spi->transfer(CMD_REG_WRITE | CMD_MUL_EN | startReg);
    
    // Write all consecutive registers in one transaction
    for (uint8_t i = 0; i < count; i++) {
        _spi->transfer(values[i]);
    }
    
    _spi->endTransaction();
    digitalWrite(csPin, HIGH);
}

uint8_t ADS1X58::updateRegister(uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t current = readRegister(reg);
    uint8_t updated = (current & ~mask) | (val & mask);
    writeRegister(reg, updated);
    return current;  // returns the previous register value
}

void ADS1X58::enableSingleEndedInputsOnly()
{
    // should be default anyway, but just to be sure:
    // Enable AUTO_SCAN mode
    updateRegister(REG_CONFIG0, MASK_CONFIG0_MUXMOD, ADS1X58_MUXMODE::AUTO_SCAN);
    
    // Enable all 8 single-ended inputs (MUXSG0 and MUXSG1)
    uint8_t configValues[5] = {0x00, 0x00, 0xFF, 0xFF, 0x00};  // MUXSCH, MUXDIF, MUXSG0, MUXSG1, SYSRED
    writeRegistersMultiple(REG_MUXSCH, configValues, 5);
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
    chanData->rawData = static_cast<int32_t>(code); // cast to signed integer
    chanData->success = true;
}

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

void ADS1X58::readChannelDataCommand(ADS1X58_ChanData* chanData)
{
    digitalWrite(csPin, LOW);
    _spi->beginTransaction(mySPISettings);
    
    _spi->transfer(CMD_DATA_READ_COMMAND | CMD_MUL_EN);// Send data read command
    _readDataBytesToResult(chanData);

    _spi->endTransaction();
    digitalWrite(csPin, HIGH);
}

float ADS1X58::codeToVoltage(int32_t code)
{
    // then convert to voltage based on reference voltage and ADC type
    if (adcType == ADC_TYPE::ADS1258) {
        return ((float) code / ADS1258_RANGE) * vref / _gain;
    } else {
        return ((float) code / ADS1158_RANGE) * vref / _gain;
    }
}

float ADS1X58::readVoltage() {
    ADS1X58_ChanData chanData;
    readChannelDataCommand(&chanData);
    return codeToVoltage(chanData.rawData);
}

void ADS1X58::_internalMeas(ADS1X58_ChanData* chanData, uint8_t sysredBitMask, uint8_t checkChid=0xFF)
{
    // Save current register values
    uint8_t savedRegs[5];
    readRegistersMultiple(REG_MUXSCH, savedRegs, 5);
    
    // Clear all MUX registers and activate the selectable bit in SYSRED
    uint8_t tempRegs[5] = {0x00, 0x00, 0x00, 0x00, sysredBitMask};
    writeRegistersMultiple(REG_MUXSCH, tempRegs, 5);
    
    // disable chopping and enable status byte
    uint8_t combined_setting = static_cast<uint8_t>(ADS1X58_CHOP::DIS) | static_cast<uint8_t>(ADS1X58_STAT::EN);
    uint8_t savedRegConfig0 = updateRegister(REG_CONFIG0, MASK_CONFIG0_CHOP | MASK_CONFIG0_STAT, combined_setting);

    // Small delay to allow measurement to settle
    delay(10);

    pulseConversion(); // easiest way to start conversion for single measurement
    waitForReadChannelDataCommand(&chanData);

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

float ADS1X58::_internalCodeConversion(int32_t code, INTERNAL_TYPE_CONV measType)
{
    float conversionFactor;
    
    if (adcType == ADC_TYPE::ADS1258) {
        switch (measType) {
            case INTERNAL_MEAS_TYPE::VCC:  conversionFactor = 786432.0f;  break;
            case INTERNAL_MEAS_TYPE::GAIN: conversionFactor = 7864320.0f; break;
            case INTERNAL_MEAS_TYPE::VREF: conversionFactor = 786432.0f;  break;
        }
    } else { // ADS1158
        switch (measType) {
            case INTERNAL_MEAS_TYPE::VCC:  conversionFactor = 3072.0f;   break;
            case INTERNAL_MEAS_TYPE::GAIN: conversionFactor = 30720.0f;  break;
            case INTERNAL_MEAS_TYPE::VREF: conversionFactor = 3072.0f;   break;
        }
    }
    
    return static_cast<float>(code) / conversionFactor;
}

float ADS1X58::measOffset()
{
    ADS1X58_ChanData chanData;
    _internalMeas(&chanData, SYSRED_OFFSET, CHID_OFFSET);
    if (!chanData.success) return WRONG_FLOAT; // measurement failed
    return codeToVoltage(chanData.rawData);
}

float ADS1X58::measVcc()
{
    ADS1X58_ChanData chanData;
    _internalMeas(&chanData, SYSRED_VCC, CHID_VCC);
    if (!chanData.success) return WRONG_FLOAT; // measurement failed
    return _internalCodeConversion(chanData.rawData, INTERNAL_TYPE_CONV::VCC);
}

float ADS1X58::measGain()
{
    ADS1X58_ChanData chanData;
    _internalMeas(&chanData, SYSRED_GAIN, CHID_GAIN);
    if (!chanData.success) return WRONG_FLOAT; // measurement failed
    return _internalCodeConversion(chanData.rawData, INTERNAL_TYPE_CONV::GAIN);
}

void ADS1X58::compensateGain(int newGain)
{  
    if (newGain == 0 || newGain < 0) return;
    _gain = static_cast<float>(newGain);
}

float ADS1X58::measVref()
{
    ADS1X58_ChanData chanData;
    _internalMeas(&chanData, SYSRED_REF, CHID_REF);
    if (!chanData.success) return WRONG_FLOAT; // measurement failed
    return _internalCodeConversion(chanData.rawData, INTERNAL_TYPE_CONV::VREF);
}

float ADS1X58::measTempC(float tempCoeff = ADS1X58_TEMP_COEFF_PCB)
{
    ADS1X58_ChanData chanData;
    _internalMeas(&chanData, SYSRED_TEMP, CHID_TEMP);
    if (!chanData.success) return WRONG_FLOAT; // measurement failed
    tempMicroVolts = codeToVoltage(chanData.rawData) / 1e6f; // convert to microVolts;
    return (tempMicroVolts - 168.0f) / tempCoeff + 25.0f; // per datasheet
}

void ADS1X58::startPulseConversion()
{
    digitalWrite(csPin, LOW);
    _spi->beginTransaction(mySPISettings);
    
    _spi->transfer(CMD_PULSE_CONVERT);
    
    _spi->endTransaction();
    digitalWrite(csPin, HIGH);
}




