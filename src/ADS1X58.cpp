#include "ADS1X58.h"

ADS1X58::ADS1X58(SPIClass *s, int cs, int drdy, ADC_TYPE type)
    : _spi(s), csPin(cs), drdyPin(drdy)
{
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
    pinMode(drdyPin, INPUT);
}

uint8_t ADS1X58::readRegister(uint8_t reg)
{
    digitalWrite(csPin, LOW);
    _spi->beginTransaction(mySPISettings);
    
    // Send read command with register address
    _spi->transfer(CMD_REG_READ | reg);
    uint8_t value = _spi->transfer(0x00);
    
    _spi->endTransaction();
    digitalWrite(csPin, HIGH);
    
    return value;
}

void ADS1X58::writeRegister(uint8_t reg, uint8_t val)
{
    digitalWrite(csPin, LOW);
    _spi->beginTransaction(mySPISettings);
    
    // Send write command with register address
    _spi->transfer(CMD_REG_WRITE | reg);
    _spi->transfer(val);
    
    _spi->endTransaction();
    digitalWrite(csPin, HIGH);
}

void ADS1X58::updateRegister(uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t current = readRegister(reg);
    uint8_t updated = (current & ~mask) | (val & mask);
    writeRegister(reg, updated);
}