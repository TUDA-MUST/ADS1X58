#ifndef ADS1X58_H
#define ADS1X58_H

#include <Arduino.h>
#include <SPI.h>

enum class ADC_TYPE { // set ADC type 
	ADS1158, // 16-bit
	ADS1258 // 24-bit
};

/* CONFIG0 */
enum class ADS1X58_SPI_RESET_TIMER { // inactivity reset timer
    LONG      = 0b00000000,  // 4096 fclk (default)
    SHORT     = 0b01000000,
    MASK      = 0b01000000  
};
enum class ADS1X58_CHAN_MODE { // Channel mode operation
    AUTO_SCAN      = 0b00000000,  // auto-scan (default)
    FIXED_CHANNEL  = 0b00100000,
    MASK           = 0b00100000  
};
enum class ADS1X58_BYPAS { // internal or external connection from MUX output to ADC input
    BYPAS_INTERNAL  = 0b00010000,  // internal MUX (default)
    BYPAS_EXTERNAL  = 0b00000000,
    MASK            = 0b00010000
};
enum class ADS1X58_CLK_OUTPUT { // enables clock output pin
    CLKENB_EN      = 0b00001000,  // enabled (default)
    CLKENB_DIS     = 0b00000000,
    MASK           = 0b00001000,
};
enum class ADS1X58_CHOP { 
    CHOP_EN      = 0b00000100,  // enabled (default)
    CHOP_DIS     = 0b00000000,
    MASK         = 0b00000100
};
enum class ADS1X58_STATUS { 
    STAT_EN      = 0b00000010,  // enabled (default)
    STAT_DIS     = 0b00000000,
    MASK         = 0b00000010
};

/* CONFIG1 */
enum class ADS1X58_IDLE_MODE{ // power consumption vs. wake up time to re-enter run mode
	IDLMOD_SLEEP   = 0b10000000, // sleep mode (default)
    IDLMOD_STANDBY = 0b00000000, // standby mode
    MASK           = 0b10000000
};
enum class ADS1X58_DELAY { // time delay in 128/f_clk periods // f_clk most likely 16 MHz
    DLY_0        = 0b00000000, // off (default)
	DLY_1        = 0b00010000,
	DLY_2        = 0b00100000,
	DLY_4        = 0b00110000,
	DLY_8        = 0b01000000,
	DLY_16       = 0b01010000,
	DLY_32       = 0b01100000,
	DLY_48       = 0b01110000,
    MASK         = 0b01110000
};
enum class ADS1X58_SENSOR_BIAS_CURR { // sensor bias current source
    SBCS_OFF     = 0b00000000, // sensor vias current source off (default)
	SBCS_SMALL   = 0b00010000, // 1.5 microAmps
	SBCS_LARGE   = 0b00100000,  // 24 microAmps
    MASK         = 0b00110000
};
enum class ADS1X58_DATA_RATE { // total data rate in SPS: Auto-Scan Mode / Fixed-Channel Mode // assuming Chop = 0 and DLY[2:0] = 000
    DRATE_00     = 0b00000000,	// 1831  / 1953
	DRATE_01     = 0b00000001, // 6168  / 7813
	DRATE_10     = 0b00000010, // 15123 / 31250
	DRATE_11     = 0b00000011,	// 23739 / 125000 (default)
    MASK         = 0b00000011
};

class ADS1X58
{
    public:
		// Command Byte: C2 C1 C0 MUL A3 A2 A1 A0
        /* SPI Commands */
        static constexpr uint8_t CMD_DATA_READ_DIRECT   {0b00000000};
		static constexpr uint8_t CMD_DATA_READ_COMMAND  {0b00100000};
		static constexpr uint8_t CMD_REG_READ           {0b01000000};
		static constexpr uint8_t CMD_REG_WRITE          {0b01100000};
		static constexpr uint8_t CMD_PULSE_CONVERT      {0b10000000};
		// static constexpr uint8_t CMD_RESERVED           {0b10100000}; 
		static constexpr uint8_t CMD_RESET              {0b11000000};
		//static constexpr uint8_t CMD_DATA_READ_DIRECT   {0b11000000};	// superfluous
		static constexpr uint8_t CMD_MUL_EN             {0b00010000}; // enables multiple register access
		
        /* register addresses */
        static constexpr uint8_t REG_CONFIG0 {0x00};
        static constexpr uint8_t REG_CONFIG1 {0x01};
        static constexpr uint8_t REG_MUXSCH  {0x02}; // Fixed-Channel mode: input channel select
        static constexpr uint8_t REG_MUXDIF  {0x03}; // Auto-Scan mode: differential input select
        static constexpr uint8_t REG_MUXSG0  {0x04}; // Auto-Scan mode: single input select (AIN7-AIN0)
        static constexpr uint8_t REG_MUXSG1  {0x05}; // Auto-Scan mode: single input select (AIN15-AIN8)
        static constexpr uint8_t REG_SYSRED  {0x06}; // System Readings (Vref, Gain, Temp, Vcc Offset)
        static constexpr uint8_t REG_GPIOC   {0x07}; // GPIO: configure as output (0) or input (1)
        static constexpr uint8_t REG_GPIOD   {0x08}; // GPIO: read or write to pins
		static constexpr uint8_t REG_ID      {0x08}; // Factory ID + ADS1258 / ADS1158

        ADS1X58(SPIClass *s, int csPin, int drdyPin, ADC_TYPE type= ADC_TYPE::ADS1258);

		uint8_t readRegister(uint8_t reg);
        void writeRegister(uint8_t reg, uint8_t val;
        void updateRegister(uint8_t reg, uint8_t mask, uint8_t val);
    private:
        SPIClass *_spi;
        SPISettings mySPISettings = SPISettings(4000000, MSBFIRST, SPI_MODE0);
        int csPin;
        int drdyPin;
}
#endif