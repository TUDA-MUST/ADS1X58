#ifndef ADS1X58_H
#define ADS1X58_H

#include <Arduino.h>
#include <SPI.h>

enum class ADC_TYPE { // set ADC type 
	ADS1158, // 16-bit
	ADS1258 // 24-bit
};

struct ADS1X58_ChanData {
    bool success = false;
    uint8_t rawStatus = 0xFF; // should be filled by read function if status byte enabled
    int32_t rawData;
};

/* CONFIG0 */
enum class ADS1X58_SPIRST { // inactivity reset timer
    LONG      = 0b00000000,  // 4096 fclk (default)
    SHORT     = 0b01000000
};
enum class ADS1X58_MUXMODE { // Channel mode operation
    AUTO_SCAN      = 0b00000000,  // auto-scan (default)
    FIXED_CHANNEL  = 0b00100000
};
enum class ADS1X58_BYPAS { // internal or external connection from MUX output to ADC input
    INTERNAL  = 0b00010000,  // internal MUX (default)
    EXTERNAL  = 0b00000000
};
enum class ADS1X58_CLKENB { // enables clock output pin
    EN      = 0b00001000,  // enabled (default)
    DIS     = 0b00000000
};
enum class ADS1X58_CHOP { 
    EN      = 0b00000100,  // enabled (default)
    DIS     = 0b00000000
};
enum class ADS1X58_STAT  { 
    EN      = 0b00000010,  // enabled (default)
    DIS     = 0b00000000
};

/* CONFIG1 */
enum class ADS1X58_IDLMOD { // power consumption vs. wake up time to re-enter run mode
    IDLMOD_SLEEP   = 0b10000000, // sleep mode (default)
    IDLMOD_STANDBY = 0b00000000 // standby mode
};
enum class ADS1X58_DLY { // time delay in 128/f_clk periods // f_clk most likely 16 MHz
    DLY_0        = 0b00000000, // off (default)
    DLY_1        = 0b00010000,
    DLY_2        = 0b00100000,
    DLY_4        = 0b00110000,
    DLY_8        = 0b01000000,
    DLY_16       = 0b01010000,
    DLY_32       = 0b01100000,
    DLY_48       = 0b01110000
};
enum class ADS1X58_SBCS { // sensor bias current source
    OFF     = 0b00000000, // sensor vias current source off (default)
    SMALL   = 0b00010000, // 1.5 microAmps
    LARGE   = 0b00100000  // 24 microAmps
};
enum class ADS1X58_DRATE { // total data rate in SPS: Auto-Scan Mode / Fixed-Channel Mode // assuming Chop = 0 and DLY[2:0] = 000
    DRATE_00     = 0b00000000,	// 1831  / 1953
    DRATE_01     = 0b00000001, // 6168  / 7813
    DRATE_10     = 0b00000010, // 15123 / 31250
    DRATE_11     = 0b00000011	// 23739 / 125000 (default)
};

class ADS1X58
/**
 * @class ADS1X58
 * @brief Driver class for ADS1X58 series analog-to-digital converters (ADS1258/ADS1158)
 * 
 * This class provides an interface to communicate with and control ADS1X58 ADC devices via SPI.
 * It supports configuration of input channels, register access, GPIO operations, and measurement
 * of various system parameters including voltage, temperature, and gain.
 * 
 * @section Commands
 * The driver supports the following SPI command operations:
 * - Direct data reading with command byte
 * - Register read/write operations
 * - Pulse conversions
 * - System reset
 * - Multiple register access mode
 * 
 * @section Registers
 * Configurable registers include:
 * - CONFIG0/CONFIG1: Device configuration and conversion settings
 * - MUXSCH/MUXDIF/MUXSG0/MUXSG1: Input channel multiplexing
 * - SYSRED: System readings (Vref, Gain, Temperature, Vcc)
 * - GPIOC/GPIOD: GPIO configuration and data
 * - ID: Factory identification
 * 
 * @section Channels
 * Supports up to 16 single-ended analog inputs (AIN0-AIN15) and 8 differential pairs.
 * Additional system channels include offset, supply voltage, temperature, gain, and reference.
 * 
 * @section Features
 * - 24-bit (ADS1258) or 16-bit (ADS1158) conversion resolution
 * - Configurable data rate and conversion delay
 * - Status byte indication (new data, overvoltage, supply warnings)
 * - Gain error compensation with external reference
 * - Temperature coefficient compensation for PCB-mounted or free-air operation
 * 
 * @param spi Pointer to SPIClass instance for communication
 * @param csPin Chip Select pin number
 * @param drdyPin Data Ready interrupt pin (optional, default=-1)
 * @param type ADC type specification (ADS1258 or ADS1158), see ADC_TYPE enum
 * @param SPIclock SPI clock frequency in Hz (default=4000000)
 */
{
    public:
		// Command Byte: C2 C1 C0 MUL A3 A2 A1 A0
        /* SPI Commands */
        static constexpr uint8_t CMD_DATA_READ_DIRECT   {0b000 << 5}; // superfluous
		static constexpr uint8_t CMD_DATA_READ_COMMAND  {0b001 << 5};
		static constexpr uint8_t CMD_REG_READ           {0b010 << 5};
		static constexpr uint8_t CMD_REG_WRITE          {0b011 << 5};
		static constexpr uint8_t CMD_PULSE_CONVERT      {0b100 << 5};
		// static constexpr uint8_t CMD_RESERVED           {0b101 << 5}; 
		static constexpr uint8_t CMD_RESET              {0b110 << 5};
		// static constexpr uint8_t CMD_DATA_READ_DIRECT2   {0b000 << 5};	// superfluous
		static constexpr uint8_t CMD_MUL_EN             {1 << 4}; // enables multiple register access
        static constexpr uint8_t CMD_ADDRESS            {0b1111 << 0}; // enables multiple register access

        /* Register addresses */
        static constexpr uint8_t REG_CONFIG0 {0x00};
        static constexpr uint8_t REG_CONFIG1 {0x01};
        static constexpr uint8_t REG_MUXSCH  {0x02}; // Fixed-Channel mode: input channel select
        static constexpr uint8_t REG_MUXDIF  {0x03}; // Auto-Scan mode: differential input select
        static constexpr uint8_t REG_MUXSG0  {0x04}; // Auto-Scan mode: single input select (AIN7-AIN0)
        static constexpr uint8_t REG_MUXSG1  {0x05}; // Auto-Scan mode: single input select (AIN15-AIN8)
        static constexpr uint8_t REG_SYSRED  {0x06}; // System Readings (Vref, Gain, Temp, Vcc Offset)
        static constexpr uint8_t REG_GPIOC   {0x07}; // GPIO: configure as output (0) or input (1)
        static constexpr uint8_t REG_GPIOD   {0x08}; // GPIO: read or write to pins
		static constexpr uint8_t REG_ID      {0x09}; // Factory ID + ADS1258 / ADS1158

        /* Register masks */
        static constexpr uint8_t MASK_CONFIG0_SPIRST   {1 << 6};
        static constexpr uint8_t MASK_CONFIG0_MUXMOD   {1 << 5};
        static constexpr uint8_t MASK_CONFIG0_BYPAS    {1 << 4};
        static constexpr uint8_t MASK_CONFIG0_CLKENB   {1 << 3};
        static constexpr uint8_t MASK_CONFIG0_CHOP     {1 << 2};
        static constexpr uint8_t MASK_CONFIG0_STAT     {1 << 1};

        static constexpr uint8_t MASK_CONFIG1_IDLMOD   {1 << 7};
        static constexpr uint8_t MASK_CONFIG1_DLY      {0b111 << 4};
        static constexpr uint8_t MASK_CONFIG1_SBCS     {0b11 << 2};
        static constexpr uint8_t MASK_CONFIG1_DRATE    {0b11 << 0};

        static constexpr uint8_t MASK_MUXSCH_AINP      {0xF << 4};
        static constexpr uint8_t MASK_MUXSCH_AINN      {0xF};

        static constexpr uint8_t MASK_MUXDIF           {0xFF};
        static constexpr uint8_t MASK_MUXSG0           {0xFF};
        static constexpr uint8_t MASK_MUXSG1           {0xFF};
        
        static constexpr uint8_t MASK_SYSRED_REF       {1 << 5};
        static constexpr uint8_t MASK_SYSRED_GAIN      {1 << 4};
        static constexpr uint8_t MASK_SYSRED_TEMP      {1 << 3};
        static constexpr uint8_t MASK_SYSRED_VCC       {1 << 2};
        static constexpr uint8_t MASK_SYSRED_OFFSET    {1<<0};

        static constexpr uint8_t MASK_GPIOC            {0xFF};
        static constexpr uint8_t MASK_GPIOD            {0xFF};
        static constexpr uint8_t MASK_ID               {0xFF};

        /* STATUS byte of Channel Data */
        static constexpr uint8_t STATUS_NEW            {1 << 7}; // new data available
        static constexpr uint8_t STATUS_OFV            {1 << 6}; // overvoltage
        static constexpr uint8_t STATUS_SUPPLY         {1 << 5}; // AVDD-AVSS is below preset limit
        static constexpr uint8_t STATUS_CHID           {0b11111 << 0}; // AVDD-AVSS is below preset limit

        /* Channel IDs for status byte (CHID) */
        static constexpr uint8_t CHID_DIFF0   {0x00};  // AIN0-AIN1
        static constexpr uint8_t CHID_DIFF1   {0x01};  // AIN2-AIN3
        static constexpr uint8_t CHID_DIFF2   {0x02};  // AIN4-AIN5
        static constexpr uint8_t CHID_DIFF3   {0x03};  // AIN6-AIN7
        static constexpr uint8_t CHID_DIFF4   {0x04};  // AIN8-AIN9
        static constexpr uint8_t CHID_DIFF5   {0x05};  // AIN10-AIN11
        static constexpr uint8_t CHID_DIFF6   {0x06};  // AIN12-AIN13
        static constexpr uint8_t CHID_DIFF7   {0x07};  // AIN14-AIN15
        static constexpr uint8_t CHID_AIN0    {0x08};
        static constexpr uint8_t CHID_AIN1    {0x09};
        static constexpr uint8_t CHID_AIN2    {0x0A};
        static constexpr uint8_t CHID_AIN3    {0x0B};
        static constexpr uint8_t CHID_AIN4    {0x0C};
        static constexpr uint8_t CHID_AIN5    {0x0D};
        static constexpr uint8_t CHID_AIN6    {0x0E};
        static constexpr uint8_t CHID_AIN7    {0x0F};
        static constexpr uint8_t CHID_AIN8    {0x10};
        static constexpr uint8_t CHID_AIN9    {0x11};
        static constexpr uint8_t CHID_AIN10   {0x12};
        static constexpr uint8_t CHID_AIN11   {0x13};
        static constexpr uint8_t CHID_AIN12   {0x14};
        static constexpr uint8_t CHID_AIN13   {0x15};
        static constexpr uint8_t CHID_AIN14   {0x16};
        static constexpr uint8_t CHID_AIN15   {0x17};
        static constexpr uint8_t CHID_OFFSET  {0x18};
        static constexpr uint8_t CHID_VCC     {0x1A};   // weird jump, datasheet might be wrong?
        static constexpr uint8_t CHID_TEMP    {0x1B};
        static constexpr uint8_t CHID_GAIN    {0x1C};
        static constexpr uint8_t CHID_REF     {0x1D};

        /* Output Codes */
        static constexpr float ADS1258_RANGE  {0x780000}; // 24 bit code for +Vref
        static constexpr float ADS1158_RANGE  {0x7800}; // 16 bit code for +Vref
    
        static constexpr float ADS1X58_TEMP_COEFF_PCB {563.0f}; // temp coefficient if ADC is on PCB
        static constexpr float ADS1X58_TEMP_COEFF_FREE {394.0f}; // temp coefficient if ADC is in free air

        

        /* Other */
        static constexpr uint32_t TIMEOUT_MS = 1000;
        static constexpr float WRONG_FLOAT = -99999.0f;

        explicit ADS1X58(SPIClass *s, int csPin, int drdyPin=-1, ADC_TYPE type, SPIclock=4000000);

		uint8_t readRegister(uint8_t reg);
        void writeRegister(uint8_t reg, uint8_t val);
        void updateRegister(uint8_t reg, uint8_t mask, uint8_t val);
        float codeToVoltage(int32_t code);
        float measOffset();
        float measVcc();
        float measGain();
        float measRef();
        float measTemp();
        float _codeToInternalReading(int32_t code, INTERNAL_TYPE_CONV measType);

    private:
        SPIClass *_spi;
        SPISettings mySPISettings;
        int csPin;
        int drdyPin;
        ADC_TYPE adcType;
        float vref;
        float _gain=1.0f; // enables gain error compensation using the external reference
        bool statusByteEnabled; // whether status byte is enabled in CONFIG0 register

        explicit ADS1X58(SPIClass *s, int csPin, int drdyPin=-1, ADC_TYPE type, SPIclock=4000000);
}
#endif