#ifndef ADS1X58_H
#define ADS1X58_H

#include <Arduino.h>
#include <SPI.h>

enum class ADC_TYPE { // set ADC type 
	ADS1158, // 16-bit
	ADS1258 // 24-bit
};

/**
 * @struct ADS1X58_ChanData
 * @brief Container for data read from a single channel of the ADS1X58 ADC
 * 
 * @var ADS1X58_ChanData::success
 *      Indicates whether the channel read operation completed successfully
 * 
 * @var ADS1X58_ChanData::rawStatus
 *      Status byte from the ADS1X58 device (only valid when status byte is enabled).
 *      Set to 0xFF when status byte reading was disabled
 * 
 * @var ADS1X58_ChanData::rawData
 *      Raw ADC conversion result as a signed 32-bit integer in two's complement format
 */
struct ADS1X58_ChanData {
    bool success = false; // whether read was successful
    uint8_t rawStatus = 0xFF; // only set when status byte enabled otherwise 0xFF
    int32_t rawData; // signed integer of channel data (two's complement)
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
 * @param type ADC type specification (ADS1258 or ADS1158), see ADC_TYPE enum
 * @param SPIclock SPI clock frequency in Hz (default=4000000)
 */
{
    public:
        /* CONFIG0 Register Settings */
        enum ADS1X58_SPIRST : uint8_t { // inactivity reset timer
            SPIRST_LONG      = 0b00000000,  // 4096fCLK -> 256us @16MHz (default)
            SPIRST_SHORT     = 0b01000000, // 256fCLK -> 16us @16MHz
        };
        enum ADS1X58_MUXMODE : uint8_t { // Channel mode operation
            MUXMODE_AUTO_SCAN      = 0b00000000,  // auto-scan (default)
            MUXMODE_FIXED_CHANNEL  = 0b00100000,  // fixed-channel
        };
        enum ADS1X58_BYPAS : uint8_t { // internal or external connection from MUX output to ADC input
            BYPAS_INTERNAL  = 0b00010000,  // internal MUX (default)
            BYPAS_EXTERNAL  = 0b00000000,  // external connection
        };
        enum ADS1X58_CLKENB : uint8_t { // enables clock output pin
            CLKENB_EN      = 0b00001000,  // enabled (default)
            CLKENB_DIS     = 0b00000000,  // disabled
        };
        enum ADS1X58_CHOP : uint8_t { // chopping enable
            CHOP_EN      = 0b00000100,  // enabled (default)
            CHOP_DIS     = 0b00000000,  // disabled
        };
        enum ADS1X58_STAT : uint8_t { // status byte enable
            STAT_EN      = 0b00000010,  // enabled (default)
            STAT_DIS     = 0b00000000,  // disabled
        };

        /* CONFIG1 Register Settings */
        enum ADS1X58_IDLMOD : uint8_t { // power consumption vs. wake up time to re-enter run mode
            IDLMOD_SLEEP   = 0b10000000, // sleep mode (default)
            IDLMOD_STANDBY = 0b00000000, // standby mode
        };
        enum ADS1X58_DLY : uint8_t { // time delay in number of 128/f_clk periods -> 8us @16MHz // important to settle external signal conditioning
            DLY_0        = 0b00000000, // off (default)
            DLY_1        = 0b00010000, // 8us
            DLY_2        = 0b00100000, // 16us
            DLY_4        = 0b00110000, // 32us
            DLY_8        = 0b01000000, // 64us
            DLY_16       = 0b01010000, // 128us
            DLY_32       = 0b01100000, // 256us
            DLY_48       = 0b01110000, // 384us
        };
        enum ADS1X58_SBCS : uint8_t { // sensor bias current source
            SBCS_OFF     = 0b00000000, // sensor bias current source off (default)
            SBCS_SMALL   = 0b00000100, // 1.5 uA
            SBCS_LARGE   = 0b00001100, // 24 uA
        };
        enum ADS1X58_DRATE : uint8_t { // total data rate in SPS: Auto-Scan Mode / Fixed-Channel Mode // assuming Chop = 0 and DLY[2:0] = 000
            DRATE_00     = 0b00000000,  // 1831 (Auto-Scan) / 1953 (Fixed-Channel)
            DRATE_01     = 0b00000001,  // 6168 (Auto-Scan) / 7813 (Fixed-Channel)
            DRATE_10     = 0b00000010,  // 15123 (Auto-Scan) / 31250 (Fixed-Channel)
            DRATE_11     = 0b00000011,  // 23739 (Auto-Scan) / 125000 (Fixed-Channel) (default)
        };
        
        /* GPIOC & GPIOD Register Settings */
        enum ADS1X58_GPIO_MODE : uint8_t { // GPIO pin mode
            GPIO_OUTPUT  = 0b0, // configure pin as output (default)
            GPIO_INPUT   = 0b1, // configure pin as input
        };
        enum ADS1X58_GPIO_STATE : uint8_t { // GPIO pin state
            GPIO_LOW  = 0b0, // set pin low
            GPIO_HIGH   = 0b1, // set pin high
        };

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
        static constexpr uint8_t REG_CONFIG0 {0x00}; // 0x0A by default
        static constexpr uint8_t REG_CONFIG1 {0x01}; // 0x83 by default
        static constexpr uint8_t REG_MUXSCH  {0x02}; // Fixed-Channel mode: input channel select // 0x00 by default
        static constexpr uint8_t REG_MUXDIF  {0x03}; // Auto-Scan mode: differential input select // 0x00 by default
        static constexpr uint8_t REG_MUXSG0  {0x04}; // Auto-Scan mode: single input select (AIN7-AIN0) // 0xFF by default
        static constexpr uint8_t REG_MUXSG1  {0x05}; // Auto-Scan mode: single input select (AIN15-AIN8) // 0xFF by default
        static constexpr uint8_t REG_SYSRED  {0x06}; // System Readings (Vref, Gain, Temp, Vcc Offset) // 0x00 by default
        static constexpr uint8_t REG_GPIOC   {0x07}; // GPIO: configure as output (0) or input (1) // 0xFF by default
        static constexpr uint8_t REG_GPIOD   {0x08}; // GPIO: read or write to pins // 0x00 by default
		static constexpr uint8_t REG_ID      {0x09}; // Factory ID + ADS1258 / ADS1158 // Bit 4: 1 for ADS1158, 0 for ADS1258

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
        static constexpr int32_t ADS1258_RANGE  {0x780000}; // 24 bit code for +Vref
        static constexpr int32_t ADS1158_RANGE  {0x7800}; // 16 bit code for +Vref

        /* Other */
        static constexpr uint32_t TIMEOUT_MS = 1000; // timeout for waiting new data in ms
        static constexpr float WRONG_FLOAT = -99999.0f; // default wrong float value if something fails

        static constexpr float ADS1X58_TEMP_COEFF_PCB {563.0f}; // temp coefficient if ADC is on PCB
        static constexpr float ADS1X58_TEMP_COEFF_FREE {394.0f}; // temp coefficient if ADC is in free air


        /////////////// FUNCTIONS ///////////////

        /* Constructors */
        explicit ADS1X58(SPIClass *s, ADC_TYPE type, int csPin, float vref, int SPIclock=4000000);
        explicit ADS1X58(SPIClass *s, ADC_TYPE type, int csPin, int SPIclock=4000000); // automatically measures vref

        /* Basic Register Manipulation */
		uint8_t readRegister(uint8_t regAddr);
        uint8_t* readRegistersMultiple(uint8_t startRegAddr, uint8_t* values, uint8_t count);

        void writeRegister(uint8_t regAddr, uint8_t val);
        void writeRegistersMultiple(uint8_t startRegAddr, const uint8_t* values, uint8_t count);
        uint8_t updateRegister(uint8_t regAddr, uint8_t mask, uint8_t val);

        /* Data Reading and Measuring*/
        void readChannelDataDirect(ADS1X58_ChanData* chanData);
        void readChannelDataCommand(ADS1X58_ChanData* chanData);
        bool waitForReadChannelDataCommand(ADS1X58_ChanData* chanData);
        void startPulseConversion();

        float codeToVoltage(int32_t code);
        float readVoltage();

        float measOffset();
        float measVcc();
        float measGain();
        float measVref();
        float measTempC(float tempCoeff = ADS1X58_TEMP_COEFF_PCB);

        /* Configuration and Control */
        void compensateGain(int newGain); // used for gain error compensation // see measGain()
        void setVref(float newVref);
        void setInactivityResetTimer(ADS1X58_SPIRST setting);
        void setChannelMode(ADS1X58_MUXMODE setting);
        void setBypassMode(ADS1X58_BYPAS setting);
        void setClockOutput(ADS1X58_CLKENB setting);
        void setChopMode(ADS1X58_CHOP setting);
        void setStatusByte(ADS1X58_STAT setting);
        void setIdleMode(ADS1X58_IDLMOD setting);
        void setDelay(ADS1X58_DLY setting);
        void setSensorBiasCurrent(ADS1X58_SBCS setting);
        void setDataRate(ADS1X58_DRATE setting);

        void setSingleEndedChannel(uint8_t channel, bool state);
        void enableAllSingleEndedInputs();
        void setDifferentialChannel(uint8_t diffPair, bool state);
        void setFixedChannel(uint8_t AINP, uint8_t AINN, bool state);

        const char* chidToName(uint8_t chid);

        void GPIOpinMode(uint8_t pin, ADS1X58_GPIO_MODE mode);
        void GPIOdigitalWrite(uint8_t pin, ADS1X58_GPIO_STATE state);
        bool GPIOdigitalRead(uint8_t pin);

    private:
        SPIClass *_spi;
        SPISettings mySPISettings;
        ADC_TYPE adcType; // determines 16-bit or 24-bit operation // see ADC_TYPE enum
        int csPin; // chip select pin number
        float vref; // reference voltage in volts. Is used to convert code to voltage
        float _gain = 1.0f; // enables gain error compensation, see compensateGain() and measGain()
        bool statusByteEnabled = true; // whether status byte is enabled in CONFIG0 register

        enum class INTERNAL_TYPE_CONV { // just used internally to quickly select conversion type
            VCC,
            GAIN,
            VREF,
            TEMP,
            OFFSET
        };

        void _readDataBytesToResult(ADS1X58_ChanData* chanData);
        void _internalMeas(ADS1X58_ChanData* chanData, uint8_t sysredBitMask, uint8_t checkChid);
        bool _isNewData(ADS1X58_ChanData* chanData);
        float _internalCodeConversion(int32_t code, INTERNAL_TYPE_CONV measType);
};
#endif