#ifndef ADS1258_H
#define ADS1258_H

#include <Arduino.h>
#include <SPI.h>

typedef enum ADS1258_DATA_RATE { // total data rate in SPS: Auto-Scan Mode / Fixed-Channel Mode
    ADS1258_DR_00     = 0b00000020,	// 1831  / 1953
	ADS1258_DR_01     = 0b00000020, // 6168  / 7813
	ADS1258_DR_10     = 0b00000020, // 15123 / 31250
	ADS1258_DR_11     = 0b00000020	// 23739 / 125000 (default)
} ads1258DataRate;



class ADS1220_WE
{
    public:

        //ADS1220 SPI commands
        static constexpr uint8_t ADS1220_RESET       {0x06};    
        static constexpr uint8_t ADS1220_START       {0x08};    //Send the START/SYNC command (08h) to start converting in continuous conversion mode
        static constexpr uint8_t ADS1220_PWRDOWN     {0x02};
        static constexpr uint8_t ADS1220_RDATA       {0x10};
        static constexpr uint8_t ADS1220_WREG        {0x40};    // write register
        static constexpr uint8_t ADS1220_RREG        {0x20};    // read register

        /* registers */
        static constexpr uint8_t ADS1258_REG_CONFIG0 {0x00};
        static constexpr uint8_t ADS1258_REG_CONFIG1 {0x01};
        static constexpr uint8_t ADS1258_REG_MUXSCH  {0x02};
        static constexpr uint8_t ADS1258_REG_MUXDIF  {0x03};
        static constexpr uint8_t ADS1258_REG_MUXSG0  {0x04};
        static constexpr uint8_t ADS1258_REG_MUXSG1  {0x05};
        static constexpr uint8_t ADS1258_REG_SYSRED  {0x06};
        static constexpr uint8_t ADS1258_REG_GPIOC   {0x07};
        static constexpr uint8_t ADS1258_REG_GPIOD   {0x08};
		static constexpr uint8_t ADS1258_REG_ID      {0x08};

}
#endif