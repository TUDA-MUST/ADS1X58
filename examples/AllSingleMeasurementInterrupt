/**
 * @file AllSingleMeasurementInterrupt.ino
 * @brief Example demonstrating auto-scan mode with all single-ended channels using DRDY interrupt
 * 
 * This example configures the ADS1X58 in auto-scan mode to measure all 16 single-ended
 * input channels (AIN0-AIN15). The DRDY pin is used with an interrupt to detect when
 * new conversion data is available. The START pin is kept HIGH to continuously trigger
 * conversions.
 * 
 * Hardware Connections:
 * - SPI: MOSI, MISO, SCK connected to Arduino SPI pins
 * - CS: Connected to pin 10 (configurable)
 * - DRDY: Connected to pin 2 (must be interrupt-capable pin)
 * - START: Connected to pin 9 (kept HIGH for continuous conversion)
 * - VREF: External reference voltage (e.g., 2.5V)
 * 
 * @note DRDY pin must be connected to an interrupt-capable pin (2 or 3 on most Arduino boards)
 */

#include <SPI.h>
#include <ADS1X58.h>

// Pin definitions
const int CS_PIN = 10;        // Chip Select pin
const int DRDY_PIN = 2;       // Data Ready pin (interrupt-capable)
const int START_PIN = 9;      // Start conversion pin

// ADC configuration
const float VREF = 2.5;       // Reference voltage in volts

// Create ADS1X58 instance (using ADS1258 as example)
ADS1X58 adc(&SPI, ADC_TYPE::ADS1258, CS_PIN, VREF);
ADS1X58_ChanData chanData; // will hold data read from ADC

// Interrupt flag
volatile bool dataReady = false;

/**
 * @brief Interrupt Service Routine for DRDY pin
 * 
 * Called when DRDY goes LOW, indicating new conversion data is available.
 * Sets a flag to signal the main loop to read the data.
 */
void drdyISR() {
    dataReady = true;
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        ; // Wait for serial port to connect (needed for native USB)
    }
    
    Serial.println("ADS1X58 Auto-Scan All Single-Ended Channels Example");
    Serial.println("====================================================");
    
    // Initialize SPI
    SPI.begin();
    
    // Configure START pin - keep HIGH for continuous conversion -> startPulseConversion() if no start pin available
    pinMode(START_PIN, OUTPUT);
    // Configure DRDY pin as input with pullup
    pinMode(DRDY_PIN, INPUT_PULLUP);

    // Configure ADC
    Serial.println("\nConfiguring ADC...");
    
    // Set auto-scan mode (scans through all enabled channels), should be on by default
    adc.setChannelMode(ADS1X58::MUXMODE_AUTO_SCAN);
    
    // Enable all 16 single-ended input channels (AIN0-AIN15), should be on by default
    adc.enableAllSingleEndedInputs();
    
    // Optional: Configure ADC settings
    adc.setDataRate(ADS1X58::DRATE_11);           // Maximum data rate
    adc.setChopMode(ADS1X58::CHOP_DIS);            // disable chopping
    adc.setStatusByte(ADS1X58::STAT_EN);          // Enable status byte to identify channels
    
    Serial.println("ADC configured successfully!");
    Serial.println("\nStarting continuous measurement...");
    Serial.println("Format: [Channel] Voltage (Status)");
    Serial.println("------------------------------------\n");
    
    delay(100); // Allow ADC to settle

    // Attach interrupt to DRDY pin (triggers on FALLING edge when data is ready)
    attachInterrupt(digitalPinToInterrupt(DRDY_PIN), drdyISR, FALLING);

    digitalWrite(START_PIN, HIGH); // Start continuous conversions
}

void loop() {
    // Check if new data is available
    if (dataReady) {
        dataReady = false; // Clear the flag
        
        // Read the conversion result
        adc.readChannelDataCommand(&chanData);
        
        // Check if read was successful
        if (chanData.success) {
            // Extract channel ID from status byte (lower 5 bits)
            uint8_t chid = chanData.rawStatus & ADS1X58::STATUS_CHID;
            
            // Convert channel ID to human-readable name
            const char* channelName = adc.chidToName(chid);
            
            // Convert raw code to voltage
            float voltage = adc.codeToVoltage(chanData.rawData);
            
            // Check status flags
            bool newData = chanData.rawStatus & ADS1X58::STATUS_NEW;
            bool overvoltage = chanData.rawStatus & ADS1X58::STATUS_OFV;
            bool supplyLow = chanData.rawStatus & ADS1X58::STATUS_SUPPLY;
            
            // Print measurement with channel identification
            Serial.print("[");
            Serial.print(channelName);
            Serial.print("] ");
            Serial.print(voltage, 6);
            Serial.print(" V");
            
            // Print status information
            Serial.print(" (");
            if (newData) Serial.print("NEW ");
            if (overvoltage) Serial.print("OVF ");
            if (supplyLow) Serial.print("SUPPLY_LOW ");
            Serial.print("Raw: 0x");
            Serial.print(chanData.rawData, HEX);
            Serial.println(")");
        } else {
            Serial.println("Error: Failed to read channel data");
        }
    }
    
    // Optional: Add a small delay to prevent serial buffer overflow
    // Remove or adjust if you need maximum speed
    delayMicroseconds(100);
}
