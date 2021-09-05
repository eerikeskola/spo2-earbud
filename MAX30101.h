/*************************************************** 
 This is a library written for the Maxim MAX30105 Optical Smoke Detector
 It should also work with the MAX30102. However, the MAX30102 does not have a Green LED.

 These sensors use I2C to communicate, as well as a single (optional)
 interrupt line that is not currently supported in this driver.
 
 Written by Peter Jansen and Nathan Seidle (SparkFun)
 BSD license, all text above must be included in any redistribution.
 *****************************************************/

//#pragma once

uint32_t getRed(void); //Returns immediate red value
uint32_t getIR(void); //Returns immediate IR value
uint32_t getGreen(void); //Returns immediate green value
_Bool safeCheck(uint8_t maxTimeToCheck); //Given a max amount of time, check for new data

void wakeUp(void);
void shutDown(void);

void setLEDMode(uint8_t mode);

void setADCRange(uint8_t adcRange);
void setSampleRate(uint8_t sampleRate);
void setPulseWidth(uint8_t pulseWidth);

void setPulseAmplitudeRed(uint8_t value);
void setPulseAmplitudeIR(uint8_t value);

//Multi-led configuration mode (page 22)
void enableSlot(uint8_t slotNumber, uint8_t device); //Given slot number, assign a device to slot

// Data Collection

//FIFO Configuration (page 18)
void setFIFOAverage(uint8_t samples);
void enableFIFORollover();
void disableFIFORollover();
void setFIFOAlmostFull(uint8_t samples);

//FIFO Reading
uint16_t check(void); //Checks for new data and fills FIFO
uint8_t available(void); //Tells caller how many new samples are available (head - tail)
void nextSample(void); //Advances the tail of the sense array
uint32_t getFIFORed(void); //Returns the FIFO sample pointed to by tail
uint32_t getFIFOIR(void); //Returns the FIFO sample pointed to by tail
uint32_t getFIFOGreen(void); //Returns the FIFO sample pointed to by tail

uint8_t getWritePointer(void);
uint8_t getReadPointer(void);
void clearFIFO(void); //Sets the read/write pointers to zero

// Die Temperature
/*float readTemperature();
float readTemperatureF();*/

// Low-level I2C communication
uint8_t readRegister8(uint8_t reg);
void writeRegister8(uint8_t reg, uint8_t value);

void bitMask(uint8_t reg, uint8_t mask, uint8_t thing);

void reboot_update();

void startup_blink();

void MAX_setup();