#include <stdint.h>
#include <string.h>
#include "MAX30101.h"

//TWI includes
#include "boards.h"
#include "app_util_platform.h"
#include "nrf_drv_twi.h"

#include "nrf_delay.h"

static const uint8_t MAX_ADDRESS =              0x57;

// MAX30101 defines:
// Status Registers
static const uint8_t MAX30105_INTSTAT1 =	0x00;
static const uint8_t MAX30105_INTSTAT2 =	0x01;
static const uint8_t MAX30105_INTENABLE1 =	0x02;
static const uint8_t MAX30105_INTENABLE2 =	0x03;

// FIFO Registers
static const uint8_t MAX30105_FIFOWRITEPTR = 	0x04;
static const uint8_t MAX30105_FIFOOVERFLOW = 	0x05;
static const uint8_t MAX30105_FIFOREADPTR = 	0x06;
static const uint8_t MAX30105_FIFODATA =	0x07;

// Configuration Registers
static const uint8_t MAX30105_FIFOCONFIG = 	0x08;
static const uint8_t MAX30105_MODECONFIG = 	0x09;
static const uint8_t MAX30105_PARTICLECONFIG = 	0x0A;    // Note, sometimes listed as "SPO2" config in datasheet (pg. 11)
static const uint8_t MAX30105_LED1_PULSEAMP = 	0x0C;
static const uint8_t MAX30105_LED2_PULSEAMP = 	0x0D;
static const uint8_t MAX30105_LED3_PULSEAMP = 	0x0E;
static const uint8_t MAX30105_LED_PROX_AMP = 	0x10;
static const uint8_t MAX30105_MULTILEDCONFIG1 = 0x11;
static const uint8_t MAX30105_MULTILEDCONFIG2 = 0x12;

// Die Temperature Registers
static const uint8_t MAX30105_DIETEMPINT = 	0x1F;
static const uint8_t MAX30105_DIETEMPFRAC = 	0x20;
static const uint8_t MAX30105_DIETEMPCONFIG = 	0x21;

// Proximity Function Registers
static const uint8_t MAX30105_PROXINTTHRESH = 	0x30;

// Part ID Registers
static const uint8_t MAX30105_REVISIONID = 	0xFE;
static const uint8_t MAX30105_PARTID = 		0xFF;    // Should always be 0x15. Identical to MAX30102.

// MAX30105 Commands
// Interrupt configuration (pg 13, 14)
static const uint8_t MAX30105_INT_A_FULL_MASK =		(uint8_t)~0b10000000;
static const uint8_t MAX30105_INT_A_FULL_ENABLE = 	0x80;
static const uint8_t MAX30105_INT_A_FULL_DISABLE = 	0x00;

static const uint8_t MAX30105_INT_DATA_RDY_MASK =       (uint8_t)~0b01000000;
static const uint8_t MAX30105_INT_DATA_RDY_ENABLE =	0x40;
static const uint8_t MAX30105_INT_DATA_RDY_DISABLE =    0x00;

static const uint8_t MAX30105_INT_ALC_OVF_MASK =        (uint8_t) ~0b00100000;
static const uint8_t MAX30105_INT_ALC_OVF_ENABLE = 	0x20;
static const uint8_t MAX30105_INT_ALC_OVF_DISABLE =     0x00;

static const uint8_t MAX30105_INT_PROX_INT_MASK =       (uint8_t)~0b00010000;
static const uint8_t MAX30105_INT_PROX_INT_ENABLE =     0x10;
static const uint8_t MAX30105_INT_PROX_INT_DISABLE =    0x00;

static const uint8_t MAX30105_INT_DIE_TEMP_RDY_MASK =   (uint8_t)~0b00000010;
static const uint8_t MAX30105_INT_DIE_TEMP_RDY_ENABLE = 0x02;
static const uint8_t MAX30105_INT_DIE_TEMP_RDY_DISABLE =0x00;

static const uint8_t MAX30105_SAMPLEAVG_MASK =          (uint8_t)~0b11100000;
static const uint8_t MAX30105_SAMPLEAVG_1 =             0x00;
static const uint8_t MAX30105_SAMPLEAVG_2 =             0x20;
static const uint8_t MAX30105_SAMPLEAVG_4 =             0x40;
static const uint8_t MAX30105_SAMPLEAVG_8 =             0x60;
static const uint8_t MAX30105_SAMPLEAVG_16 =            0x80;
static const uint8_t MAX30105_SAMPLEAVG_32 =            0xA0;

static const uint8_t MAX30105_ROLLOVER_MASK =           0xEF;
static const uint8_t MAX30105_ROLLOVER_ENABLE =         0x10;
static const uint8_t MAX30105_ROLLOVER_DISABLE =        0x00;

static const uint8_t MAX30105_A_FULL_MASK =             0xF0;

// Mode configuration commands (page 19)
static const uint8_t MAX30105_SHUTDOWN_MASK = 	0x7F;
static const uint8_t MAX30105_SHUTDOWN = 	0x80;
static const uint8_t MAX30105_WAKEUP = 		0x00;

static const uint8_t MAX30105_RESET_MASK = 	0xBF;
static const uint8_t MAX30105_RESET = 		0x40;

static const uint8_t MAX30105_MODE_MASK = 	0xF8;
static const uint8_t MAX30105_MODE_REDONLY = 	0x02;
static const uint8_t MAX30105_MODE_REDIRONLY = 	0x03;
static const uint8_t MAX30105_MODE_MULTILED = 	0x07;

// Particle sensing configuration commands (pgs 19-20)
static const uint8_t MAX30105_ADCRANGE_MASK = 	0x9F;
static const uint8_t MAX30105_ADCRANGE_2048 = 	0x00;
static const uint8_t MAX30105_ADCRANGE_4096 = 	0x20;
static const uint8_t MAX30105_ADCRANGE_8192 = 	0x40;
static const uint8_t MAX30105_ADCRANGE_16384 = 	0x60;

static const uint8_t MAX30105_SAMPLERATE_MASK = 0xE3;
static const uint8_t MAX30105_SAMPLERATE_50 = 	0x00;
static const uint8_t MAX30105_SAMPLERATE_100 = 	0x04;
static const uint8_t MAX30105_SAMPLERATE_200 = 	0x08;
static const uint8_t MAX30105_SAMPLERATE_400 = 	0x0C;
static const uint8_t MAX30105_SAMPLERATE_800 = 	0x10;
static const uint8_t MAX30105_SAMPLERATE_1000 = 0x14;
static const uint8_t MAX30105_SAMPLERATE_1600 = 0x18;
static const uint8_t MAX30105_SAMPLERATE_3200 = 0x1C;

static const uint8_t MAX30105_PULSEWIDTH_MASK = 0xFC;
static const uint8_t MAX30105_PULSEWIDTH_69 = 	0x00;
static const uint8_t MAX30105_PULSEWIDTH_118 = 	0x01;
static const uint8_t MAX30105_PULSEWIDTH_215 = 	0x02;
static const uint8_t MAX30105_PULSEWIDTH_411 = 	0x03;

//Multi-LED Mode configuration (pg 22)
static const uint8_t MAX30105_SLOT1_MASK = 	0xF8;
static const uint8_t MAX30105_SLOT2_MASK = 	0x8F;
static const uint8_t MAX30105_SLOT3_MASK = 	0xF8;
static const uint8_t MAX30105_SLOT4_MASK = 	0x8F;

static const uint8_t SLOT_NONE = 		0x00;
static const uint8_t SLOT_RED_LED = 		0x01;
static const uint8_t SLOT_IR_LED = 		0x02;
static const uint8_t SLOT_GREEN_LED = 		0x03;
static const uint8_t SLOT_NONE_PILOT = 		0x04;
static const uint8_t SLOT_RED_PILOT =		0x05;
static const uint8_t SLOT_IR_PILOT = 		0x06;
static const uint8_t SLOT_GREEN_PILOT = 	0x07;

static const uint8_t MAX_30105_EXPECTEDPARTID = 0x15;

uint8_t activeLEDs = 2;

uint8_t readBuffer[32];

#define I2C_BUFFER_LENGTH 32

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

#define STORAGE_SIZE 4 //Each long is 4 bytes so limit this to fit on your micro
typedef struct Record
{
  uint32_t red[STORAGE_SIZE];
  uint32_t IR[STORAGE_SIZE];
  uint32_t green[STORAGE_SIZE];
  uint8_t head;
  uint8_t tail;
} sense_struct; //This is our circular buffer of readings from the sensor

sense_struct sense;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

//
// Low-level I2C Communication
//
uint8_t readRegister8(uint8_t reg) {
  uint8_t ret = 0;
  APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, MAX_ADDRESS, &reg, 1, true));
  APP_ERROR_CHECK(nrf_drv_twi_rx(&m_twi, MAX_ADDRESS, &ret, 1));
  return ret;
}

void writeRegister8(uint8_t reg, uint8_t value) {
  uint8_t combined[2] = {reg, value};
  APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, MAX_ADDRESS, combined, 2, false));
}

//Given a register, read it, mask it, and then set the thing
void bitMask(uint8_t reg, uint8_t mask, uint8_t thing)
{
  // Grab current register context
  uint8_t originalContents = readRegister8(reg);

  // Zero-out the portions of the register we're interested in
  originalContents = originalContents & mask;

  // Change contents
  writeRegister8(reg, originalContents | thing);
}

void wakeUp(void) {
  // Pull IC out of low power mode (datasheet pg. 19)
  bitMask(MAX30105_MODECONFIG, MAX30105_SHUTDOWN_MASK, MAX30105_WAKEUP);
}

void shutDown(void) {
  // Put IC into low power mode (datasheet pg. 19)
  // During shutdown the IC will continue to respond to I2C commands but will
  // not update with or take new readings (such as temperature)
  bitMask(MAX30105_MODECONFIG, MAX30105_SHUTDOWN_MASK, MAX30105_SHUTDOWN);
}

void setLEDMode(uint8_t mode) {
  // Set which LEDs are used for sampling -- Red only, RED+IR only, or custom.
  // See datasheet, page 19
  bitMask(MAX30105_MODECONFIG, MAX30105_MODE_MASK, mode);
}

void setPulseAmplitudeRed(uint8_t amplitude) {
  writeRegister8(MAX30105_LED1_PULSEAMP, amplitude);
}

void setPulseAmplitudeIR(uint8_t amplitude) {
  writeRegister8(MAX30105_LED2_PULSEAMP, amplitude);
}

void setPulseAmplitudeGreen(uint8_t amplitude) {
  writeRegister8(MAX30105_LED3_PULSEAMP, amplitude);
}

void setFIFOAverage(uint8_t numberOfSamples) {
  bitMask(MAX30105_FIFOCONFIG, MAX30105_SAMPLEAVG_MASK, numberOfSamples);
}

void enableSlot(uint8_t slotNumber, uint8_t device) {

  uint8_t originalContents;

  switch (slotNumber) {
    case (1):
      bitMask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT1_MASK, device);
      break;
    case (2):
      bitMask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT2_MASK, device << 4);
      break;
    case (3):
      bitMask(MAX30105_MULTILEDCONFIG2, MAX30105_SLOT3_MASK, device);
      break;
    case (4):
      bitMask(MAX30105_MULTILEDCONFIG2, MAX30105_SLOT4_MASK, device << 4);
      break;
    default:
      //Shouldn't be here!
      break;
  }
}

//Read the FIFO Write Pointer
uint8_t getWritePointer(void) {
  return (readRegister8(MAX30105_FIFOWRITEPTR));
}

//Read the FIFO Read Pointer
uint8_t getReadPointer(void) {
  return (readRegister8(MAX30105_FIFOREADPTR));
}

void setADCRange(uint8_t adcRange) {
  // adcRange: one of MAX30105_ADCRANGE_2048, _4096, _8192, _16384
  bitMask(MAX30105_PARTICLECONFIG, MAX30105_ADCRANGE_MASK, adcRange);
}

void setSampleRate(uint8_t sampleRate) {
  // sampleRate: one of MAX30105_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
  bitMask(MAX30105_PARTICLECONFIG, MAX30105_SAMPLERATE_MASK, sampleRate);
}

void setPulseWidth(uint8_t pulseWidth) {
  // pulseWidth: one of MAX30105_PULSEWIDTH_69, _188, _215, _411
  bitMask(MAX30105_PARTICLECONFIG, MAX30105_PULSEWIDTH_MASK, pulseWidth);
}

void enableFIFORollover(void) {
  bitMask(MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_ENABLE);
}

uint16_t check(void)
{
  //Read register FIDO_DATA in (3-byte * number of active LED) chunks
  //Until FIFO_RD_PTR = FIFO_WR_PTR

  uint8_t readPointer = getReadPointer();
  uint8_t writePointer = getWritePointer();

  int numberOfSamples = 0;

  int count = 0;

  //Do we have new data?
  if (readPointer != writePointer)
  {
    uint8_t fifo = MAX30105_FIFODATA;
    //Calculate the number of readings we need to get from sensor
    numberOfSamples = writePointer - readPointer;
    if (numberOfSamples < 0) numberOfSamples += 32; //Wrap condition

    //We now have the number of readings, now calc bytes to read
    //For this example we are just doing Red and IR (3 bytes each)
    int bytesLeftToRead = numberOfSamples * activeLEDs * 3;

    //We may need to read as many as 288 bytes so we read in blocks no larger than I2C_BUFFER_LENGTH
    //I2C_BUFFER_LENGTH changes based on the platform. 64 bytes for SAMD21, 32 bytes for Uno.
    //Wire.requestFrom() is limited to BUFFER_LENGTH which is 32 on the Uno
    while (bytesLeftToRead > 0)
    {
      int toGet = bytesLeftToRead;
      if (toGet > I2C_BUFFER_LENGTH)
      {
        //If toGet is 32 this is bad because we read 6 bytes (Red+IR * 3 = 6) at a time
        //32 % 6 = 2 left over. We don't want to request 32 bytes, we want to request 30.
        //32 % 9 (Red+IR+GREEN) = 5 left over. We want to request 27.

        toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (activeLEDs * 3)); //Trim toGet to be a multiple of the samples we need to read
      }

      bytesLeftToRead -= toGet;

      //Request toGet number of bytes from sensor
      memset(&readBuffer[0], 0, 32);
      APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, MAX_ADDRESS, &fifo, 1, true));
      APP_ERROR_CHECK(nrf_drv_twi_rx(&m_twi, MAX_ADDRESS, &readBuffer[0], toGet));
      
      while (toGet > 0)
      {
        sense.head++; //Advance the head of the storage struct
        sense.head %= STORAGE_SIZE; //Wrap condition

        uint8_t temp[sizeof(uint32_t)]; //Array of 4 bytes that we will convert into long
        uint32_t tempLong;

        //Burst read three bytes - RED
        temp[3] = 0;
        temp[2] = readBuffer[count];
        count++;
        temp[1] = readBuffer[count];
        count++;
        temp[0] = readBuffer[count];
        count++;

        //Convert array to long
        memcpy(&tempLong, temp, sizeof(tempLong));
		
		tempLong &= 0x3FFFF; //Zero out all but 18 bits

        sense.red[sense.head] = tempLong; //Store this reading into the sense array

        if (activeLEDs > 1)
        {
          //Burst read three more bytes - IR
          temp[3] = 0;
          temp[2] = readBuffer[count];
          count++;
          temp[1] = readBuffer[count];
          count++;
          temp[0] = readBuffer[count];
          count++;

          //Convert array to long
          memcpy(&tempLong, temp, sizeof(tempLong));

		  tempLong &= 0x3FFFF; //Zero out all but 18 bits
          
		  sense.IR[sense.head] = tempLong;
        }


        toGet -= activeLEDs * 3;
      }
      count = 0;

    } //End while (bytesLeftToRead > 0)

  } //End readPtr != writePtr

  return (numberOfSamples); //Let the world know how much new data we found
}

//Check new values
_Bool safeCheck(uint8_t maxTimeToCheck)
{
  check();
  return true;
}

//Tell caller how many samples are available
uint8_t available(void)
{
  int8_t numberOfSamples = sense.head - sense.tail;
  if (numberOfSamples < 0) numberOfSamples += STORAGE_SIZE;

  return (numberOfSamples);
}

//Advance sample by one
void nextSample(void)
{
  if(available()) //Only advance the tail if new data is available
  {
    sense.tail++;
    sense.tail %= STORAGE_SIZE; //Wrap condition
  }
}

//Report the most recent red value
uint32_t getRed(void)
{
  //Check the sensor for new data for 250ms
  if(safeCheck(250))
    return (sense.red[sense.head]);
  else
    return(0); //Sensor failed to find new data
}

//Report the most recent IR value
uint32_t getIR(void)
{
  //Check the sensor for new data for 250ms
  if(safeCheck(250))
    return (sense.IR[sense.head]);
  else
    return(0); //Sensor failed to find new data
}

/**
 * @brief TWI initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = 6,
       .sda                = 9,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

void reboot_update(){
    setLEDMode(MAX30105_MODE_MULTILED);
    enableSlot(3, SLOT_GREEN_LED);
    setPulseAmplitudeRed(0x00);
    setPulseAmplitudeIR(0x00);
    setPulseAmplitudeGreen(0x3C);
}

void startup_blink() {
    nrf_delay_ms(300);
    shutDown();
    nrf_delay_ms(300);
    wakeUp();
    nrf_delay_ms(300);
    shutDown();
}

/**@brief Function for application main entry.
 */
void MAX_setup(void)
{
    twi_init();
    
    setLEDMode(MAX30105_MODE_REDIRONLY);
    setFIFOAverage(MAX30105_SAMPLEAVG_4);
    setADCRange(MAX30105_ADCRANGE_4096);
    setSampleRate(MAX30105_SAMPLERATE_100);
    setPulseWidth(MAX30105_PULSEWIDTH_411);
    enableFIFORollover();
    enableSlot(1, SLOT_RED_LED);
    enableSlot(2, SLOT_IR_LED);
    setPulseAmplitudeRed(0x3C);
    setPulseAmplitudeIR(0x3C);
}