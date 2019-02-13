#include <Arduino.h>
#include <Wire.h>

#include "ADXL355.h"

ADXL355::ADXL355(uint8_t deviceId, bool wireBegin) : _deviceId(deviceId)
{
  if (wireBegin)
    Wire.begin();
};

ADXL355::~ADXL355()
{
  if (isRunning())
    stop();
}

bool ADXL355::isDeviceRecognized()
{
  // Read two registers to make sure it is working
  uint16_t test = read16(0x01);

  // These registers should always have 0x1ded in them
  if (test != 0x1ded)
  {
    Serial.printf("Connected device does not appear to be an ADXL355: %02x\n\n", test);
  }  

  return (test == 0x1ded);
}

int ADXL355::start()
{
  int result = 0;

  if (!isDeviceRecognized())
  {
    result = -1;
  }
  else
  {
    uint8_t power = read8(POWER_CTL);

    if (power & POWER_CTL_VALUES::POWER_CTL_OFF)
  	{
		  power = power & (int)POWER_CTL_VALUES::POWER_CTL_ON;
		  write8(POWER_CTL, power);
    }
	}

  return result;
}

int ADXL355::stop()
{
	int power = read8(POWER_CTL);

	if (!(power & POWER_CTL_VALUES::POWER_CTL_OFF))
	{
		power = power | (int)POWER_CTL_VALUES::POWER_CTL_OFF;
		write8(POWER_CTL, power);
	}
}

uint8_t ADXL355::getAnalogDevicesID()
{
  uint8_t work = read8(DEVID_AD);

  return work;
}

uint8_t ADXL355::getAnalogDevicesMEMSID()
{
  uint8_t work = read8(DEVID_MST);

  return work;
}

uint8_t ADXL355::getDeviceId()
{
  uint8_t work = read8(PARTID);

  return work;
}

uint8_t ADXL355::getRevision()
{
  uint8_t work = read8(REVID);

  return work;
}

bool ADXL355::isRunning()
{
  bool result = false;
	int work = read8(POWER_CTL);

	result = (work & POWER_CTL_VALUES::POWER_CTL_OFF)
		? false
		: true;

    return result;
}

bool ADXL355::isTempSensorOn()
{
  bool result = false;
	
	uint8_t work = read8(POWER_CTL);

	result = ((work & POWER_CTL_VALUES::POWER_CTL_OFF) || (work & POWER_CTL_VALUES::POWER_CTL_TEMP_OFF))
		? false
		: true;

    return result;
}

void ADXL355::startTempSensor()
{
	uint8_t work = read8(POWER_CTL);

	if (work & POWER_CTL_VALUES::POWER_CTL_TEMP_OFF)
	{
		work = work & (int)POWER_CTL_VALUES::POWER_CTL_TEMP_ON;
		write8(POWER_CTL, work);
	}
}

void ADXL355::stopTempSensor()
{
	uint8_t work = read8(POWER_CTL);

	if (!(work & POWER_CTL_VALUES::POWER_CTL_TEMP_OFF))
	{
		work = work | (int)POWER_CTL_VALUES::POWER_CTL_TEMP_OFF;
		write8(POWER_CTL, work);
	}
}

double ADXL355::getTemperatureC()
{
  uint16_t itemp = read16(TEMP2);
  double dtemp = ((double)(1852 - itemp)) / 9.05 + 19.21;

	return dtemp;
}

double ADXL355::getTemperatureF()
{
    double result = getTemperatureC();

    return result * 9 / 5 + 32;
}

ADXL355::HPF_CORNER ADXL355::getHpfCorner()
{
  uint8_t work = read8(FILTER);

  return (HPF_CORNER)((work & HPF_CORNER::HPF_CORNER_MASK) >> 4);
}

int ADXL355::getRawAxes(long *x, long *y, long *z)
{
    uint8_t output[9];
    memset(output, 0, 9);

    int result = readBlock(XDATA3, 9, (uint8_t *)output);

    unsigned long workx;
    unsigned long worky;
    unsigned long workz;

    if (result == 9)
    {
        workx = (output[0] << 12) | (output[1] << 4) | (output[2] >> 4);
        worky = (output[3] << 12) | (output[4] << 4) | (output[5] >> 4);
        workz = (output[6] << 12) | (output[7] << 4) | (output[8] >> 4);
    }

    *x = twosCompliment(workx);
    *y = twosCompliment(worky);
    *z = twosCompliment(workz);

    return result;
}

void ADXL355::setHpfCorner(HPF_CORNER value)
{
  if (errorIfRunning())
    return;

  int work = read8(FILTER);

  work = (work & ~(HPF_CORNER::HPF_CORNER_MASK << 4)) | ((int)value) << 4;
	
  write8(FILTER, work);
}

ADXL355::ODR_LPF ADXL355::getOdrLpf()
{
  uint8_t work = read8(FILTER);

  return (ODR_LPF)(work & ODR_LPF::ODR_LPF_MASK);
}

void ADXL355::setOdrLpf(ODR_LPF value)
{
  if (errorIfRunning())
    return;
  
  uint8_t work = read8(FILTER);

  work = (work & ~(ODR_LPF::ODR_LPF_MASK)) | ((int)value);

  write8(FILTER, work);
}

ADXL355::RANGE_VALUES ADXL355::getRange()
{
  int range = read8(I2CSPEED_INTPOLARITY_RANGE);

  return (RANGE_VALUES)(range & RANGE_VALUES::RANGE_MASK);
}

void ADXL355::setRange(RANGE_VALUES value)
{
  if (errorIfRunning())
    return;
  
  uint8_t range = read8(I2CSPEED_INTPOLARITY_RANGE);

  range &= ~(RANGE_VALUES::RANGE_MASK);
  range |= (int)value;

  write8(I2CSPEED_INTPOLARITY_RANGE, range);
}

int ADXL355::getTrim(int32_t *x, int32_t *y, int32_t *z)
{
    uint8_t output[6];

    memset(output, 0xff, sizeof(output));

    int result = readBlock(OFFSET_X_H, 6, (uint8_t *)output);

    if (result == 0)
    {
        *x = twosCompliment((output[0] << 8 | output[1]) << 4);
        *y = twosCompliment((output[2] << 8 | output[3]) << 4);
        *z = twosCompliment((output[4] << 8 | output[5]) << 4);
    }

    return result;
}

void ADXL355::setTrim(int32_t x, int32_t y, int32_t z)
{
  if (errorIfRunning())
    return;

  int16_t workx = (x >> 4);
  int16_t worky = (y >> 4);
  int16_t workz = (z >> 4);
  uint8_t hix = (workx & 0xff00) >> 8;
  uint8_t lox = workx & 0x00ff;
  uint8_t hiy = (worky & 0xff00) >> 8;
  uint8_t loy = worky & 0x00ff;
  uint8_t hiz = (workz & 0xff00) >> 8;
  uint8_t loz = workz & 0x00ff;

  write8(OFFSET_X_H, hix);
  write8(OFFSET_X_L, lox);
  write8(OFFSET_Y_H, hiy);
  write8(OFFSET_Y_L, loy);
  write8(OFFSET_Z_H, hiz);
  write8(OFFSET_Z_L, loz);
}

int ADXL355::getFifoCount()
{
  uint8_t work = read8(FIFO_ENTRIES);

  return work;
}

ADXL355::STATUS_VALUES ADXL355::getStatus()
{
    uint8_t work = read8(STATUS);

    return (STATUS_VALUES)work;
}

bool ADXL355::isFifoFull()
{
    STATUS_VALUES work = getStatus();

    return (work & STATUS_VALUES::FIFO_FULL)? true : false;
}

bool ADXL355::isFifoOverrun()
{
    STATUS_VALUES work = getStatus();

    return (work & STATUS_VALUES::FIFO_OVERRUN)? true : false;
}

bool ADXL355::isDataReady()
{
    STATUS_VALUES work = getStatus();

    return (work & STATUS_VALUES::DATA_READY)? true : false;
}

int ADXL355::readFifoEntries(long *output)
{
    int fifoCount = getFifoCount();
    uint8_t data[9];
    memset(data, 0, 9);

    unsigned long work[3];

    for (int i = 0; i < fifoCount / 3; i++)
    {
        int result = readBlock(FIFO_DATA, 9, (uint8_t *)data);

        if (result > 0)
        {
            for (int j = 0; j < 9; j+= 3)
            {
                work[j / 3] = (data[0 + j] << 12) | (data[1 + j] << 4) | (data[2 + j] >> 4);
                output[i * 3 + j / 3] = twosCompliment(work[j / 3]); 
            }
        }
        else
        {
            return -1;
        }
    }

    return fifoCount / 3;
}

double ADXL355::valueToGals(long rawValue, int decimals)
{
    double slider = (decimals > 1)? pow(10.0, (double)decimals) : 1.0;

    double result = (double)rawValue / 260000.0 * 980.665;

    result = round(result * slider) / slider;

    return result;
}

int64_t ADXL355::valueToGalsInt(int32_t rawValue, int decimals)
{
  double slider = (decimals > 1)? pow(10.0, (double)decimals) : 1.0;
	double work = valueToGals(rawValue, decimals);
	int64_t asInt = (int64_t)(work * slider);
	
	return asInt;
}

bool ADXL355::errorIfRunning()
{
  bool result = false;

	if (isRunning())
  {
		Serial.println("*** ERROR *** Sensor modification attempted when sensor is running");
    result = true;
  }

  return result;
}

long ADXL355::twosCompliment(unsigned long value)
{
  if (value & (1 << 20 - 1))
    value = value - (1 << 20);

  return value;
}

uint8_t ADXL355::read8(uint8_t reg)
{
  uint8_t data = 0;
  
  Wire.beginTransmission(_deviceId);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.beginTransmission(_deviceId);
  Wire.requestFrom(_deviceId, (uint8_t)1);

  if (Wire.available())
  {
    data = Wire.read();
  }
  else
  {
    Serial.println("No data received");
  }

  Wire.endTransmission();

  return data;
}

uint16_t ADXL355::read16(uint8_t reg)
{
  uint16_t data = 0;
  uint8_t oneByte;
  
  Wire.beginTransmission(_deviceId);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.beginTransmission(_deviceId);
  Wire.requestFrom(_deviceId, (uint8_t)2);

  int i = 2;

  while (i--)
  {
    if (Wire.available())
    {
      oneByte = Wire.read();
      data |= (oneByte << (i * 8));
    }
    else
    {
      Serial.println("No data received");
    }
  }

  Wire.endTransmission();

  return data;
}
  
uint8_t ADXL355::readBlock(uint8_t reg, uint8_t length, uint8_t *output)
{
  Wire.beginTransmission(_deviceId);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.beginTransmission(_deviceId);
  Wire.requestFrom(_deviceId, length);

  int i = length;

  while (i)
  {
    if (Wire.available())
    {
      *output++ = Wire.read();
      i--;
    }
    else
    {
      Serial.printf("Ran out of data reading block - requested=%d read=%d\r\n", length, length - i);
      break;
    }
  }

  return length - i;
}

void ADXL355::write8(uint8_t reg, uint8_t data)
{
  Wire.beginTransmission(_deviceId);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}