/****************************************************************************
This library is based on the Adafruit VCNL4010 library and got enhanced by 
features from the mbed library from Dirck Sowada.
*****************************************************************************/

/****************************************************************************
Copyright [2013] [Dirck Sowada]

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*****************************************************************************/

/****************************************************************************
Software License Agreement (BSD License)

Copyright (c) 2012, Adafruit Industries
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holders nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*****************************************************************************/

#include "application.h"
#include "VCNL40x0.h"


/**************************************************************************/
/*! 
    @brief  Instantiates a new VCNL4010 class
*/
/**************************************************************************/
VCNL40x0::VCNL40x0() {
}


/**************************************************************************/
/*! 
    @brief  Setups the HW
*/
/**************************************************************************/
boolean VCNL40x0::begin(uint8_t addr) {
  _i2caddr = addr;
  Wire.begin();

  uint8_t rev = read8(REGISTER_ID);
  //Serial.println(rev, HEX);
  if ((rev & 0xF0) != 0x20) {
    return false;
  }
  
  writeCurrent(20);
  setFrequency(VCNL40x0_390K625);

  write8(REGISTER_INTERRUPT_CONTROL, 0x08);
  return true;
}
 
/**************************************************************************/
/*! 
    @brief  Get and set the LED current draw
*/
/**************************************************************************/

void VCNL40x0::writeCurrent(uint8_t CurrentValue) {
  if (CurrentValue > 20) CurrentValue = 20;
  write8(REGISTER_PROX_CURRENT, CurrentValue);
}

uint8_t VCNL40x0::readCurrent(void) {
  return read8(REGISTER_PROX_CURRENT);
}

/**************************************************************************/
/*! 
    @brief  read Product ID
*/
/**************************************************************************/

uint8_t VCNL40x0::readID (void) {
   return read8(REGISTER_ID);
}

/**************************************************************************/
/*! 
    @brief  set Command Register
*/
/**************************************************************************/

void VCNL40x0::setCommandRegister (uint8_t Command) {
 	write8(REGISTER_COMMAND, Command);
}

/**************************************************************************/
/*! 
    @brief  read Command Register
*/
/**************************************************************************/

uint8_t VCNL40x0::readCommandRegister (void) {
	return read8(REGISTER_COMMAND);
 }

/**************************************************************************/
/*! 
    @brief  set proximity rate
*/
/**************************************************************************/

void VCNL40x0::setProximityRate(uint8_t ProximityRate) {
	write8(REGISTER_PROX_RATE, ProximityRate);
}

/**************************************************************************/
/*! 
    @brief  set ambient configuration
*/
/**************************************************************************/

void VCNL40x0::setAmbiConfiguration (uint8_t AmbiConfiguration) {
	write8(REGISTER_AMBI_PARAMETER, AmbiConfiguration);
}

/**************************************************************************/
/*! 
    @brief  set interrupt control
*/
/**************************************************************************/

void VCNL40x0::setInterruptControl (uint8_t InterruptControl) {
	write8(REGISTER_INTERRUPT_CONTROL, InterruptControl);
}

/**************************************************************************/
/*! 
    @brief  read interrupt control
*/
/**************************************************************************/

uint8_t VCNL40x0::readInterruptControl (void) {
   return read8(REGISTER_INTERRUPT_CONTROL);
}


/**************************************************************************/
/*! 
    @brief  set interrupt status
*/
/**************************************************************************/

void VCNL40x0::setInterruptStatus (uint8_t InterruptStatus) {
	write8(REGISTER_INTERRUPT_STATUS, InterruptStatus);
}

/**************************************************************************/
/*! 
    @brief  set modulator timing adjustment
*/
/**************************************************************************/

void VCNL40x0::setModulatorTimingAdjustment (uint8_t ModulatorTimingAdjustment) {
	write8(REGISTER_PROX_TIMING, ModulatorTimingAdjustment);
}

/**************************************************************************/
/*! 
    @brief  read interrupt status
*/
/**************************************************************************/

uint8_t VCNL40x0::readInterruptStatus (void) {
   return read8(REGISTER_INTERRUPT_STATUS);
}

/**************************************************************************/
/*! 
    @brief  set low threshold
*/
/**************************************************************************/

void VCNL40x0::setLowThreshold (uint16_t LowThreshold) {

	byte LoByte = (LowThreshold & 0x00ff);
    byte HiByte = (LowThreshold & 0xff00)>>8;
	
	//Serial.print("LoByte: "); Serial.println(LoByte, HEX);
	//Serial.print("HiByte: "); Serial.println(HiByte, HEX);
	
	write8(REGISTER_INTERRUPT_LOW_THRES, HiByte);
	write8(REGISTER_INTERRUPT_LOW_THRES+1, LoByte);
}
/**************************************************************************/
/*! 
    @brief  set high threshold
*/
/**************************************************************************/

void VCNL40x0::setHighThreshold (uint16_t HighThreshold) {

	byte LoByte = (HighThreshold & 0x00ff);
    byte HiByte = (HighThreshold & 0xff00)>>8;
	
	//Serial.print("LoByte: "); Serial.println(LoByte, HEX);
	//Serial.print("HiByte: "); Serial.println(HiByte, HEX);
	
	write8(REGISTER_INTERRUPT_HIGH_THRES, HiByte);
	write8(REGISTER_INTERRUPT_HIGH_THRES+1, LoByte);
}

/**************************************************************************/
/*! 
    @brief  read proxi on demand
*/
/**************************************************************************/

uint16_t VCNL40x0::readProxiOnDemand (void) {
 
    unsigned char Command = 0;
	uint16_t ProxiValue = 0;
 
    // enable prox value on demand
    setCommandRegister(COMMAND_PROX_ENABLE | COMMAND_PROX_ON_DEMAND);
 
    // wait on prox data ready bit
    do {
        Command = readCommandRegister();  // read command register
    } while (!(Command & COMMAND_MASK_PROX_DATA_READY));
		
    ProxiValue = readProxiValue();                            // read prox value
 
    setCommandRegister (COMMAND_ALL_DISABLE);               // stop prox value on demand
    
    return ProxiValue;
}
 
/**************************************************************************/
/*! 
    @brief  read ambi on demand
*/
/**************************************************************************/
 
uint16_t VCNL40x0::readAmbiOnDemand (void) {
 
    unsigned char Command=0;
	uint16_t AmbiValue = 0;
 
    // enable ambi value on demand
    setCommandRegister (COMMAND_PROX_ENABLE | COMMAND_AMBI_ON_DEMAND);
 
    // wait on ambi data ready bit
    do {
        Command = readCommandRegister();                     // read command register
    } while (!(Command & COMMAND_MASK_AMBI_DATA_READY));
 
    AmbiValue = readAmbiValue();                              // read ambi value    
 
    setCommandRegister (COMMAND_ALL_DISABLE);               // stop ambi value on demand    
 
    return AmbiValue;
}

/**************************************************************************/
/*! 
    @brief  Get and set the measurement signal frequency
*/
/**************************************************************************/

void VCNL40x0::setFrequency(vcnl40x0_freq f) {
  uint8_t r =  read8(REGISTER_PROX_TIMING);
  r &= ~(0b00011000);
  r |= f << 3;
  write8(REGISTER_PROX_TIMING, r);
}

/**************************************************************************/
/*! 
    @brief  Get proximity measurement
*/
/**************************************************************************/

uint16_t  VCNL40x0::readProxiValue(void) {
  uint8_t i = read8(REGISTER_INTERRUPT_STATUS);
  i &= ~0x80;
  write8(REGISTER_INTERRUPT_STATUS, i);

  write8(REGISTER_COMMAND, COMMAND_PROX_ON_DEMAND);
  while (1) {
    //Serial.println(read8(REGISTER_INTERRUPT_STATUS), HEX);
    uint8_t result = read8(REGISTER_COMMAND);
    //Serial.print("Ready = 0x"); Serial.println(result, HEX);
    if (result & COMMAND_MASK_PROX_DATA_READY) {
      return read16(REGISTER_PROX_VALUE);
    }
    delay(1);
  }
}

uint16_t  VCNL40x0::readAmbiValue(void) {
  uint8_t i = read8(REGISTER_INTERRUPT_STATUS);
  i &= ~0x40;
  write8(REGISTER_INTERRUPT_STATUS, i);


  write8(REGISTER_COMMAND, COMMAND_AMBI_ON_DEMAND);
  while (1) {
    //Serial.println(read8(REGISTER_INTERRUPT_STATUS), HEX);
    uint8_t result = read8(REGISTER_COMMAND);
    //Serial.print("Ready = 0x"); Serial.println(result, HEX);
    if (result & COMMAND_MASK_AMBI_DATA_READY) {
      return read16(REGISTER_AMBI_VALUE);
    }
    delay(1);
  }
}

/**************************************************************************/
/*! 
    @brief  I2C low level interfacing
*/
/**************************************************************************/


// Read 1 byte from the VCNL4000 at 'address'
uint8_t VCNL40x0::read8(uint8_t address)
{
  uint8_t data;

  Wire.beginTransmission(_i2caddr);
  Wire.write(address);
  Wire.endTransmission();

  delayMicroseconds(170);  // delay required

  Wire.requestFrom(_i2caddr, (uint8_t)1);
  while(!Wire.available());

  return Wire.read();
}


// Read 2 byte from the VCNL4000 at 'address'
uint16_t VCNL40x0::read16(uint8_t address)
{
  uint16_t data;

  Wire.beginTransmission(_i2caddr);

  Wire.write(address);

  Wire.endTransmission();

  Wire.requestFrom(_i2caddr, (uint8_t)2);
  while(!Wire.available());

  data = Wire.read();
  data <<= 8;
  while(!Wire.available());
  data |= Wire.read();
  
  return data;
}

// write 1 byte
void VCNL40x0::write8(uint8_t address, uint8_t data)
{
  Wire.beginTransmission(_i2caddr);

  Wire.write(address);
  Wire.write(data);  

  Wire.endTransmission();
}
