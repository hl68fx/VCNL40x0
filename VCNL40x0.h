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

// the i2c address
#define VCNL40x0_I2CADDR_DEFAULT 0x13

// registers 
#define REGISTER_COMMAND               0x80
#define REGISTER_ID                    0x81
#define REGISTER_PROX_RATE             0x82
#define REGISTER_PROX_CURRENT          0x83
#define REGISTER_AMBI_PARAMETER        0x84
#define REGISTER_AMBI_VALUE            0x85
#define REGISTER_PROX_VALUE            0x87
#define REGISTER_INTERRUPT_CONTROL     0x89
#define REGISTER_INTERRUPT_LOW_THRES   0x8a
#define REGISTER_INTERRUPT_HIGH_THRES  0x8c
#define REGISTER_INTERRUPT_STATUS      0x8e
#define REGISTER_PROX_TIMING           0x8f
#define REGISTER_AMBI_IR_LIGHT_LEVEL   0x90 

// Bits in Command register (0x80)
#define COMMAND_ALL_DISABLE 0x00
#define COMMAND_SELFTIMED_MODE_ENABLE 0x01
#define COMMAND_PROX_ENABLE 0x02
#define COMMAND_AMBI_ENABLE 0x04
#define COMMAND_PROX_ON_DEMAND 0x08
#define COMMAND_AMBI_ON_DEMAND 0x10
#define COMMAND_MASK_PROX_DATA_READY 0x20
#define COMMAND_MASK_AMBI_DATA_READY 0x40
#define COMMAND_MASK_LOCK 0x80

// Bits in Product ID Revision Register (0x81)
#define PRODUCT_MASK_REVISION_ID 0x0f
#define PRODUCT_MASK_PRODUCT_ID 0xf0
// Bits in Prox Measurement Rate register (0x82)
#define PROX_MEASUREMENT_RATE_2 0x00 // DEFAULT
#define PROX_MEASUREMENT_RATE_4 0x01
#define PROX_MEASUREMENT_RATE_8 0x02
#define PROX_MEASUREMENT_RATE_16 0x03
#define PROX_MEASUREMENT_RATE_31 0x04
#define PROX_MEASUREMENT_RATE_62 0x05
#define PROX_MEASUREMENT_RATE_125 0x06
#define PROX_MEASUREMENT_RATE_250 0x07
#define PROX_MASK_MEASUREMENT_RATE 0x07
// Bits in Proximity LED current setting (0x83)
#define PROX_MASK_LED_CURRENT 0x3f // DEFAULT = 2
#define PROX_MASK_FUSE_PROG_ID 0xc0
// Bits in Ambient Light Parameter register (0x84)
#define AMBI_PARA_AVERAGE_1 0x00
#define AMBI_PARA_AVERAGE_2 0x01
#define AMBI_PARA_AVERAGE_4 0x02
#define AMBI_PARA_AVERAGE_8 0x03
#define AMBI_PARA_AVERAGE_16 0x04
#define AMBI_PARA_AVERAGE_32 0x05 // DEFAULT
#define AMBI_PARA_AVERAGE_64 0x06
#define AMBI_PARA_AVERAGE_128 0x07
#define AMBI_MASK_PARA_AVERAGE 0x07
#define AMBI_PARA_AUTO_OFFSET_ENABLE 0x08 // DEFAULT enable
#define AMBI_MASK_PARA_AUTO_OFFSET 0x08
#define AMBI_PARA_MEAS_RATE_1 0x00
#define AMBI_PARA_MEAS_RATE_2 0x10 // DEFAULT
#define AMBI_PARA_MEAS_RATE_3 0x20
#define AMBI_PARA_MEAS_RATE_4 0x30
#define AMBI_PARA_MEAS_RATE_5 0x40
#define AMBI_PARA_MEAS_RATE_6 0x50
#define AMBI_PARA_MEAS_RATE_8 0x60
#define AMBI_PARA_MEAS_RATE_10 0x70
#define AMBI_MASK_PARA_MEAS_RATE 0x70
#define AMBI_PARA_CONT_CONV_ENABLE 0x80
#define AMBI_MASK_PARA_CONT_CONV 0x80 // DEFAULT disable
// Bits in Interrupt Control Register (x89)
#define INTERRUPT_THRES_SEL_PROX 0x00
#define INTERRUPT_THRES_SEL_ALS 0x01
#define INTERRUPT_THRES_ENABLE 0x02
#define INTERRUPT_ALS_READY_ENABLE 0x04
#define INTERRUPT_PROX_READY_ENABLE 0x08
#define INTERRUPT_COUNT_EXCEED_1 0x00 // DEFAULT
#define INTERRUPT_COUNT_EXCEED_2 0x20
#define INTERRUPT_COUNT_EXCEED_4 0x40
#define INTERRUPT_COUNT_EXCEED_8 0x60
#define INTERRUPT_COUNT_EXCEED_16 0x80
#define INTERRUPT_COUNT_EXCEED_32 0xa0
#define INTERRUPT_COUNT_EXCEED_64 0xc0
#define INTERRUPT_COUNT_EXCEED_128 0xe0
#define INTERRUPT_MASK_COUNT_EXCEED 0xe0
// Bits in Interrupt Status Register (x8e)
#define INTERRUPT_STATUS_THRES_HI 0x01
#define INTERRUPT_STATUS_THRES_LO 0x02
#define INTERRUPT_STATUS_ALS_READY 0x04
#define INTERRUPT_STATUS_PROX_READY 0x08
#define INTERRUPT_MASK_STATUS_THRES_HI 0x01
#define INTERRUPT_MASK_THRES_LO 0x02
#define INTERRUPT_MASK_ALS_READY 0x04
#define INTERRUPT_MASK_PROX_READY 0x08




//********************************************************

typedef enum
  {
    VCNL40x0_3M125   = 3,
    VCNL40x0_1M5625  = 2,
    VCNL40x0_781K25  = 1,
    VCNL40x0_390K625 = 0,
  } vcnl40x0_freq;
  
class VCNL40x0 {
 public:
  VCNL40x0();
  boolean begin(uint8_t a = VCNL40x0_I2CADDR_DEFAULT);  

  uint8_t readCurrent(void);
  void writeCurrent(uint8_t CurrentValue);
  
  void setProximityRate(uint8_t ProximityRate);
  void setAmbiConfiguration (uint8_t AmbiConfiguration);
  void setInterruptControl (uint8_t InterruptControl);
  
  uint8_t readInterruptControl (void);
  uint8_t readID (void);
  void setInterruptStatus (uint8_t InterruptStatus);
  void setModulatorTimingAdjustment (uint8_t ModulatorTimingAdjustment);
  uint8_t readInterruptStatus (void);
  void setLowThreshold (uint16_t LowThreshold);
  void setHighThreshold (uint16_t HighThreshold);
  void setCommandRegister (uint8_t Command);
  uint8_t readCommandRegister (void);

  void setFrequency(vcnl40x0_freq f);
  uint16_t readProxiValue(void);
  uint16_t readAmbiValue(void);
  uint16_t readProxiOnDemand(void);
  uint16_t readAmbiOnDemand (void);

 private:

  void write8(uint8_t address, uint8_t data);
  uint16_t read16(uint8_t address);
  uint8_t read8(uint8_t address);

  uint8_t _i2caddr;
};
