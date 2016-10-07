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

#include "VCNL40x0.h"

SYSTEM_THREAD(ENABLED);

VCNL40x0 vcnl;

int interruptPin = D4;
int led = D7;
volatile bool state = LOW;

unsigned int nextTime = 0;

unsigned int i=0;
uint8_t ID=0;
uint8_t Command=0;
uint8_t Current=0;
unsigned int  ProxiValue=0;
unsigned long  SummeProxiValue=0;
unsigned int  AverageProxiValue=0;
unsigned int  AmbiValue=0;
uint8_t InterruptStatus=0;
uint8_t InterruptControl=0;

void setup() {
  Serial.begin(9600);
  Serial.println("VCNL40x0 test");
  
  pinMode(interruptPin, INPUT);
  //attachInterrupt(interruptPin, VCNL40x0Isr, CHANGE);

  if (! vcnl.begin()){
    Serial.println("Sensor not found :(");
    while (1);
  }
  Serial.println("Found VCNL4010");
  
  Serial.println(vcnl.readID());	// Read VCNL40x0 product ID revision register
	
  vcnl.writeCurrent(20);	// Set current to 20mA
  
  Serial.println(vcnl.readCurrent());	// Read back IR LED current
  
  // stop all activities (necessary for changing proximity rate, see datasheet)
  vcnl.setCommandRegister (COMMAND_ALL_DISABLE);
  
  // set proximity rate to 2/s
  vcnl.setProximityRate (PROX_MEASUREMENT_RATE_2);
  
  // enable prox and ambi in selftimed mode
  vcnl.setCommandRegister (COMMAND_PROX_ENABLE |
                           COMMAND_AMBI_ENABLE |
                           COMMAND_SELFTIMED_MODE_ENABLE);
						   
  // set interrupt control for threshold
  vcnl.setInterruptControl (INTERRUPT_THRES_SEL_PROX |
                            INTERRUPT_THRES_ENABLE |
                            INTERRUPT_COUNT_EXCEED_2);
  
  
  // set ambient light measurement parameter
  vcnl.setAmbiConfiguration (AMBI_PARA_AVERAGE_32 |
                        AMBI_PARA_AUTO_OFFSET_ENABLE |
                        AMBI_PARA_MEAS_RATE_2);
						
  // measure average of prox value
  SummeProxiValue = 0;
 
    for (i=0; i<30; i++) {  
		  /*do {
			  vcnl.readCommandRegister(); // read command register
		  } while (!COMMAND_MASK_PROX_DATA_READY);
        */
       SummeProxiValue += vcnl.readProxiValue();           // read prox value and sum of all measured prox values                      
    }
 
    AverageProxiValue = SummeProxiValue / 30;                     // calculate average
 
    vcnl.setHighThreshold (AverageProxiValue + 1000);   // set upper threshold for interrupt
    Serial.print("Upper Threshold Value: ");
	Serial.println(AverageProxiValue + 1000);
	Particle.publish("AverageProxy",String(AverageProxiValue));
}

void loop() {
   Serial.print("Ambient: "); Serial.println(vcnl.readAmbiValue());
   Serial.print("Proximity: "); Serial.println(vcnl.readProxiValue());
   //Serial.print("State: "); 
   //Serial.println(digitalRead(interruptPin));  

   if (nextTime > millis()) {
       return;
   }
   nextTime = millis() + 30000;
  
   System.sleep(interruptPin, FALLING, 500);
}

void VCNL40x0Isr(){
    //state != state;
    Serial.println("test");
}
