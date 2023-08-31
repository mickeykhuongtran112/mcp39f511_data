#include <Arduino.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
SoftwareSerial Serial1(2,3);

#define dubug 0
#define debugState 1
#define debugHEX 0
#define debugDOUBLE 0
#define debugVerbose 0


uint8_t state = 0;
uint8_t receiveCounter = 0;
double tempOutput;
int outputCounter = 0;
uint8_t receivedData[50];
uint8_t inChar;
uint8_t checksum;
int availablebBytes = 0;
double valuetoPush[7];
int response;

uint16_t calculateChecksum(uint32_t dataReceive);

void setup(){
  Serial.begin(9600);
  Serial1.begin(115200);
  Serial.println("\n\rRead data from MCP39F511-E/MQ");
}

void loop(){
  if(Serial1.available() && (state == 0))
  {
    inChar = Serial1.read();
    receivedData[receiveCounter++] = inChar;
    if(receiveCounter == 3)
    {
      if((receivedData[0] != 0xAB)||(receivedData[1] != 0xCD)||(receivedData[2] != 0xEF)) 
      {
        receiveCounter = 0;
        checksum = 0;
      }
    }
    if(receiveCounter < 20)  checksum += inChar;
    if(receiveCounter == 20)  state = 1;
  }
  if(state == 1)
  {
    calculateChecksum(checksum);
    if (checksum = receivedData[19]){
        tempOutput = (receivedData[6] << 24) | (receivedData[5] << 16) | (receivedData[4] << 8) | receivedData[3];
        valuetoPush[1] = tempOutput;
        response = tempOutput / 10000;
        Serial.print("\n\rCurrent RMS = ");
        Serial.println(response);

        tempOutput = 0x0000FFFF & ((receivedData[8] << 8) | receivedData[7]);
        valuetoPush[2] = tempOutput;
        response = tempOutput/10;
        Serial.print("\n\rVoltage RMS = ");
        Serial.println(response);

        tempOutput = (receivedData[12] << 24) | (receivedData[11] << 16) | (receivedData[10] << 8) | receivedData[9];
        valuetoPush[3] = tempOutput;
        response = tempOutput/100;
        Serial.print("\n\rActive Power = ");
        Serial.println(response);

        tempOutput = (receivedData[16] << 24) | (receivedData[15] << 16) | (receivedData[14] << 8) | receivedData[13];
        valuetoPush[4] = tempOutput;
        response = tempOutput/100;
        Serial.print("\n\rReactive Power = ");
        Serial.println(response);

        tempOutput = 0x0000FFFF & ((receivedData[18] << 8) | receivedData[17]);
        valuetoPush[5] = tempOutput;
        response = tempOutput/1000;
        Serial.print("\n\rLine Frequency = ");
        Serial.println(response);
    }
    #if debugHEX   
        Serial.print("\n\rCurent RMS x 1000 = 0x");
        Serial.print(receivedData[6], HEX);
        Serial.print(receivedData[5], HEX);
        Serial.print(receivedData[4], HEX);
        Serial.print(receivedData[3], HEX);

        Serial.print("\n\rVoltage RMS x 10 = 0x");
        Serial.print(receivedData[8], HEX);
        Serial.print(receivedData[7], HEX);
                                
        Serial.print("\n\rActive Power x 100 = 0x");
        Serial.print(receivedData[12], HEX);
        Serial.print(receivedData[11], HEX);
        Serial.print(receivedData[10], HEX);
        Serial.print(receivedData[9], HEX);
                    
        Serial.print("\n\rReactive Power x 100 = 0x");
        Serial.print(receivedData[16], HEX);
        Serial.print(receivedData[15], HEX);
        Serial.print(receivedData[14], HEX);
        Serial.print(receivedData[13], HEX);

        Serial.print("\n\rLine Frequency x 1000 = 0x");
        Serial.print(receivedData[18], HEX);
        Serial.print(receivedData[17], HEX);
                                    
        Serial.print("\n\rChecksum Value Received  =  0x");
        Serial.print(receivedData[30], HEX);
        Serial.print("\n\rChecksum Value Calculated = 0x");
        Serial.println(checksum, HEX);
    #endif

      #if  debugDOUBLE 
        Serial.print("\n\rCurent RMS = ");
        tempOutput = (receivedData[6] << 24) | (receivedData[5] << 16) | (receivedData[4] << 8) | receivedData[3];
        Serial.print((tempOutput / 10000), 4);    
        
        Serial.print("\n\rVoltage RMS = ");
        tempOutput = 0x0000FFFF & ((receivedData[8] << 8) | receivedData[7]);
        Serial.print((tempOutput / 10), 4);                     
              
        Serial.print("\n\rActive Power = ");
        tempOutput = (receivedData[12] << 24) | (receivedData[11] << 16) | (receivedData[10] << 8) | receivedData[9];
        Serial.print((tempOutput / 100), 4);
                    
        Serial.print("\n\rReactive Power = ");
        tempOutput = (receivedData[16] << 24) | (receivedData[15] << 16) | (receivedData[14] << 8) | receivedData[13];
        Serial.print((tempOutput / 100), DEC);
                    
        Serial.print("\n\rLine Frequency = ");
        tempOutput = 0x0000FFFF & ((receivedData[18] << 8) | receivedData[17]);
        Serial.print((tempOutput/ 1000), DEC);
        
        Serial.print("\n\rChecksum Value Reveived = 0x");
        Serial.print(receivedData[19], HEX);
        Serial.print("\n\rChecksum Value Calculated = 0x");
        Serial.println(checksum, HEX);
    #endif
    #if debugVerbose
        for(outputCounter = 0; outputCounter < receiveCounter; outputCounter++)
        {
          Serial.print(receivedData[outputCounter], HEX);
          Serial.print(" - outputCounter ");
          Serial.print(outputCounter, DEC);
          Serial.print(" - receiveCounter ");
          Serial.println(receiveCounter, HEX);
        }
    #endif
  
  receiveCounter = 0;
  state = 0;
  #if debugState
    Serial.println("State 1");
  #endif
  
  }
  
}

uint16_t calculateChecksum(uint32_t dataReceive){
  uint32_t remainder = dataReceive % 256;
  uint8_t byte1 = (remainder >> 8) & 0xFF;
  uint8_t byte2 = remainder & 0xFF;
  return (byte1 << 8) | byte2;
}



