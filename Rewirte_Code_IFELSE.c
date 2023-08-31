#include <Arduino.h>
//#include <HardwareSerial.h>
#include <SoftwareSerial.h>
SoftwareSerial Serial1(2,3); //RX, TX

#define ACK 0x06
#define NACK 0x15
#define CSFAIL 0x51

#define debug 0
#define debugState 1
#define debugVerbose 0 
#define debugHEX 0
#define debugDOUBLE 0

#define RED_LED 4
#define GREEN_LED 5

byte requestData[] = {0xA5, 0x08, 0x41, 0x00, 0x02, 0x4E, 0x1C, 0x5A};

int time0 = 0;
int time1 = 0;
int state = 0;
int timeSlot = 300;
int receiveCounter = 0;
int bytesToReceive = -1;
unsigned char receivedData[50];
unsigned char inChar;
unsigned char checksum;
double tempOutput;
int deviceReadFrequency = 10000;
double valuetoPush[7];
int availableBytes = 0;
int disconnectCounter = 0;
int outputCounter = 0;
int i=0;
int response;
unsigned int tempUInt_16;
unsigned long tempUInt_32;

void setup(){
	Serial.begin(9600);
	Serial1.begin(115200);
	Serial.println("\n\n\rRead data from MCP39F511");
}

void loop(){
	//send read data command to mcp39f511
	time1 = millis(); //count time from begin
	switch(state)
	{
	case 0:
		Serial.flush(); //day tat ca du lieu can truyen ra ngoai
		if((time1 - time0) >= deviceReadFrequency)
		{
			for (i=0; i < sizeof(requestData); i++){
				Serial1.write(requestData[i]);
				delayMicroseconds(timeSlot);
			}
		time0 = millis();
		#if debugState
		Serial.println("0 send command");
		#endif
		state++;
		}
	break;

	case 1:
		availableBytes = Serial1.available(); //tra ve so byte trong rx buffer
		if(availableBytes > 0){
			inChar = (char)Serial1.read();
				if(inChar == ACK){
					receivedData[receiveCounter] = inChar;
					checksum = inChar;
					receiveCounter++;
					state++;
															#if debug
																Serial.print("1 Receive first byte");
																Serial.print(" - inChar ");
																Serial.print(inChar, HEX);
																Serial.print(" - receiveCounter ");
																Serial.print(receiveCounter, HEX);
																Serial.print(" - state ");
																Serial.print(state, DEC);
																Serial.print(" - availableBytes ");
																Serial.println(availableBytes, DEC);
															#endif
				}
				else if(inChar == NACK){
					//error message
					Serial.println("Frame received with success command not understood");
					state--;
				}
				else if(inChar == CSFAIL){
					//error message
					Serial.println("Frame received with success, however, checksum fail");
					state--;
				}
				else{
					state--;
				}
		}
		else{
			if(disconnectCounter >= 10){
				state = 0;
				disconnectCounter = 0;
			}
		}
		disconnectCounter++;
		#if debugState
			Serial.println("State #1");
		#endif
		break;

	case 2:
		disconnectCounter = 0;
		availableBytes = Serial1.available();
		if(availableBytes > 0){
			inChar = (char)Serial1.read();
			receivedData[receiveCounter] = inChar;
			bytesToReceive = inChar;
			checksum += inChar;
			receiveCounter++;
			state++;
											#if debug
												Serial.print("2 Receive number of bytes to receive");
												Serial.print(" - inChar ");
												Serial.print(inChar, HEX);
												Serial.print(" - receiveCounter ");
												Serial.print(receiveCounter, HEX);
												Serial.print(" - bytesToReceive ");
												Serial.print(bytesToReceive, DEC);
												Serial.print(" - state ");
												Serial.print(state, DEC);
												Serial.print(" - availableBytes ");
												Serial.println(availableBytes, DEC);
											#endif
			}
			#if debugState
				Serial.println("State #2");
			#endif
	break;

	case 3:
		availableBytes = Serial1.available();
        if(availableBytes > 0){
        inChar = (char)Serial1.read();
        receivedData[receiveCounter] = inChar;
        receiveCounter++;
        if(receiveCounter < bytesToReceive){
            checksum += inChar;
        }
        if(receiveCounter >= bytesToReceive){
            state++;
            #if debugState
            Serial.println("State #3 -> #4");
            #endif
        }
											#if debug
												Serial.print("3 Receive remaining bytes");
												Serial.print(" - inChar ");
												Serial.print(inChar, HEX);
												Serial.print(" - receiveCounter ");
												Serial.print(receiveCounter, HEX);
												Serial.print(" - bytesToReceive ");
												Serial.print(bytesToReceive, DEC);
												Serial.print(" - state ");
												Serial.print(state, DEC);
												Serial.print(" - availableBytes ");
												Serial.println(availableBytes, DEC);
											#endif
    	}
		#if debugState
			Serial.println("State #3");
		#endif
	break;

	case 4:
		if(checksum == receivedData[30]){
			digitalWrite(RED_LED, HIGH);
			tempOutput = (receivedData[7] << 8) | receivedData[6];
			valuetoPush[1] = tempOutput;
			response = tempOutput/10;
			Serial.print("\n\rVoltage RMS = ");
			Serial.println(response);
			
			tempOutput = 0x0000FFFF & (receivedData[9] << 8) | receivedData[8];
			valuetoPush[3] = tempOutput;
			response = tempOutput/1000;
			Serial.print("\n\rLine Frequency = ");
			Serial.println(response);

			tempOutput = 0x0000FFFF & ((receivedData[13] << 8 ) | receivedData[12]);
			valuetoPush[7] = tempOutput;
			response = (tempOutput * 3051757813) / 100000000000000;
			Serial.print("\n\rPower Factor = ");
			Serial.println(response);

			tempOutput = (receivedData[17] << 24) | (receivedData[16] << 16) | (receivedData[15] << 8) | receivedData[14];
			valuetoPush[2] = tempOutput;
			response = tempOutput / 10000;
			Serial.print("\n\rCurrent RMS = ");
			Serial.println(response);

			tempOutput = (receivedData[21] << 24) | (receivedData[20] << 16) | (receivedData[19] << 8) | receivedData[18];
			valuetoPush[4] = tempOutput;
			response = tempOutput/100;
			Serial.print("\n\rActive Power = ");
			Serial.println(response);

			tempOutput = (receivedData[25] << 24) | (receivedData[24] << 16) | (receivedData[23] << 8) | receivedData[22];
			valuetoPush[5] = tempOutput;
			response = tempOutput/100;
			Serial.print("\n\rReactive Power = ");
			Serial.println(response);

			tempOutput = (receivedData[29] << 24) | (receivedData[28] << 16) | (receivedData[27] << 8) | receivedData[26];
			valuetoPush[6] = tempOutput;
			response = tempOutput/100;
			Serial.print("\n\rApparent Power = ");
			Serial.println(response);
		}
	
    	#if debugHEX
	      Serial.print("ACK - 0x");
	      Serial.print(receivedData[0], HEX);
	      Serial.print("\n\rNumber of Bytes - 0x");
	      Serial.print(receivedData[1], HEX);
	     
	      Serial.print("\n\rSystem Status - 0x");
	      Serial.print(receivedData[3], HEX);
	      Serial.print(receivedData[2], HEX);
	     
	      Serial.print("\n\rSystem Version - 0x");
	      Serial.print(receivedData[5], HEX);
	      Serial.print(receivedData[4], HEX);
	     
	      Serial.print("\n\rVoltage RMS x 10 = 0x");
	      Serial.print(receivedData[7], HEX);
	      Serial.print(receivedData[6], HEX);
	           
	      Serial.print("\n\rLine Frequency x 1000 = 0x");
	      Serial.print(receivedData[9], HEX);
	      Serial.print(receivedData[8], HEX);
	           
	      Serial.print("\n\rAnalog Input Voltage = 0x");
	      Serial.print(receivedData[11], HEX);
	      Serial.print(receivedData[10], HEX);
	           
	      Serial.print("\n\rPower Factor / 2^(-15) = 0x");
	      Serial.print(receivedData[13], HEX);
	      Serial.print(receivedData[12], HEX);
	     
	      Serial.print("\n\rCurent RMS x 1000 = 0x");
	      Serial.print(receivedData[17], HEX);
	      Serial.print(receivedData[16], HEX);
	      Serial.print(receivedData[15], HEX);
	      Serial.print(receivedData[14], HEX);
	           
	      Serial.print("\n\rActive Power x 100 = 0x");
	      Serial.print(receivedData[21], HEX);
	      Serial.print(receivedData[20], HEX);
	      Serial.print(receivedData[19], HEX);
	      Serial.print(receivedData[18], HEX);
	                 
	      Serial.print("\n\rReactive Power x 100 = 0x");
	      Serial.print(receivedData[25], HEX);
	      Serial.print(receivedData[24], HEX);
	      Serial.print(receivedData[23], HEX);
	      Serial.print(receivedData[22], HEX);
	                 
	      Serial.print("\n\rApparent Power x 100 = 0x");
	      Serial.print(receivedData[29], HEX);
	      Serial.print(receivedData[28], HEX);
	      Serial.print(receivedData[27], HEX);
	      Serial.print(receivedData[26], HEX);
	     
	      Serial.print("\n\rChecksum Value Received  =  0x");
	      Serial.print(receivedData[30], HEX);
	      Serial.print("\n\rChecksum Value Calculated = 0x");
	      Serial.println(checksum, HEX);
    	#endif

		#if  debugDOUBLE     
	      Serial.print("\n\rNumber of Bytes - ");
	      Serial.print(receivedData[1], DEC);
	     
	      Serial.print("\n\rSystem Status - 0x");
	      Serial.print(receivedData[3], HEX);
	      Serial.print(receivedData[2], HEX);
	     
	      Serial.print("\n\rSystem Version - Year-20");
	      Serial.print(((receivedData[5] & 0xF0) >> 4), DEC);
	      Serial.print(" Month-");
	      Serial.print((receivedData[5] & 0x0F), DEC);
	      Serial.print(" Day-");
	      Serial.print(receivedData[4], DEC);
	     
	      Serial.print("\n\rVoltage RMS = ");
	      tempOutput = (receivedData[7] << 8) | receivedData[6];
	      Serial.print((tempOutput / 10), 4);
	           
	      Serial.print("\n\rLine Frequency = ");
	      tempOutput = 0x0000FFFF & (receivedData[9] << 8) | receivedData[8];
	      Serial.print((tempOutput/ 1000), DEC);
	           
	      Serial.print("\n\rAnalog Input Voltage = ");
	      tempUInt_16 = (receivedData[11] << 8) | receivedData[10];
	      Serial.print(tempUInt_16, 4);
	           
	      Serial.print("\n\rPower Factor = ");//TODO: This is a signed number, this needs to betaken into account
	      tempOutput = 0x0000FFFF & ((receivedData[13] << 8) | receivedData[12]);
	      Serial.print((tempOutput * 3051757813) / 100000000000000, 4);
	     
	      Serial.print("\n\rCurent RMS = ");
	      tempOutput = (receivedData[17] << 24) | (receivedData[16] << 16) | (receivedData[15] << 8) | receivedData[14];
	      Serial.print((tempOutput / 10000), 4);
	           
	      Serial.print("\n\rActive Power = ");
	      tempOutput = (receivedData[21] << 24) | (receivedData[20] << 16) | (receivedData[19] << 8) | receivedData[18];
	      Serial.print((tempOutput / 100), 4);
	                 
	      Serial.print("\n\rReactive Power = ");
	      tempOutput = (receivedData[25] << 24) | (receivedData[24] << 16) | (receivedData[23] << 8) | receivedData[22];
	      Serial.print((tempOutput / 100), DEC);
	                 
	      Serial.print("\n\rApparent Power = ");
	      tempOutput = (receivedData[29] << 24) | (receivedData[28] << 16) | (receivedData[27] << 8) | receivedData[26];
	      Serial.print((tempOutput / 100), DEC);
	     
	      Serial.print("\n\rChecksum Value Reveived = 0x");
	      Serial.print(receivedData[30], HEX);
	      Serial.print("\n\rChecksum Value Calculated = 0x");
	      Serial.println(checksum, HEX);
		#endif
	      
		#if debugVerbose
	      for(outputCounter = 0; outputCounter < receiveCounter; outputCounter++){
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
			Serial.println("State #4");
		#endif
	break;
	}
}