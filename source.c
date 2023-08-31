#define ACK 0x06
#define NACK 0x15
#define CSFAIL 0x51

byte requestData[]={0xA5, 0x08, 0x41, 0x00, 0x02, 0x4E, 0x1C, 0x5A};
unsigned char reveivedData[50]; //input array
int i=0;
int state=0;
unsigned char inchar;
unsigned char checksum;


Serial.println(
