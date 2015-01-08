#include "PimaticProbe.h"
#include <avr/io.h>
#include <arduino.h>

int SystemcodeBytes[30];
int DataBytes[30];
int TypeBytes[4];
int senderPin;

PimaticProbe::PimaticProbe(int transmitPin, int systemcode)
{
	//get Bytes for Systemcode
	itob(systemcode, 14, SystemcodeBytes);
	senderPin = transmitPin;
	pinMode(senderPin, OUTPUT);
}



void PimaticProbe::itob(unsigned long integer, int length, int *array)
{  
	//int Bytes[30];
	for (int i=0; i<length; i++)
	{
		if ((integer / power2(length-1-i))==1)
		{
			integer-=power2(length-1-i);
			array[i]=1;
		}
		else array[i]=0;
	}
}

//gives 2 to the (power)
unsigned long PimaticProbe::power2(int power)
{ 
	unsigned long integer=1;          
	for (int i=0; i<power; i++){      
		integer*=2;
	}
	return integer;
}

void PimaticProbe::transmit(bool positive, unsigned long Counter, int Type, int repeats)
{
	//get Bytes for Data
	itob(Counter, 30, DataBytes);
	//get Bytes for Type
	itob(Type, 4, TypeBytes);
	int ii;
	for(ii=0; ii<repeats;ii++)
	{
		int i;
		// Send the unique ID of your Arduino node
		for(i=0; i<14;i++)
		{
			sendPair(SystemcodeBytes[i]);
		}

		// Send protocol type
		for(int j = 0; j<4; j++)
		{
			sendPair(TypeBytes[j]);
		}

		// Send the flag to mark the value as positive or negative
		sendPair(positive);

		// Send value (ie your counter)
		for(int j = 0; j<30; j++)
		{
			sendPair(DataBytes[j]);
		}

		// Send the flag "End of the transmission"
		digitalWrite(senderPin, HIGH);
		delayMicroseconds(650);     
		digitalWrite(senderPin, LOW);
		delayMicroseconds(8602);
	}   
}

void PimaticProbe::sendPair(bool b) {
 if(b)
 {
   sendBit(true);
   sendBit(false);
 }
 else
 {
   sendBit(false);
   sendBit(true);
 }
}

void PimaticProbe::sendBit(bool b) {
 if (b) {
   digitalWrite(senderPin, HIGH);
   delayMicroseconds(650);   //506 orinally, but tweaked.
   digitalWrite(senderPin, LOW);
   delayMicroseconds(2024);  //1225 orinally, but tweaked.
 }
 else {
   digitalWrite(senderPin, HIGH); 
   delayMicroseconds(650);   //506 orinally, but tweaked.
   digitalWrite(senderPin, LOW);
   delayMicroseconds(4301);   //305 orinally, but tweaked.
 }
}