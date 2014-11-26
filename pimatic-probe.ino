/*
                 Arduino Nano V3
                     _______
  RX             D0 -|  N  |- 
  TX             D1 -|  A  |-             GND
  RESET             -|  N  |-             RESET
  GND               -|  O  |-   	  5V
  receive        D2 -|     |- A7
  IR-LED         D3 -|  V  |- A6      
  transmit       D4 -|  3  |- A5          SCL
  DS18B20        D5 -|     |- A4          SDA
                 D6 -|     |- A3      
                 D7 -|     |- A2      
  LED            D8 -|     |- A1      
  DHT11          D9 -|     |- A0      			 
  SS            D10 -|     |-             AREF
  MOSI          D11 -|     |-             3.3V			
  MISO          D12 -|_____|- D13         LED/SCK
  
  
v 0.1    Read temperature from DS18B20 and broadcast with KaKu_dim protocol
v 0.2    Changed protocol KaKu_dim to Probe protocol
v 0.3    Added DHT11 humidity transmit
v 0.4    Changed to pimatic_generic protocol
v 0.4.1  Changed timings: https://github.com/pimatic/pimatic/issues/74#issuecomment-60989449 
v 0.4.2  Changed transmit() function with argument number of repeats
v 0.5    Add NewRemoteSwitch (KaKu) libraries for retransmitting
v 0.6    Changed Attiny drawing to Nano and changed pin's
v 0.7    Added KaKu retransmit as proxy
v 0.8    Added IR retransmit as proxy


 * Generic Sender code : Send a value (counter) over RF 433.92 mhz
 * Fréquence : 433.92 mhz
 * Protocole : homepi 
 * Licence : CC -by -sa
 * Auteur : Yves Grange
 * Version : 0.1
 * Lase update : 10/10/2014
 * 
 * Based on: Valentin CARRUESCO aka idleman and Manuel Esteban aka Yaug (http://manuel-esteban.com) work  
 */

// Includes
#include <OneWire.h> // http://www.pjrc.com/teensy/arduino_libraries/OneWire.zip
#include <DallasTemperature.h> // http://download.milesburton.com/Arduino/MaximTemperature/DallasTemperature_LATEST.zip
#include <dht.h> // http://playground.arduino.cc/Main/DHTLib#.UyMXevldWCQ
#include <NewRemoteReceiver.h>
#include <NewRemoteTransmitter.h>
#include <IRremote.h>

// Define vars
#define DHT11_PIN 9
#define senderPin 4
#define ONE_WIRE_BUS 5 // DS18B20 PIN
#define rxPin 2   // RF RECEIVER PIN

long codeKit = 666;  // Your unique ID for your Arduino node
int Bytes[30]; 
int BytesData[30]; 

// Set to 1 for serial output
boolean debug = 1;

/*
Config sensors
0=disable
1=enable / every loop
n=run everyth n loop
*/
int DS18B20 = 1;
int DHT11 = 1;
int kaku_proxy = 1;
int ir_proxy = 1;
int LoopDelay = 10000;
int SensorDelay = 1000;

int DS18B20_i = DS18B20;
int DHT11_i = DHT11;

unsigned int AClivingON[] = {3420, 1592, 500, 1128, 464, 1160, 460, 372, 464, 372, 412, 424, 456, 1160, 460, 376, 460, 376, 500, 1132, 412, 1212, 456, 376, 456, 1160, 464, 372, 460, 376, 460, 1160, 412, 1208, 504, 348, 456, 1160, 460, 1160, 460, 372, 464, 372, 460, 1160, 464, 372, 460, 376, 496, 1136, 460, 372, 464, 372, 460, 376, 460, 376, 412, 424, 408, 424, 456, 376, 500, 348, 412, 424, 412, 424, 456, 376, 460, 372, 464, 372, 412, 424, 460, 376, 500, 344, 460, 376, 412, 1208, 460, 376, 460, 376, 408, 1212, 408, 424, 412, 424, 496, 1136, 460, 1160, 460, 380, 408, 420, 464, 372, 460, 376, 412, 424, 412, 424, 500, 344, 408, 428, 412, 1208, 412, 1208, 412, 424, 460, 376, 408, 424, 412, 424, 496, 348, 460, 1160, 412, 424, 412, 1208, 412, 1212, 408, 1212, 412, 420, 460, 1160, 500, 348, 408, 428, 408, 424, 412, 424, 456, 376, 460, 376, 408, 428, 408, 424, 504, 340, 412, 424, 412, 424, 460, 376, 412, 424, 408, 428, 408, 424, 456, 376, 500, 348, 412, 424, 456, 376, 408, 428, 408, 424, 412, 424, 412, 424, 412, 424, 496, 348, 412, 424, 412, 424, 408, 428, 456, 372, 460, 376, 412, 424, 412, 424, 500, 344, 408, 1212, 412, 424, 408, 428, 408, 424, 412, 424, 412, 424, 408, 424, 492};
int AClivingON_len = 227;
unsigned int AClivingOFF[] = {3424, 1592, 504, 1128, 460, 1160, 460, 380, 452, 380, 456, 376, 460, 1160, 460, 372, 464, 372, 500, 1132, 464, 1160, 460, 372, 460, 1160, 460, 376, 460, 376, 460, 1160, 460, 1160, 500, 348, 460, 1160, 460, 1160, 460, 376, 484, 348, 460, 1160, 460, 376, 460, 376, 500, 1128, 460, 376, 460, 372, 460, 376, 460, 376, 460, 372, 464, 372, 460, 376, 500, 348, 456, 380, 456, 376, 456, 380, 456, 376, 460, 372, 464, 372, 460, 376, 500, 348, 456, 380, 456, 376, 456, 380, 456, 380, 456, 1164, 408, 424, 460, 376, 496, 1136, 456, 1160, 460, 376, 460, 376, 460, 372, 464, 372, 460, 376, 460, 376, 500, 348, 456, 376, 460, 1156, 464, 1160, 460, 376, 456, 380, 456, 380, 456, 376, 496, 348, 460, 1160, 460, 376, 460, 1160, 460, 1160, 460, 1160, 460, 376, 460, 1160, 500, 348, 408, 428, 408, 424, 456, 380, 456, 380, 456, 376, 456, 380, 456, 380, 496, 348, 460, 372, 464, 372, 412, 424, 412, 424, 408, 424, 456, 380, 456, 380, 496, 348, 460, 372, 464, 372, 460, 376, 460, 376, 408, 424, 412, 424, 412, 424, 496, 348, 412, 424, 460, 372, 460, 376, 460, 376, 460, 372, 412, 424, 412, 424, 500, 344, 412, 1208, 460, 1160, 460, 1160, 460, 1160, 460, 1164, 412, 1208, 412, 1208, 492};
int AClivingOFF_len = 227;

unsigned int aan2[] = {7672, 2836, 584, 1360, 588, 1356, 584, 2760, 584, 2752, 584, 2752, 584, 2760, 584, 1360, 584, 2760, 584, 2760, 576, 1364, 584, 1360, 584, 1364, 584, 1360, 584, 1360, 584, 1360, 584, 2764, 584, 2752, 584, 2760, 576, 2760, 584, 2752, 588, 2756, 580, 2756, 584, 2756, 584, 2764, 584, 1360, 580, 2756, 584, 1364, 584, 1360, 580, 1368, 580, 1360, 584, 1364, 584, 1364, 588, 1360, 584, 1360, 584, 1364, 580, 1364, 580, 1364, 584, 1360, 584, 1364, 580, 1372, 584, 1360, 580, 1368, 580, 1360, 584, 1364, 584, 1360, 584, 1364, 580, 1364, 580, 1372, 580, 1364, 580, 1364, 584, 1360, 584, 1364, 580, 1360, 584, 1364, 584, 1360, 584, 1368, 584, 1360, 584, 1364, 584, 1360, 580, 1368, 580, 1364, 580, 1364, 584, 1360, 584, 1368, 584, 1360, 584, 2756, 584, 2756, 584, 2752, 584, 2756, 584, 2756, 584, 1364, 580, 1364, 584};
unsigned int uit2[] = {7672, 2840, 584, 1356, 584, 1364, 584, 2756, 584, 2756, 580, 2760, 584, 2752, 584, 1360, 584, 2764, 584, 2752, 584, 1364, 584, 1360, 584, 1364, 584, 1356, 588, 1356, 584, 1364, 584, 1364, 584, 2760, 584, 2752, 584, 2756, 584, 2756, 584, 2756, 580, 2756, 584, 2760, 576, 2764, 584, 1360, 584, 2760, 580, 1360, 584, 1368, 580, 1360, 584, 1360, 584, 1364, 584, 1364, 584, 1364, 584, 1360, 584, 1364, 580, 1360, 584, 1368, 580, 1360, 584, 1364, 584, 1364, 584, 1364, 584, 1360, 584, 1360, 588, 1360, 580, 1364, 580, 1364, 584, 1360, 584, 1368, 584, 1360, 584, 1364, 584, 1360, 584, 1360, 584, 1364, 584, 1360, 580, 1368, 580, 1372, 576, 1368, 580, 1368, 576, 1364, 584, 1360, 584, 1364, 584, 1360, 584, 1360, 580, 1372, 584, 1364, 580, 2756, 584, 2756, 580, 2756, 588, 2756, 580, 2756, 584, 1364, 580, 2756, 584};

// Start includes
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature  
DeviceAddress insideThermometer;
dht DHT;
IRsend irsend;


void setup()
{
  if (debug) {
    // Show all configured values
    Serial.begin(115200);
    Serial.print("Configured values:\n");
    Serial.print("DS18B20=");
    Serial.print(DS18B20);
    Serial.print("\n");
    Serial.print("DHT11=");
    Serial.print(DHT11);
    Serial.print("\n");    
    Serial.print("kaku_proxy=");
    Serial.print(kaku_proxy);
    Serial.print("\n");        
    Serial.print("ir_proxy=");
    Serial.print(ir_proxy);
    Serial.print("\n");        
    Serial.print("LoopDelay=");
    Serial.print(LoopDelay);
    Serial.print("\n");    
    Serial.print("SensorDelay=");
    Serial.print(SensorDelay);
    Serial.print("\n");        
  }
  
  // Define pins
  pinMode(senderPin, OUTPUT);

  if (kaku_proxy != 0) {
    // Initialize receiver on interrupt 0 (= digital pin 2), calls the callback "retransmit"
    NewRemoteReceiver::init(0, 2, retransmit);
  } 

  if (DS18B20 != 0) {
     //start up temp sensor
    sensors.begin();   
    sensors.getAddress(insideThermometer, 0);
    if (debug) {
      Serial.print("DS18B20 found on address: ");
      printAddress(insideThermometer);
      Serial.print("\n");
    }
    int reso = sensors.getResolution(insideThermometer);
    if (reso != 12) {
      if (debug) {
        Serial.print("Resolution of DS18B20 is not 12 but ");
        Serial.print(reso);
        Serial.print(" changing to 12\n");
      }
      sensors.setResolution(insideThermometer, 12);
      if (debug) {
        Serial.print("Done\n");
      }
    }
  }

  // Convert Probe ID to binair
  buildSignal();
}


void loop()
{
//  Serial.print("send IR\n");
//  irsend.sendRaw(AClivingOFF, AClivingOFF_len, 38);
  
  
    if (debug) {
      Serial.print("Start loop\n");
    }  
  if (DS18B20 == DS18B20_i) {
    if (debug) {
      Serial.print("DS18B20\n");
    }  
    // Read DS18B20 and transmit value as sensor 1
    int BytesType[] = {0,0,0,1};

    sensors.requestTemperatures(); // Get the temperature
    float temperature = sensors.getTempCByIndex(0); // Get temperature in Celcius
    //temperature = temperature - 35.0; 
    //unsigned long SendTemp;
    int SendTemp;
    
    if (temperature >= 0.0) {
      SendTemp = temperature * 100;      
      if (debug) {
        Serial.print("\tTemperature: [Positive] ");
        Serial.println(SendTemp);        
        Serial.print("\tStart transmit\n");
      }  
      transmit(true, SendTemp, BytesType, 6);
      if (debug) {
        Serial.print("\tEnd transmit\n");               
      }
    }

    if (temperature < 0.0) {
      SendTemp = (temperature * 100) * -1;      
      if (debug) {
        Serial.print("\tTemperature: [Negative] ");
        Serial.println(SendTemp);
        Serial.print("\tStart transmit\n");               
      }  
      transmit(false, SendTemp, BytesType, 6);
      if (debug) {
        Serial.print("\tEnd transmit\n");               
      }       
    }

    DS18B20_i = 0;
  }

  if (DHT11 == DHT11_i) {
    if (debug) {
      Serial.print("DHT11\n");
    }  
    // Read DHT11 and transmit value as sensor 2
    int chk = DHT.read11(DHT11_PIN);
    switch (chk)
    {
      case DHTLIB_OK:
      float humfloat = DHT.humidity;
      int CounterValue = humfloat * 10;
      int BytesType[] = {0,0,1,0};
      if (debug) {
        Serial.print("\tHumidity: ");
        Serial.println(CounterValue);
        Serial.print("\tStart transmit\n");
      }
      transmit(true, CounterValue, BytesType, 6);
      if (debug) {
        Serial.print("\tEnd transmit\n");
      }
      break;
    }
    delay(SensorDelay);    
    DHT11_i = 0;    
  }


  DS18B20_i ++; 
  DHT11_i ++;   
  
  if (debug) {
    Serial.print("End of loop, sleep.\n\n");
  }  
  delay(LoopDelay);
}
  


void itob(unsigned long integer, int length)
{  
 for (int i=0; i<length; i++){
   if ((integer / power2(length-1-i))==1){
     integer-=power2(length-1-i);
     Bytes[i]=1;
   }
   else Bytes[i]=0;
 }
}

void itobCounter(unsigned long integer, int length)
{  
 for (int i=0; i<length; i++){
   if ((integer / power2(length-1-i))==1){
     integer-=power2(length-1-i);
     BytesData[i]=1;
   }
   else BytesData[i]=0;
 }
}

unsigned long power2(int power){    //gives 2 to the (power)
 unsigned long integer=1;          
 for (int i=0; i<power; i++){      
   integer*=2;
 }
 return integer;
}

/**
 * Crée notre signal sous forme binaire
**/
void buildSignal()
{
  if (debug) {
    Serial.print("Probe ID: ");  
    Serial.println(codeKit);
  }
  // Converti les codes respectifs pour le signal en binaire
  if (debug) {
    Serial.print("Probe ID binair: ");
  }
  itob(codeKit, 14);
  for(int j=0;j < 14; j++){
    if (debug) {
      Serial.print(Bytes[j]);
    }
  }
  if (debug) {
    Serial.print("\n\n");
  }
}


// Convert 0 in 01 and 1 in 10 (Manchester conversion)
void sendPair(bool b) {
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

//Envois d'une pulsation (passage de l'etat haut a l'etat bas)
//1 = 310µs haut puis 1340µs bas
//0 = 310µs haut puis 310µs bas
void sendBit(bool b) {
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

/** 
 * Transmit data
 * @param boolean  positive : if the value you send is a positive or negative one
 * @param long Counter : the value you want to send
 **/
void transmit(boolean positive, unsigned long Counter, int BytesType[], int repeats)
{
 if (debug) {
   Serial.print("\t");
 }   
 int ii;
 for(ii=0; ii<repeats;ii++)
 {
  if (debug) {
    Serial.print(" . ");
  }    
  int i;
  itobCounter(Counter, 30);

  // Send the unique ID of your Arduino node
  for(i=0; i<14;i++)
 {
  sendPair(Bytes[i]);
 }

  // Send protocol type
 for(int j = 0; j<4; j++)
 {
  sendPair(BytesType[j]);
 }

 // Send the flag to mark the value as positive or negative
 sendPair(positive);

 // Send value (ie your counter)
 for(int j = 0; j<30; j++)
 {
   sendPair(BytesData[j]);
 }

 // Send the flag "End of the transmission"
 digitalWrite(senderPin, HIGH);
 delayMicroseconds(650);     
 digitalWrite(senderPin, LOW);
 delayMicroseconds(8602);
 }
 if (debug) {
   Serial.print("\n");
 }    
}


// Callback function is called only when a valid code is received.
void retransmit(NewRemoteCode receivedCode) {
  
  /*
  receivedCode.address      -> ID
  receivedCode.unit         -> unit
  receivedCode.period       -> µs (microseconds)
  receivedCode.switchType   -> on/off/dimlevel
  receivedCode.groupBit     -> group command
  */
  

        Serial.print("ik ontvang iets! \n");
  // Disable the receiver; otherwise it might pick up the retransmit as well.
  NewRemoteReceiver::disable();

  // Need interrupts for delay()
  interrupts();

  // Wait 1 second before sending.
//  delay(1000);
  
  if (receivedCode.address == 8934706 && receivedCode.unit == 9 && receivedCode.switchType == 0) {  
      Serial.print("receive 8934706-9 OFF \n");
      for (int i = 0; i < 2; i++) {
        irsend.sendRaw(AClivingOFF, AClivingOFF_len, 38);
//        irsend.sendRaw(uit2, 147, 38);     
        delay(100);   
      }
  }


  if (receivedCode.address == 8934706 && receivedCode.unit == 9 && receivedCode.switchType == 1) {  
      Serial.print("receive 8934706-9 ON \n");
      for (int i = 0; i < 2; i++) {
//        irsend.sendRaw(AClivingON, AClivingON_len, 38);               
//        irsend.sendRaw(aan2, 147, 38);     
        delay(100);           
      }
  }


  // Enable the receiver.
  NewRemoteReceiver::enable();
        
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
