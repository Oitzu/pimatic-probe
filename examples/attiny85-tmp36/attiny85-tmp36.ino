/*
# Sketch to read a TMP36 and send the results with a 433mhz transmitter to pimatic
# The sketch and circuit are designed to run with a 3V coin cell and therefore save energy.
# To improve range of the 433mhz transmitter try to run this with 3 or 4 AA batterys.
#
# To get the correct accuracy for Vcc you need to compensate for the tolerance of the Attiny85 internal reference voltage. 
# For this you need to replace the constant 1102943L in the function readVcc() for your specific attiny85.
# To calculate your constant use: "(1.1 * Vcc1 / Vcc2) * 1023 * 1000" 
# where: Vcc1 = Voltage per voltmeter and Vcc2 = Voltage per readVcc()
# See "Improving Accuracy" at the source "corrected way to determine Vcc".
#
# Source of energy saving methods: 
# http://newblog.lewisd.com/?p=11
# Source of the corrected way to determine Vcc: 
# http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
#
#
#
#
*/


#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

#include <PimaticProbe.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

long codeKit = 105;
int SenderPin = 4;
int TransmitterSupply = 0;
int TMP36Supply = 1;
// Variables for the Sleep/power down modes:
int watchdog_counter = 0;

PimaticProbe probe = PimaticProbe(SenderPin, codeKit);

void setup()
{
  setup_watchdog();
  watchdog_counter = 113; //Set the watchdog_counter high to directly send data after power-up
}

void loop()
{
  system_sleep();
  if(watchdog_counter >= 112) //one system_sleep is about 8 seconds long. So 112 counts should be near 15min.
  {
    watchdog_counter = 0;  //reset watchdog counter
    sbi(ADCSRA,ADEN);     // switch Analog to Digitalconverter ON
    
    //Activating necessary Pins
    pinMode(SenderPin, OUTPUT);
    pinMode(TransmitterSupply, OUTPUT);
    pinMode(TMP36Supply, OUTPUT);
    
    //Powering up Transmitter and tmp36
    digitalWrite(TransmitterSupply, HIGH);
    digitalWrite(TMP36Supply, HIGH);
    
    delay(500);   //Let system settle

    float Vcc = (float)readVcc(); //read Voltage
	
    int reading = analogRead(3); //read the tmp36
    float voltage = (((float)reading / 1024) * (Vcc/1000)); //convert the reading to voltage with the help of vcc and internal 1.1V reference voltage
    
    float temperature = (voltage - 0.5) * 100; // Get temperature in Celcius
    float SendTemp = temperature * 10;
    probe.transmit(true, SendTemp, 1, 4); //send temperature
	
    delay(500);   //Delay send of voltage
	
    probe.transmit(true, Vcc, 7, 4); //send voltage
    
    //Powering down Transmitter and tmp36
    digitalWrite(TransmitterSupply, LOW);
    digitalWrite(TMP36Supply, LOW);
    
    //Deactivating necessary Pins
    pinMode(SenderPin, INPUT);
    pinMode(TransmitterSupply, INPUT);
    pinMode(TMP36Supply, INPUT);
    
    cbi(ADCSRA,ADEN);   // switch Analog to Digitalconverter OFF
  }
}


void system_sleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  sleep_mode();                        // System actually sleeps here
  sleep_disable();                     // System continues execution here when watchdog timed out 
}


ISR(WDT_vect) {
  watchdog_counter++; //increase watchdog-timer
}

void setup_watchdog() {

  byte bb;
  int ww;
  int ii = 9;
  bb=ii & 7;
  bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;

  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCR = bb;
  WDTCR |= _BV(WDIE);
}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1102943L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}