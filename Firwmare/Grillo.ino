/* GRILLO by Alessandro Matera
 *  26/10/2019
 *  
 *  Board: ATtiny84 - Clock 8Mhz Internal
 *  Board library: https://github.com/SpenceKonde/ATTinyCore
 */

#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/power.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

byte piezoPin1 = 3;
byte piezoPin2 = 4;
byte touchPin = 2;
byte Led1Pin = 1;
byte Led2Pin = 0;
boolean ledState = LOW;

void setup() {

  //disabling all unnecessary peripherals to reduce power
  ADCSRA &= ~bit(ADEN); //disable ADC
  power_adc_disable(); // disable ADC converter
  power_usi_disable(); // disable USI

  // enable global interrupts:
  // Watchdog timeout values
  // 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
  // 6=1sec, 7=2sec, 8=4sec, 9=8sec
  watchdogSetup(3); //128msec

  pinMode (piezoPin1, INPUT);
  pinMode (piezoPin2, INPUT);
  pinMode (Led1Pin, INPUT);
  pinMode (Led2Pin, INPUT);
  pinMode (touchPin, INPUT);
}


void loop() {
  if (pinIsTouched(touchPin)) {
    pinMode (piezoPin1, OUTPUT);
    pinMode (piezoPin2, OUTPUT);
    pinMode (Led1Pin, OUTPUT);
    pinMode (Led2Pin, OUTPUT);
    cricri();
    pinMode (piezoPin1, INPUT);
    pinMode (piezoPin2, INPUT);
    pinMode (Led1Pin, INPUT);
    pinMode (Led2Pin, INPUT);
  }
  goSleep();
}

bool pinIsTouched(byte pin) {
  // This function check it a pin is touched enabling the internal pullup resistor of a
  // digital pin when it's set to INPUT.
  // - We put the pin to OUTPUT and LOW
  // - Next we put the pin as INPUT enabling the PULLUP resistor.
  //
  // If we read the pin we expect to read an HIGH value because of the pullup
  // But, if we touch the pin after we set the pullup resistor, the pin will not
  // goes HIGH right after, because the internal resistor has to charge the capacitance
  // on our finger. So, if we measure the voltage, with the digitalRead function,
  // we see that the pin goes LOW for a few microsec (depending on the value of the
  // pullup resistor). This means that we have a touch

  pinMode (pin, OUTPUT); //we put the pin to OUTPUT and LOW to discharge it
  digitalWrite (pin, LOW);
  delay(1); //we waint until it's really discharged
  cli(); //disable interrupt for precaution
  pinMode (pin, INPUT_PULLUP); //put the pin as INPUT and enabling the PULLUP resistor
  bool isTouched = !digitalRead(touchPin); //check the value of the pin. If it's LOW we have a touch, so we invert the result.
  sei(); //re-enabling the interrupt
  return isTouched; //Send the result of our reading. (with a touch we send a TRUE boolen)
}

void cricri() {

  int x, y, z;

  //int b = random(40, 120); //se b = 4 cricri = 1 sec //3,7
  byte b = random(40, 50);
  int e = random(2000, 3000);

  for (int a = 0; a < b; a++) {

    digitalWrite(Led1Pin, ledState);
    digitalWrite(Led2Pin, !ledState);
    ledState = !ledState;

    byte w = random(20 - 22);
    byte z = random(195, 197);
    for (byte b = 5; b; b--) {
      byte x = 1;
      byte y = z + (random(1, 2));
      for (byte a = 0; a < 125; ++a) {
        digitalWrite(piezoPin1, HIGH);
        digitalWrite(piezoPin2, LOW);
        delayMicroseconds(x);
        delayMicroseconds(b);
        digitalWrite(piezoPin1, LOW);
        digitalWrite(piezoPin2, HIGH);
        delayMicroseconds(x);
        delayMicroseconds(b);
        x++;
        y--;
        if (x == 100)
          y -= 50;
      }
      delay(w);
    }
    int f = random(120, 220);
    delay(f);
  }
}



// Watchdog timeout values
// 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
// 6=1sec, 7=2sec, 8=4sec, 9=8sec
void watchdogSetup(byte ii) {
  // The prescale value is held in bits 5,2,1,0
  // This block moves ii itno these bits
  byte bb;
  if (ii > 9 ) ii = 9;
  bb = ii & 7;
  if (ii > 7) bb |= (1 << 5);
  bb |= (1 << WDCE);

  // Reset the watchdog reset flag
  MCUSR &= ~(1 << WDRF);
  // Start timed sequence
  WDTCR |= (1 << WDCE) | (1 << WDE);
  // Set new watchdog timeout value
  WDTCR = bb;
  // Enable interrupts instead of reset
  WDTCR |= _BV(WDIE);
}


void goSleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode();
  watchdogSetup(3); //3 = 128ms
}

ISR(WDT_vect)
{
  //do nothing
}
