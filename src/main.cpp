//PORTB |= ((1 << 0) | (1 << 2)); // setzt Bit 0 und 2 in PORTB auf "1"
//PORTB &= ~((1 << 0) | (1 << 3)); // löscht Bit 0 und 3 in PORTB

#include <Arduino.h>
//#include <avr/sleep.h>

#define ledISR PB0
#define ledPinCIR PB1
#define ledStatus PB3
#define button PB4

int sleepCnt = 0;
int pinChangeCnt = 0;

bool sleepStatus = false;
bool pinChangeStatus = false;

ISR(WDT_vect) { // MUST be present, otherwise ATTiny reboots instead of returning from Sleep!
  
  sleepCnt = sleepCnt + 1;
  sleepStatus = true;
} 

ISR(PCINT0_vect)
{
  pinChangeCnt = pinChangeCnt + 1;
  pinChangeStatus = true;
}

void watchDogSetup(void) {
  // setup of the WDT
  SREG &= ~(1 << 7); //entspricht "cli();" - bit7 in SREG wird auf 0 gesetzt
  MCUSR &= ~(1 << WDRF); // remove reset flag
  WDTCR = (1 << WDCE); // set WDCE, access prescaler
  WDTCR |= ((1 << WDP0) | (0 << WDP1) | (0 << WDP2) | (1 << WDP3)); // set prescaler bits to to 8s
  WDTCR |= (1 << WDIE); // access WDT interrupt
  SREG |= (1 << 7); //entspricht "sei();" - bit7 in SREG wird auf 1 gesetzt
}

void pinChangeIntSetup(void){
  SREG &= ~(1 << 7); //entspricht "cli();" - bit7 in SREG wird auf 0 gesetzt
  GIMSK |= (1 << PCIE) & ~(1 << INT0); //Setze des Pin Change Interrupt Enable Bit - loeschen des INT0 Interrupt enable
  PCMSK |= (1 << PCINT4); //& ~(1 << PCINT5) & ~(1 << PCINT3) & ~(1 << PCINT2) & ~(1 << PCINT1) & ~(1 << PCINT0); //Setzen des Pin Change Enable Mask Bit 4 (PCINT4)  ==> Pin PB4
  SREG |= (1 << 7); //entspricht "sei();" - bit7 in SREG wird auf 1 gesetzt
}

void sendToSleep(void) {
  byte adcsra;
  adcsra = ADCSRA; // save ADC control and status register A
  ADCSRA &= ~(1 << ADEN); // disable ADC
  PRR |= (1 << PRADC); //| (1 << PRTIM0) | (1 << PRTIM1);
  //GIMSK &= ~(1 << PCIE); // Pin Change Interrupt AUS im Sleep
  MCUCR |= (1 << SM1) & ~(1 << SM0); // Sleep-Modus = Power Down
  MCUCR |= (1 << SE); // set sleep enable
  //sleep_cpu(); // sleep
  asm("SLEEP"); // sleep_cpu();
  MCUCR &= ~(1 << SE); // reset sleep enable
  MCUSR &= ~(1 << WDRF); // remove reset flag
  //GIMSK |= (1 << PCIE) & ~(1 << INT0); // Pin Change Interrupt wieder an
  ADCSRA = adcsra; // restore ADC control and status register A
}

void setup() {
  // put your setup code here, to run once:
  pinMode(ledISR, OUTPUT);
  pinMode(ledPinCIR, OUTPUT);
  pinMode(ledStatus, OUTPUT);
  pinMode(button, INPUT_PULLUP);
  
  watchDogSetup();
  pinChangeIntSetup();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (sleepStatus == true) {
    for (int i = 0; i <= sleepCnt; i++) {
      digitalWrite(ledStatus, HIGH);
      delay(80);
      digitalWrite(ledStatus, LOW);
      delay(80);
    }
    sendToSleep();
  }
  if (sleepCnt >= 1) {
    digitalWrite(ledISR,!digitalRead(ledISR));
    sleepCnt = 0;
  }
  else if (sleepCnt > 4) {
    digitalWrite(ledISR, HIGH);
  }
  if (pinChangeStatus == true) {
    pinChangeStatus = false;
    digitalWrite(ledPinCIR,!digitalRead(ledPinCIR));   
  }
  
}