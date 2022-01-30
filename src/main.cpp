//PORTB |= ((1 << 0) | (1 << 2)); // setzt Bit 0 und 2 in PORTB auf "1"
//PORTB &= ~((1 << 0) | (1 << 3)); // löscht Bit 0 und 3 in PORTB

#include <Arduino.h>
//#include <avr/sleep.h>

// Include the libraries we need
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS PB4

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

int tempC = 0;
int tempCOld = 0;

#define NrOfMotorOutputs 4 //Anzahl der AusgÃ¤nge zur Ansteuerung des Motors
#define NrOfPatternsOneTurn 4 //Anzahl der Muster zur Ansteuerung eines Schritts

//define direction
#define DirMotorRight 1 
#define DirMotorLeft 0

byte ActStepNr = 0; //Schritte innerhalb des Step-Patterns

//Schrittmuster definieren
byte StepPattern[NrOfPatternsOneTurn][NrOfMotorOutputs] = 
{
  {1,1,0,0},
  {0,1,1,0},
  {0,0,1,1},
  {1,0,0,1}
};

//define ports for stepper motor
byte MotorOutputPin[4] = {0,1,2,3};

//int StepCount = 0;

int sleepCnt = 0;
int pinChangeCnt = 0;

bool sleepStatus = false;
bool pinChangeStatus = false;
bool checkTempStatus = false;

void doStep(bool MotorDir) {
  if (MotorDir == DirMotorRight) { //Zaehle Positiv - DirMotorRight = 0
    ActStepNr++;
    if (ActStepNr == NrOfPatternsOneTurn){ ActStepNr = 0;}
    for (byte j=0; j < NrOfMotorOutputs; j++)
    {
      digitalWrite(MotorOutputPin[j], StepPattern[ActStepNr][j]);
    }
  }
  else {//means (MotorDir == DirMotorLeft) //Zaehle Negativ
    if (ActStepNr == 0) {ActStepNr = NrOfPatternsOneTurn-1;}
    else {ActStepNr--;}
    for (byte j=0; j < NrOfMotorOutputs; j++)
      {
        digitalWrite(MotorOutputPin[j], StepPattern[ActStepNr][j]);
      } 
  }
}

void stopMotor(void) {
  digitalWrite(MotorOutputPin[0], LOW);
  digitalWrite(MotorOutputPin[1], LOW);
  digitalWrite(MotorOutputPin[2], LOW);
  digitalWrite(MotorOutputPin[3], LOW);
}

ISR(WDT_vect) { // MUST be present, otherwise ATTiny reboots instead of returning from Sleep!
  
  sleepCnt = sleepCnt + 1;
  //sleepStatus = true;
} 

ISR(PCINT0_vect)
{
  //pinChangeCnt = pinChangeCnt + 1;
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

void WDT_off(void) {
  MCUSR &= ~(1 << WDRF); // remove reset flag
  /* Write logical one to WDCE and WDE */
  WDTCR |= (1<<WDCE) | (1<<WDE);
  /* Turn off WDT */
  WDTCR = 0x00;
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
  asm("SLEEP"); // same as sleep_cpu();
  MCUCR &= ~(1 << SE); // reset sleep enable
  MCUSR &= ~(1 << WDRF); // remove reset flag
  //GIMSK |= (1 << PCIE) & ~(1 << INT0); // Pin Change Interrupt wieder an
  ADCSRA = adcsra; // restore ADC control and status register A
}

void setup() {
  // put your setup code here, to run once:
   
  pinMode(PB0, OUTPUT); //Motor Pin 0
  pinMode(PB1, OUTPUT); //Motor Pin 1
  pinMode(PB2, OUTPUT); //Motor Pin 2
  pinMode(PB3, OUTPUT); //Motor Pin 3
  
  // Start up the library
  sensors.begin();

  //WDT_off();
  watchDogSetup();
  //pinChangeIntSetup();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (sleepStatus == true) {
    sendToSleep();
  }
  if (sleepCnt >= 3) {
    sleepCnt = 0;
    for (int i=0; i<1; i++){ 
      doStep(0);
      delay(10);
      stopMotor();
      delay(50);
    }
    for (int i=0; i<1; i++){ 
      doStep(1);
      delay(10);
      stopMotor();
      delay(50);
    }
    for (int i=0; i<1; i++){ 
      doStep(1);
      delay(10);
      stopMotor();
      delay(50);
    }
    for (int i=0; i<1; i++){ 
      doStep(0);
      delay(10);
      stopMotor();
      delay(50);
    }
    checkTempStatus = true;
  }
  else {
  }
  if (pinChangeStatus == true) {
    pinChangeStatus = false;
    //GIMSK &= ~(1 << PCIE);
    //GIFR &= ~((1 << PCIF) | (1 << INTF0));

    //GIMSK |= (1 << PCIE) & ~(1 << INT0); 
  } 
  
  if (checkTempStatus == true) {
    sensors.requestTemperatures(); // Send the command to get temperatures
    tempC = (sensors.getTempCByIndex(0) + 0.5);
    
    if (tempC > tempCOld) {
      for (int i=0; i<(tempC - tempCOld); i++){ 
        doStep(DirMotorRight);
        delay(10);
        stopMotor();
        delay(50);
      }
      tempCOld = tempC; 
      sleepStatus = true;
    }
    else if (tempC < tempCOld) {
      for (int i=0; i<(tempCOld - tempC); i++){ 
        doStep(DirMotorLeft);
        delay(10);
        stopMotor();
        delay(50);
      }
      tempCOld = tempC;
      sleepStatus = true;
    }
    else {
      tempCOld = tempC;
    }
    checkTempStatus = false;
  }
  else {
    sleepStatus = true;
  }
  /*
  delay(1000);
  for (int i=0; i<20; i++){ 
    doStep(0);
    delay(300);
    stopMotor();
  }
  //stopMotor();
  delay(1000);
  for (int i=0; i<20; i++){ 
    doStep(1);
    delay(3);
    stopMotor();
  }
  //stopMotor();
  digitalWrite(PB0, HIGH);
  digitalWrite(PB1, HIGH);
  digitalWrite(PB2, LOW);
  digitalWrite(PB3, LOW);
  delay(30);
  digitalWrite(PB0, LOW);
  digitalWrite(PB1, LOW);
  digitalWrite(PB2, LOW);
  digitalWrite(PB3, LOW);
  delay(500);
  digitalWrite(PB0, LOW);
  digitalWrite(PB1, HIGH);
  digitalWrite(PB2, HIGH);
  digitalWrite(PB3, LOW);
  delay(30);
  digitalWrite(PB0, LOW);
  digitalWrite(PB1, LOW);
  digitalWrite(PB2, LOW);
  digitalWrite(PB3, LOW);
  delay(500);
  digitalWrite(PB0, LOW);
  digitalWrite(PB1, LOW);
  digitalWrite(PB2, HIGH);
  digitalWrite(PB3, HIGH);
  delay(30);
  digitalWrite(PB0, LOW);
  digitalWrite(PB1, LOW);
  digitalWrite(PB2, LOW);
  digitalWrite(PB3, LOW);
  delay(500);
  digitalWrite(PB0, HIGH);
  digitalWrite(PB1, LOW);
  digitalWrite(PB2, LOW);
  digitalWrite(PB3, HIGH);
  delay(30);
  digitalWrite(PB0, LOW);
  digitalWrite(PB1, LOW);
  digitalWrite(PB2, LOW);
  digitalWrite(PB3, LOW);
  delay(500);
  */
  //stopMotor();

}