#include <DHT.h>
#include <DHT_U.h>
#include <LiquidCrystal.h>
#include <AccelStepper.h>
#include <RTClib.h>



const int rs = 12, en = 13, d4 = 8, d5 = 9, d6 = 10, d7 = 11;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

const int in1 = 22, in2 = 24, in3 = 26, in4 = 28;

#define MotorInterfaceType 8 
AccelStepper stepper = AccelStepper(MotorInterfaceType, in1, in3, in2, in4);
const float SPR = 2048;

#define RDA 0x80
#define TBE 0x20 

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

volatile unsigned char* port_k = (unsigned char*) 0x108; 
volatile unsigned char* ddr_k  = (unsigned char*) 0x107; 
volatile unsigned char* pin_k  = (unsigned char*) 0x106;

volatile unsigned char* port_k1 = (unsigned char*) 0x108; 
volatile unsigned char* ddr_k1  = (unsigned char*) 0x107; 
volatile unsigned char* pin_k1  = (unsigned char*) 0x106;

volatile unsigned char* port_f = (unsigned char*) 0x31; 
volatile unsigned char* ddr_f  = (unsigned char*) 0x30; 
volatile unsigned char* pin_f  = (unsigned char*) 0x2F;


// Define Port E Register Pointers
volatile unsigned char* port_e = (unsigned char*) 0x2E; 
volatile unsigned char* ddr_e  = (unsigned char*) 0x2D; 
volatile unsigned char* pin_e  = (unsigned char*) 0x2C;

volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;


volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
volatile unsigned char *myTIFR1 =  (unsigned char *) 0x36;

unsigned int currentTicks;
unsigned int timer_running;
unsigned int ticks[7]= {440,494,523,587,659,698,784};

int butPress = 0;
// Disabled = 0
// Idle = 1
// Error = 2
// Running = 3

int a1 = 0;
int a2 = 0;
int a3 = 0;
int a4 = 0;

int d1 = 0;

int t1 = 0;
int waterThresh = 0;

float motorControl;

bool onOff = false;
// On = true;
// Off = false;

DHT dht(42, DHT11);

RTC_DS1307 rtc;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//Remove
#define motorPin 48
int speed = 0;

void setup() {

  U0Init(9600);
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(600);
  *ddr_k &= 0xFB;
  *port_k |= 0x04;
  *ddr_k1 &= 0xFD;
  *port_k1 |= 0x02;
  // Set lights as output
  set_PE_as_output(4);
  set_PE_as_output(5);
  set_PE_as_output(3);
  motorControl = adc_read(0);
  lcd.begin(16,2);
  adc_init();
  setup_timer_regs();

  #ifndef ESP8266
    while (!Serial); 
  #endif

  if (! rtc.begin()) {
    uartPrintln("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (! rtc.isrunning()) {
    uartPrintln("RTC is NOT running, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  dht.begin();
}
  

void loop() {
  startStop(onOff);
  checkWaterLevel();
  while (t1 == 0){
    waterThresh = adc_read(1) + 100;
    
    t1 = 1;
  }
  
  if (onOff == true){ 
    if (butPress == 1){
      
      startStop(onOff);
      idleState(onOff, butPress);
      
    }
    else if (butPress == 2){
      
      startStop(onOff);
      errorState(onOff, butPress);
    }
    else {
      
      startStop(onOff);
      runningState(onOff, butPress);
      
    }
  }
  else {
    
    startStop(onOff);
    disabledState(onOff, butPress);
  }
  updateMotorAngle();
}

void startStop(bool state){
  if(!(*pin_k & 0x04))
  {
    if (onOff == false){
      onOff = true;
     
    }
    else {
      onOff = false;
     
    }
  }
  
}

int stopStart(bool state){
  if (state == true){  
    if(!timer_running)
    {
        *myTCCR1B |= 0b00000001;
        timer_running = 1;
    }
  }
  else {
      // set the current ticks to the max value
      currentTicks = 65535;
      // if the timer is running
      if(timer_running)
      {
        // stop the timer
        *myTCCR1B &= 0xF8;
        // set the flag to not running
        timer_running = 0;
        // set PB6 LOW
        *port_k &= 0xFB;
      }
    }
}

void disabledState(bool pos, int state){

  while(a1 == 0){
    uartPrintln("Motor is off");
    uartPrintln("Current state: DISABLED");
    tellTime();
  a1 = 1;
  a2 = 0;
  a3 = 0;
  a4 = 0;
  }
  lcdControl(pos);
  write_pe(4, 0);
  write_pe(5, 1);
  write_pe(3, 1);
  startStop(pos);
  
}

void idleState(bool pos, int state){

  while (a2 == 0){
    uartPrintln("Current state: IDLE");
    uartPrintln("Motor is on");
    tellTime();
    a1 = 0;
    a2 = 1;
    a3 = 0;
    a4 = 0;

    
    
  }
  if (checkWaterLevel() < waterThresh){
    
    butPress = 2;
  }
  else if (dht.readTemperature() >= 22.5) {
    butPress = 3;
  } 
  else {
    lcdControl(pos);
  }
  write_pe(4, 0);
  write_pe(5, 0);
  write_pe(3, 1);
  startStop(pos);
}

void errorState(bool pos, int state){

  while(a3 == 0){
    uartPrintln("Current state: ERROR");
    uartPrintln("Motor is off");
    tellTime();
    a1 = 0;
    a2 = 0;
    a3 = 1;
    a4 = 0;

    
    
  }
  if(!(*pin_k1 & 0x02))
  {
    if (checkWaterLevel() > waterThresh){
      butPress = 1;
    }
  }
  write_pe(4, 0);
  write_pe(5, 1);
  write_pe(3, 0);
  startStop(pos);
}

void runningState(bool pos, int state){

  while (a4 == 0){
    uartPrintln("Current state: RUNNING");
    uartPrintln("Motor is on");
    tellTime();
    a1 = 0;
    a2 = 0;
    a3 = 0;
    a4 = 1;

    
    
  }
 
  if (checkWaterLevel() < waterThresh){
    butPress = 2;
  }
  else if (dht.readTemperature() < 22.5){
    butPress = 1;
  }
  else {
    lcdControl(pos);
    
  }
  write_pe(4, 1);
  write_pe(5, 0);
  write_pe(3, 0);
  startStop(pos);
}

void resetState(int state){
  state = 1;
}

void angleChange(float angle){
  static float lastAngle = 0;
  float trueAngle = angle / 384;
  
    
    float angleDifference = (trueAngle - lastAngle) * SPR;
    if (angleDifference != 0){
    stepper.move(angleDifference);
    stepper.runToPosition();
    

    lastAngle = trueAngle;
    }
}

int checkWaterLevel() {
  int waterLevel = adc_read(1);
  return waterLevel;
}


void checkTemperature() {
  float temperature = dht.readTemperature();
  if (temperature >= 22.5) {
    butPress = 3;
  } else {
    butPress = 2;
  }
}

void lcdControl(bool pos){
  if (pos == true){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.setCursor(10, 0);
    lcd.print(dht.readTemperature());
    lcd.setCursor(0, 1);
    lcd.print("Humidity: ");
    lcd.setCursor(10, 1);
    lcd.print(dht.readHumidity());
    d1 = 0;
  }
  else {
    while (d1 == 0){
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("DISABLED");
      d1 = 1;
    }
  }
}

void tellTime(){
    DateTime now = rtc.now();

    uartPrint("Time: ");
    Serial.print(now.hour(), DEC);
    uartPrint(':');
    Serial.print(now.minute(), DEC);
    uartPrint(':');
    Serial.print(now.second(), DEC);
    uartPrintln(" ");
    
}

void readHumidty(){
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();

  char tempStr[10];
  char humStr[10];

  dtostrf(temp, 4, 1, tempStr);
  dtostrf(hum, 4, 1, humStr);
  uartPrint("Temperature = ");
  uartPrintln(tempStr);
  uartPrint("Humidity = ");
  uartPrintln(humStr);
}

void updateMotorAngle() {
  if (butPress == 1 || butPress == 3){
  motorControl = adc_read(0);
  angleChange(motorControl);
  }
}

int getWaterThreshold() {
  return waterThresh;
}

void set_PF_as_output(unsigned char pin_num)
{
    *ddr_f |= 0x01 << pin_num;
}

void write_pf(unsigned char pin_num, unsigned char state)
{
  if(state == 0)
  {
    *port_f &= ~(0x01 << pin_num);
  }
  else
  {
    *port_f |= 0x01 << pin_num;
  }
}

void set_PE_as_output(unsigned char pin_num)
{
    *ddr_e |= 0x01 << pin_num;
}

void write_pe(unsigned char pin_num, unsigned char state)
{
  if(state == 0)
  {
    *port_e &= ~(0x01 << pin_num);
  }
  else
  {
    *port_e |= 0x01 << pin_num;
  }
}
void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}
unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}
void setup_timer_regs()
{
  // setup the timer control registers
  *myTCCR1A= 0x00;
  *myTCCR1B= 0X00;
  *myTCCR1C= 0x00;
  
  // reset the TOV flag
  *myTIFR1 |= 0x01;
  
  // enable the TOV interrupt
  *myTIMSK1 |= 0x01;
}


// TIMER OVERFLOW ISR
ISR(TIMER1_OVF_vect)
{
  // Stop the Timer
  *myTCCR1B &=0xF8;
  // Load the Count
  *myTCNT1 =  (unsigned int) (65535 -  (unsigned long) (currentTicks));
  // Start the Timer
  *myTCCR1B |=   0x01;
  // if it's not the STOP amount
  if(currentTicks != 65535)
  {
    // XOR to toggle PB6
    *port_k ^= 0x04;
  }
}

void U0Init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}
unsigned char kbhit()
{
  return *myUCSR0A & RDA;
}
unsigned char getChar()
{
  return *myUDR0;
}
void putChar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}
void uartPrintln(const char* str) {
  while (*str) {
    putChar(*str++);
  }
  putChar('\n');
  putChar('\r');
}
void uartPrint(const char* str) {
  while (*str) {
    putChar(*str++);
  }
}
