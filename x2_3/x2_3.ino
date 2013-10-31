
#include "Wire.h"
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/io.h>
#include <SD.h>
#include <EEPROM.h>
#include <GString.h>

#define DS1307_I2C_ADDRESS 0x68
#define LED_PWR 11

#define LED_MAX 12
#define LED_MIN 6
#define LED_SENSOR 4
#define VCC A3
#define FSR0 5
#define FSR1 4
#define STAT 8
#define R 100000
#define MULTIPLY 5000

#define VCC_MIN 3.1
#define VCC_MIN_WARN 3.15
#define F_SQW 4096
#define F_WINDOW 128


#define T_MSRMNT F_SQW/F_WINDOW
#define T_PRESSURE_CHECK 3
#define MAX_ADDR 0
#define MIN_ADDR 2
#define PERIOD_ADDR 4
#define W_PERIOD_ADDR 5 
#define FILENAMELENGTH 13
Sd2Card card;
SdVolume volume;
SdFile root;
File myFile;
// month+day+hour+minute

char fileName[FILENAMELENGTH];
char PrintBuffer[ 40 ];
GString Printer( PrintBuffer );


volatile boolean measured =false;
volatile boolean resultStored=false;
boolean samplesReady = false;
double tempSum0=0.0;
double tempSum1=0.0;
volatile int actualNumberOfMeasurements0 =0;
volatile int actualNumberOfMeasurements1 =0;
volatile int secondsPassed=0;
volatile int interruptsOccured=0;
volatile int partsOfSecond =0;
volatile float maxMeasurement=0;
volatile float minMeasurement=0;
volatile int windowPeriod=5;

byte second, minute, hour, day, month, year;  
volatile int period=15;

volatile boolean needToMeasureVoltage=false;
volatile boolean needToCheckPressure= false;

byte checkingSeconds=0;
boolean firstSensor=true;

// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)
{
  return ( (val/10*16) + (val%10) );
}

// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return ( (val/16*10) + (val%16) );
}


void setDateDs1307()          // 0-99
{
   Wire.beginTransmission(DS1307_I2C_ADDRESS);
   Wire.write(0);
   Wire.write(decToBcd(second));    // 0 to bit 7 starts the clock
   Wire.write(decToBcd(minute));
   Wire.write(decToBcd(hour));      // If you want 12 hour am/pm you need to set
                                   // bit 6 (also need to change readDateDs1307)
   Wire.write(decToBcd(0x01)); // day of week is iggnored;
   Wire.write(decToBcd(day));
   Wire.write(decToBcd(month));
   Wire.write(decToBcd(year));
   Wire.endTransmission();
   
}

void setRTCSQW(){
  Wire.beginTransmission(DS1307_I2C_ADDRESS);
    Wire.write(0x0E);
    switch(F_SQW){
      case 1: Wire.write(0b00100000); break; 
      case 4096: Wire.write(0b00101000); break;
      case 8192: Wire.write(0b00110000); break;
      case 32768: Wire.write(0b00111000); break;
    }
    
    Wire.endTransmission();
}

void parseDateTimeValue(byte n){
  if (n<10)Printer+="0";
  Printer+=n;
}


void getTimeStamp(boolean forFileName){
  getDateDs1307();
  if (forFileName){
     parseDateTimeValue(month);
     parseDateTimeValue(day);
     parseDateTimeValue(hour);
     parseDateTimeValue(minute); 
  }
  else{
      parseDateTimeValue(hour);
      Printer+=":";
      parseDateTimeValue(minute);
      Printer+=":";
      parseDateTimeValue(second);
      Printer+=" ";
      parseDateTimeValue(day);
      Printer+="/";
      parseDateTimeValue(month);
      Printer+="/";
      parseDateTimeValue(year);
  }
 
}

void convertFloatAndStore(byte addr){
  int i=atof(Printer)*1000;

  EEPROM.write(addr,(byte) i);

  EEPROM.write(addr+1,(byte) (i>>8));

}

float convertStoredToFloat(byte addr){
  int temp;
  float res;
  temp=((EEPROM.read(addr+1))<<8);
  temp|= EEPROM.read(addr);
  res=temp/1000.0;
  Printer.concat((float)res,3);
  return (float)res;
}


double readSensor(byte n){
  
  switch (n){
    case 0: return getConductance(analogRead(FSR0)/1023.0); break;
    case 1: return getConductance(analogRead(FSR1)/1023.0); break;
  }
  //WARNING
  return -1;
}

byte createFile(){
  Printer.clear();
  getTimeStamp(true);
  Printer+=".txt";
  strcpy(fileName,Printer);
  myFile=SD.open(fileName,FILE_WRITE);
  return myFile;
}

// Gets the date and time from the ds1307
void getDateDs1307(){
  // Reset the register pointer
  Wire.beginTransmission(DS1307_I2C_ADDRESS);
  Wire.write(0);
  Wire.endTransmission();
  Wire.requestFrom(DS1307_I2C_ADDRESS, 7);
  // A few of these need masks because certain bits are control bits
  second     = bcdToDec(Wire.read() & 0x7f);
  minute     = bcdToDec(Wire.read());
  hour       = bcdToDec(Wire.read() & 0x3f);  // Need to change this if 12 hour am/pm
  Wire.read(); //ignore day of week
  day         = bcdToDec(Wire.read());
  month      = bcdToDec(Wire.read());
  year       = bcdToDec(Wire.read());
}

void toggleTXLED(){
  if ((PORTD & (0b00100000))==0)
    TXLED0;
   else
    TXLED1;
}

boolean isMeasurementNeeded(){
 
  interruptsOccured++;
  if (interruptsOccured==T_MSRMNT){
    interruptsOccured=0;
    partsOfSecond++;
  
  if (partsOfSecond==F_WINDOW){
      partsOfSecond=0;
      secondsPassed++;
      needToMeasureVoltage=true;
      needToCheckPressure=true;
    
  if (secondsPassed==period){
      secondsPassed=0;
   }
  if (secondsPassed==windowPeriod){
      measured=true;
    }
  }
  }
  return false;
}

ISR(INT3_vect) 
{ 
 

    if (isMeasurementNeeded()){
        tempSum0+=readSensor(0);
        actualNumberOfMeasurements0++;
   
        tempSum1+=readSensor(1);
        actualNumberOfMeasurements1++;
      }
       
 
}

void initSQWInterrupt(){
  EICRA = (1<<ISC30)|(1<<ISC31); // sets the interrupt type
  EIMSK = (1<<INT3); // activates the interrupt

}
void getStringFromSerial(byte length,boolean settingTimestamp){
    char inputChar;

    byte i=0;
    Printer.clear();
    while (1){
       if (Serial.available() > 0 ){
         inputChar=Serial.read();
         if (i<length && (settingTimestamp||(inputChar !='\r'&& inputChar !='\n' ))){  
            
            if (settingTimestamp){
             switch (i){
               case 0: second=inputChar; break;
               case 1: minute=inputChar; break;
               case 2: hour=inputChar; break;
               case 3: day=inputChar; break;
               case 4: month=inputChar; break;
               case 5: year=inputChar; break;
             }
           }    
            else{     
             Printer+=inputChar;  
            }    
                  
             Serial.write(inputChar);             
             Serial.flush();
            
           i++;
           }
         else break;
                
           }
           
         }
   Serial.write("\n\r");
}


double getConductance(double x){
  if (x==1) x=0.99999999;
  return (x/((1.0-x)*R))*MULTIPLY;
}


boolean cardSetup(void){

  pinMode(SS, OUTPUT);     // change this to 53 on a mega
  if (!card.init(SPI_HALF_SPEED, SS)||!SD.begin() || !volume.init(card)) {
    return false;
  } 
  root.openRoot(volume);
  return true;
}

void setupPins(){
  pinMode(LED_PWR,OUTPUT);
  TX_RX_LED_INIT;
 
  pinMode(LED_MAX,OUTPUT);
  pinMode(LED_MIN,OUTPUT);
  pinMode(LED_SENSOR,OUTPUT);
  pinMode(FSR0,INPUT);
  pinMode(FSR1,INPUT);
  pinMode(VCC,INPUT);
  pinMode(STAT,INPUT);
}

void checkBatteryVoltage(){
  if (needToMeasureVoltage){
    float voltage=getBatteryVoltage();
    
    if (voltage<VCC_MIN_WARN){
      toggleTXLED();
    }
    else {
      TXLED1;
    }
    
    if (voltage<VCC_MIN){
      EIMSK = (0<<INT3); // activates the interrupt
      digitalWrite(LED_PWR,LOW);
      digitalWrite(LED_MAX,LOW);
      digitalWrite(LED_MIN,LOW);
      digitalWrite(LED_SENSOR,LOW);
      while(1){
        delay(500);
        toggleTXLED();
      }
    }
    needToMeasureVoltage=false;
  }
}




double getBatteryVoltage(){
  return analogRead(VCC)/1023.0*3.0*2.0;
}
void checkPressure(){
  if (needToCheckPressure){
    checkingSeconds++;
    if (checkingSeconds>=T_PRESSURE_CHECK){
      firstSensor=!firstSensor;
      checkingSeconds=0;
    }
    float pressure;
    byte n=10;
    byte ledSignal;
    byte sensor;
    if (firstSensor){
      ledSignal=HIGH;
      sensor=0;
    }
    else{
      ledSignal=LOW;
      sensor=1;
    }
    for(byte i=0;i<n;i++){
      pressure+=readSensor(sensor);
    }
    pressure=pressure/n;
    digitalWrite(LED_MIN,LOW);
    digitalWrite(LED_MAX,LOW);
    digitalWrite(LED_SENSOR,ledSignal);
    if (pressure>maxMeasurement)
      digitalWrite(LED_MAX,HIGH);
    else if(pressure<minMeasurement)
      digitalWrite(LED_MIN,HIGH);
    needToCheckPressure=false;
  } 
}
void loadSensorValuesToPrinter(double s0, double s1){
  Printer.concat(s0,3);
  Printer+=" ";
  Printer.concat(s1,3);
}

void processChargerStatus(){
  if(digitalRead(STAT)==HIGH) TXLED0;
   else TXLED1;
}

void setup()
{
  delay(200);
  Wire.begin();
  
  setupPins();
  maxMeasurement=convertStoredToFloat(MAX_ADDR);
  minMeasurement=convertStoredToFloat(MIN_ADDR);
  period=EEPROM.read(PERIOD_ADDR);
  windowPeriod=EEPROM.read(W_PERIOD_ADDR);
  if (period<=5 || period ==255) period = 10;
  if (windowPeriod <=0 || windowPeriod>=period) windowPeriod=period+5;
  if (maxMeasurement>2 || maxMeasurement<0) maxMeasurement =0.1;
  if (minMeasurement>2 || minMeasurement<0) maxMeasurement =1.5;
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  setRTCSQW();
 

}


void loop()
{
   
 
  if (getBatteryVoltage() >4.0){
    byte incomingByte;
   
    
    Serial.begin(9600);
    while (!Serial){
      processChargerStatus();
    }
    if (cardSetup()){
      digitalWrite(LED_PWR,HIGH);
    }
    while(1){
     Printer.clear();
      processChargerStatus();
      
      if (Serial.available() > 0) {
        // read the incoming byte:
        incomingByte = Serial.read();
        switch(incomingByte){
        case 'A':{
          loadSensorValuesToPrinter(readSensor(0),readSensor(1));
          Serial.println(Printer);
          break;}
          
        
        case 'B':{
          root.ls();
          Serial.println("RDY");
          break;}
          
        case 'C':{
          
          getStringFromSerial(FILENAMELENGTH,false);
          myFile=SD.open(Printer,FILE_READ);
          if (myFile) {
            char temp;
            while (myFile.available()) {
              temp=myFile.read();
              Serial.write(temp);
             
              if (temp=='\n') {
                while(Serial.read() != 'R');
                
              }
            }
            myFile.close();
            
          }
         Serial.println("\nRDY");
         break;
         
        }
        case 'D':{
          getStringFromSerial(FILENAMELENGTH,false);
          SD.remove(Printer);  
         break; 
        }
        
        case 'E':{
            convertStoredToFloat(MIN_ADDR);
            Printer+="\n\r";
            Serial.write(Printer);
            break;
        }
        
        case 'F' :{
            getStringFromSerial(5,false);
            convertFloatAndStore(MIN_ADDR);
            break;
        }
        
        case 'G':{
            convertStoredToFloat(MAX_ADDR);
            
           
            Printer+="\n\r";
            Serial.write(Printer);
            break;
        }
        
        case 'H' :{
            getStringFromSerial(5,false);
            convertFloatAndStore(MAX_ADDR);
            break;
        }
        
        case 'I':{
            Printer+=EEPROM.read(PERIOD_ADDR);
            Printer+="\n\r";
            Serial.write(Printer);
            break;
        }
        
        case 'J' :{
            getStringFromSerial(5,false);
            EEPROM.write(PERIOD_ADDR,atoi(Printer));
            break;
        }
        
        case 'K':{
            getTimeStamp(false);
            Serial.println(Printer);
            break;
        }
        
        case 'L':{
            getStringFromSerial(6,true);
            setDateDs1307();
            break;
        }
        
        case 'M':{
            Printer+=EEPROM.read(W_PERIOD_ADDR);
            Printer+="\n\r";
            Serial.write(Printer);
            break;
        }
        
        case 'N':{
            getStringFromSerial(5,false);
            EEPROM.write(W_PERIOD_ADDR,atoi(Printer));
            break;
        }
        
        }

      }
    }
  }
  else{

    while (!(cardSetup()&&createFile())){
     
      delay(100);
    }
     
    checkBatteryVoltage(); 
    initSQWInterrupt();
    digitalWrite(LED_PWR,HIGH);
    
    while (1){
   
      if (measured){
        Printer.clear();
        getTimeStamp(false);
        float avrg0=(actualNumberOfMeasurements0==0)? 0 : tempSum0/actualNumberOfMeasurements0;
        float avrg1=(actualNumberOfMeasurements1==0)? 0 : tempSum1/actualNumberOfMeasurements1;
          
        tempSum0=0.0;
        tempSum1=0.0;
        actualNumberOfMeasurements0=0;
        actualNumberOfMeasurements1=0;
        
        Printer+=" ";
        loadSensorValuesToPrinter(readSensor(0),readSensor(1));
        myFile=SD.open(fileName,FILE_WRITE);
        if (myFile){
          
           myFile.println(Printer);
           myFile.close();
           
        }
        
        measured=false;
         
      
      }
         checkBatteryVoltage();
         checkPressure();
         sleep_mode();
    }
    
    
  }
        
        
  
}
