#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <PID_v1.h>
#include <EEPROM.h>

// Pinout
const int heaterSensorPin = A0; // Пин куда подключен выход с ОУ
const int SolderMosfetPin = 11;       // Пин куда подключен выход на пояльник
const int ledPin = 12;          // Пин куда подключен светодиод индикации
const int faultTemp = 0;        // Погрешность температоруры.

const int buttonUp = 8;
const int buttonDown = 10;
const int buttonOk = 9;


int SoldertempRead = 0;
int tempreal = 0;
int incomingByte = 0;
int setTemp = 180;
int flag = 0;
int solder = 0;

int SetTemp = 180;

int i = 0;                // loop count 

int addrEeprom = 0;

//задаем начальные значения для pid регулятора
float Solderkp = 2.0;
float Solderki = 0.4;
float Solderkd = 0.1;
//подключаем PID.
double SolderSetpoint, SolderInput, SolderOutput;
PID SolderPID(&SolderInput, &SolderOutput, &SolderSetpoint, Solderkp, Solderki, Solderkd, DIRECT);


// Экран

// pin 3 - Serial clock out (SCLK)
// pin 4 - Serial data out (DIN)
// pin 5 - Data/Command select (D/C)
// pin 7 - LCD chip select (CS/CE)
// pin 6 - LCD reset (RST)

Adafruit_PCD8544 display = Adafruit_PCD8544(3, 4, 5, 7, 6);





void setup() {
  // make some light to show it is alive
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  
  pinMode(SolderMosfetPin, OUTPUT);
  digitalWrite(SolderMosfetPin, LOW);
  // serial output init
  Serial.begin(115200);
  SolderPID.SetOutputLimits(0, 255);

  
  pinMode(buttonUp, INPUT);
  digitalWrite(buttonUp, HIGH);
  
  pinMode(buttonOk, INPUT);
  digitalWrite(buttonUp, HIGH);
  
  pinMode(buttonDown, INPUT);
  digitalWrite(buttonUp, HIGH);
  
  display.begin();
  display.setContrast(50); //Ajusta o contraste do display
  display.clearDisplay();   //Apaga o buffer e o display
  display.setTextSize(1);  //Seta o tamanho do texto
  display.setTextColor(BLACK); //Seta a cor do texto
  display.setCursor(0,0);  //Seta a posição do cursor
  display.println();  

  

  display.println();

  display.setTextSize(1); 
  display.setTextColor(BLACK);
  display.print("Smok Solder");
  display.display();
  delay(100);

SolderSetpoint = EEPROM.read(addrEeprom) * 4;

}


class Sensors{
    unsigned long int countTempRead = 0;
    unsigned long int TempRead = 0;
    int sensorPin = A0;
    int res = 0;
    int sensorAverage = 64;

    public:
    Sensors(int pin, int average=64)
    {
      sensorPin = pin;
      sensorAverage = average;

  
    }
    
    // Округляем значения поулученные с сенсора.
    int getOversampled() {
        
        if (countTempRead != sensorAverage)
        {
          TempRead += analogRead(sensorPin);
          countTempRead ++;
      
        }
        else
        {
          res = TempRead / sensorAverage; // Делим знаячение на колличество сложений
          TempRead = 0;
          countTempRead = 0;   
        } 
        
    return res; 
    }
};



// Возвращаяем реальную температору
int getRealTemp(int temp){
  
  int tempres=map(temp,-50,700,0,500);   // нужно вычислить
  tempres = tempres - faultTemp;
  return tempres;
  }


Sensors SolderSensor(heaterSensorPin, 32);


void loop() {

      SoldertempRead = SolderSensor.getOversampled();
//      tempRead = analogRead(heaterSensorPin);    
      tempreal = getRealTemp(SoldertempRead);


      if (Serial.available() > 0) {
                    // read the incoming byte:
                    incomingByte = Serial.read();
                    switch (incomingByte) {
                          case 48:
                              
                              SolderPID.SetMode(MANUAL);
                              SolderOutput = 0;
                              Serial.println("Solder OFF");
                              break;
                          case 49:
                              EEPROM.write(addrEeprom, 200);
                              Serial.println("eeprom write");
                              break;

                          case 50:
                              SolderInput = tempreal;
                              SolderSetpoint = 200;
                              SolderPID.SetMode(AUTOMATIC);
                              Serial.println("Solder ON 200");
                              break;
                          case 51:
                              SolderInput = tempreal;
                              SolderSetpoint = 250;
                              SolderPID.SetMode(AUTOMATIC);
                              Serial.println("Solder ON 250");
                              break;
                          case 52:
                              SolderInput = tempreal;
                              SolderSetpoint = 280;
                              SolderPID.SetMode(AUTOMATIC);
                              Serial.println("Solder ON 280");
                              break;
                          case 53:
                              SolderInput = tempreal;
                              SolderSetpoint = 300;
                              SolderPID.SetMode(AUTOMATIC);
                              Serial.println("Solder ON 300");
                              break;
                          case 13: // перевод коретки ничего не печатаем
                              break;
                          default:
                              Serial.print("I received: ");
                              Serial.println(incomingByte, DEC);
                         }
                    // say what you got:
                    
      }



      // выключатель паяльника
      if(digitalRead(buttonOk)==HIGH&&flag==0)//если кнопка нажата    
        // и перемення flag равна 0 , то ... 
      { 
          if (solder==1){
              SolderInput = tempreal;
              
              SolderPID.SetMode(AUTOMATIC);
              Serial.println("Solder ON 250");
              solder = 0;
             
            }
            else
            { 
              SolderPID.SetMode(MANUAL);
              SolderOutput = 0;
              Serial.println("Solder OFF");
              solder = 1;
              
              }
          
          flag=1; 
              
       } 
        
      if(digitalRead(buttonOk)==LOW&&flag==1)//если кнопка НЕ нажата 
         //и переменная flag равна - 1 ,то ... 
         { 
                
            flag=0;//обнуляем переменную flag 
         } 

      // задаем температуру
      if (SolderSetpoint>512){
          SolderSetpoint= 0;
          EEPROM.write(addrEeprom, SolderSetpoint);
        }


      if(digitalRead(buttonUp)==LOW){
        SolderSetpoint = SolderSetpoint + 1;
        EEPROM.write(addrEeprom, SolderSetpoint / 4);
        delay(200);
        }

      if(digitalRead(buttonDown)==LOW){
        SolderSetpoint = SolderSetpoint - 1;
        EEPROM.write(addrEeprom, SolderSetpoint / 4);
        delay(200);
        }

      SolderInput = tempreal;
      SolderPID.Compute();
      analogWrite(SolderMosfetPin,SolderOutput);
      
      if (++i == 100) { // print debug info every 100th loop 
        i = 0;
        Serial.print("Heater: ");
        Serial.print(SoldertempRead);
        Serial.print(", PID: ");
        Serial.print(SolderOutput);
        Serial.print(", Temp Real: ");
        Serial.println(tempreal);
    //    Serial.print(";, ");
    //    Serial.print(heaterSensorResistance, 4);
 
      }

      int displaySolderSetpoint = SolderSetpoint;
      display.clearDisplay();
      display.clearDisplay();   //Apaga o buffer e o display
      display.setTextSize(1);  //Seta o tamanho do texto
      display.setTextColor(BLACK); //Seta a cor do texto
      display.setCursor(0,0);  //Seta a posição do cursor
      display.print("Heater: "); 
      display.println(SoldertempRead); 
      display.print("PID: "); 
      display.println(SolderOutput); 
      display.println("Temp Real:");
      
      display.setTextSize(2); 
      display.print("  ");
      display.println(tempreal);
      display.setTextSize(1);
      display.print("Temp Set:"); 
      display.println(displaySolderSetpoint);
      display.display();
//     delay(10);

}



