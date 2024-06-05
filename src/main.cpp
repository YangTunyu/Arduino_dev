#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}


#include <MQUnifiedsensor.h>

//Definitions
#define placa "Arduino UNO"
#define Voltage_Resolution 5
#define pin 13 //Analog input 4 of your arduino
#define type "MQ2" //MQ2
#define ADC_Bit_Resolution 10 // For arduino UNO/MEGA/NANO
#define RatioMQ2CleanAir 4.4  //RS / R0 = 4.4 ppm 
//#define calibration_button 13 //Pin to calibrate your sensor

//Declare Sensor
MQUnifiedsensor MQ2(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);
#define alarmPin 18
void setup() {
  //Init serial port
  Serial.begin(115200);
  //Set math model to calculate the PPM concentration and the value of constants
  MQ2.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ2.setA(30000000); MQ2.setB(-8.308); // Configure the equation to to calculate CH4 concentration

  MQ2.init(); 
 
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ2.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ2.calibrate(RatioMQ2CleanAir);
    Serial.print(".");
  }
  MQ2.setR0(calcR0/10);
  Serial.println("  done!.");
  
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while(1);}
  /*****************************  MQ CAlibration ********************************************/ 
  MQ2.serialDebug(true);
  pinMode(alarmPin, OUTPUT);
  digitalWrite(alarmPin,HIGH);
}

  void loop() {

  MQ2.update();
  float smokePPM = MQ2.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  if(smokePPM > 50  ) {Serial.println("Warning: High concentrations of smoke detected");
    digitalWrite(alarmPin, LOW); // 触发警报
 } else {
    Serial.println("Smoke concentration normal");
    digitalWrite(alarmPin, HIGH); // 关闭警报
 }
 Serial.print("Smoke Concentration: "); 
 Serial.print(smokePPM); 
 Serial.println(" ppm");
  MQ2.serialDebug(); // Will print the table on the serial port
  delay(4000);  
}

