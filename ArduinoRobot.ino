#include <SoftwareSerial.h>
#include <Servo.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <DHT_U.h>
//Thanks to https://github.com/robsoncouto/arduino-songs/blob/master/imperialmarch/imperialmarch.ino
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
#define REST      0

//SERVO
Servo servo1;

//bluetooth
SoftwareSerial miBT(10,11);
//SoftwareSerial miBT(0,1); //prueba para conectar al sensorshield, no parece funcionar, no recupera datos corréctamente
// está preconstruido en el 0,1 pero hay que conectar RX y TX al revés y no se puede utilizar para obtener mensajes o depurar la salida
// además hay que configurar la emisión al valor por defecto del bluetooh a 9600 baudios, el cable RXD se conectaría al pin D0 y el cable TXD se conectaría al pin D1 (más a la derecha)
// si miramos de frente

const int melody[] = {
  
  // Dart Vader theme (Imperial March) - Star wars 
  // Score available at https://musescore.com/user/202909/scores/1141521
  // The tenor saxophone part was used
  
  NOTE_A4,-4, NOTE_A4,-4, NOTE_A4,16, NOTE_A4,16, NOTE_A4,16, NOTE_A4,16, NOTE_F4,8, REST,8,
  NOTE_A4,-4, NOTE_A4,-4, NOTE_A4,16, NOTE_A4,16, NOTE_A4,16, NOTE_A4,16, NOTE_F4,8, REST,8,
  NOTE_A4,4, NOTE_A4,4, NOTE_A4,4, NOTE_F4,-8, NOTE_C5,16,

  NOTE_A4,4, NOTE_F4,-8, NOTE_C5,16, NOTE_A4,2,//4
  NOTE_E5,4, NOTE_E5,4, NOTE_E5,4, NOTE_F5,-8, NOTE_C5,16,
  NOTE_A4,4, NOTE_F4,-8, NOTE_C5,16, NOTE_A4,2,
  
  NOTE_A5,4, NOTE_A4,-8, NOTE_A4,16, NOTE_A5,4, NOTE_GS5,-8, NOTE_G5,16, //7 
  NOTE_DS5,16, NOTE_D5,16, NOTE_DS5,8, REST,8, NOTE_A4,8, NOTE_DS5,4, NOTE_D5,-8, NOTE_CS5,16,

  NOTE_C5,16, NOTE_B4,16, NOTE_C5,16, REST,8, NOTE_F4,8, NOTE_GS4,4, NOTE_F4,-8, NOTE_A4,-16,//9
  NOTE_C5,4, NOTE_A4,-8, NOTE_C5,16, NOTE_E5,2,

  NOTE_A5,4, NOTE_A4,-8, NOTE_A4,16, NOTE_A5,4, NOTE_GS5,-8, NOTE_G5,16, //7 
  NOTE_DS5,16, NOTE_D5,16, NOTE_DS5,8, REST,8, NOTE_A4,8, NOTE_DS5,4, NOTE_D5,-8, NOTE_CS5,16,

  NOTE_C5,16, NOTE_B4,16, NOTE_C5,16, REST,8, NOTE_F4,8, NOTE_GS4,4, NOTE_F4,-8, NOTE_A4,-16,//9
  NOTE_A4,4, NOTE_F4,-8, NOTE_C5,16, NOTE_A4,2,
  
};

//MOTOR DC
#define IN1 7 
//pines para dc
#define IN2 8
#define ENA 5
//int IN1 = 7; 
//int IN2= 8;
//int ENA = 5;  //pin pwm


#define IN4 3
#define IN3 4
#define ENB 6 //pin pwm
// change this to make the song slower or faster
byte velocidad=0;

byte tempo = 120;

// change this to whichever pin you want to use
#define buzzer  9
//variable para bluetooh

char DATO=0;

//sensor ultrasónico
//int TRIG=12; es A0
//int ECO=13; es A1
long DURACION;
long DISTANCIA;



//Servo
#define PINSERVO 2
#define PULSOMIN 700
#define PULSOMAX 2500
#define temposervo 1000
//PANTALLA LCD_ Crear el objeto lcd  dirección  0x3F y 16 columnas x 2 filas
// Se conecta a la SENSOR SHIELD V5 pin A5 y PIN A4, por lo que se aprovechan los pines analógicos
LiquidCrystal_I2C lcd(0x3F,16,2);
#define mensaje_lcd(cadena){ lcd.init(); lcd.print(cadena); }

 //int ledPIN = 13;
  
  //int ESTADO = LOW;
//SENSOR TEMPERATURA** NO SE USA  
//int SENSOR=4;
//int TEMPERATURA;
//int HUMEDAD;
// DHT dht(SENSOR,DHT11);

 /********************************
  * SETUP
  *******************************/
void setup() {
  //Sensor ultrasónico
   //pinMode(TRIG,OUTPUT);
   //pinMode(ECO,INPUT);
   pinMode(A0,OUTPUT); //TRIGGER SENSOR SHIELD V5, equivalente al pin digital 14
   pinMode(A1,INPUT); //ECO SENSOR SHIELD V5, equivalente al pin digital 15
   
   pinMode(buzzer,OUTPUT);
    Serial.begin(9600);
      Serial.println(F("Comenzamos..."));

      //LCD Pantalla
        lcd.init();
        lcd.begin(16,2); 
      //dht.begin();
  
  //Encender la luz de fondo.
  lcd.backlight();
  mensaje_lcd(F("Buenos dias"));
   delay(2000);
  // Escribimos el Mensaje en el LCD.
  //lcd.print("Hola Mundo");
 //lcd.init();

   Serial.println(F("Fin setup..."));
   
// song();
//melodia_r2d3();
//motor DC
pinMode(IN1,OUTPUT);
pinMode(IN2,OUTPUT);
pinMode(ENA,OUTPUT);

pinMode(IN3,OUTPUT); 
pinMode(IN4,OUTPUT);
pinMode(ENB,OUTPUT); 

//sensorth();


//BLUETOOTH
//se configuró a 38400, si se utilizan los pines 0 y 1 debe dejarse el valor por defecto, 9600
miBT.begin(38400);
//  Serial.begin(9600); // Serial enabled to allow Bluetooth communications
  delay(2000);
//PREPARADO!!-> servomotor();
//SERVO
servo1.attach(PINSERVO);//,PULSOMIN,PULSOMAX);
servomotor(90);
   //servomotor(1);                                                                                                        
}

/*********************************
 * LOOP
 *********************************/
void loop() {


      // servomotor();
   moduloBluetooth();
   //motordc();
}

//**SENSORUS
int sensorUS(int switch_sensorus)
{
  int iSensorUs=0;
  //Serial.println("Entramos en SensorUS");
 /* 
 if(i== 100){ //Cada 100 ciclos actualizamos la temperatura y el sensor
  i=0;
  sensorth();
 // motordc();
 }
i++;
*/
 //lcd.setCursor(0,1);
 //lcd.print("");
 if(switch_sensorus==1){
 digitalWrite(A0,LOW);
 delayMicroseconds(4); 
 digitalWrite(A0,HIGH); 
 delayMicroseconds(10);
 digitalWrite(A0,LOW);
 DURACION = pulseIn(A1,HIGH);
 Serial.println();
 Serial.print(F(" DURACION: "));
 Serial.print(DURACION);
 DISTANCIA = DURACION / 59;
 Serial.print(F(" DISTANCIA (cm): "));
 Serial.print(DISTANCIA);

 delay(100);
 if(DISTANCIA >=0 && DISTANCIA <=50){
 
  //digitalWrite(ledPIN, HIGH);
  //delay(DISTANCIA*10);
   iSensorUs=1;
   Serial.print(F("  iSensorUS: "));
    Serial.print(iSensorUs);
    mensaje_lcd("Dis: ");
   // lcd.setCursor(1,0);
   lcd.print(DISTANCIA);       
   return 1;
 }
 else
 {iSensorUs=0;
 Serial.print(F("  iSensorUS: "));
Serial.print(iSensorUs);
  return 0;
 }
 }
 else 
 return 0;



   
}

/*
 * en combinación con sensor ultra sonido irá alternando la ruta
 */


/*
 * MOTOR AUTONOMO
 */
void motordc_autonomo(int pasos)
{
 // Serial.println("eNTRÉ en motordc");
  //MOTOR DC
byte i=0;
byte gradoservo=0;
while(i<=pasos)
{
  mensaje_lcd(F("Modo auto"));
    
  //sensorUS(1);
  servomotor(90);  //no encapsular el código en una función para permitir interactuar con el sensor
     
 Serial.println(F("MOTORDC AUTONOMO - SEGUIMOS CAMINO "));   
 
 
 while(sensorUS(1)==0 && i <=pasos){ 
   
   
    gradoservo=servomotor(45);
    if(sensorUS(1)==1)
     break;
    gradoservo=servomotor(0);
    if(sensorUS(1)==1)
     break;
    gradoservo=servomotor(45);
    if(sensorUS(1)==1)
     break;

     gradoservo=servomotor(90);
     if(sensorUS(1)==1)
     break;
     gradoservo=servomotor(135);
     if(sensorUS(1)==1)
     break;
     gradoservo=servomotor(180);
     if(sensorUS(1)==1)
     break;
     gradoservo=servomotor(135);
     if(sensorUS(1)==1)
     break;
     gradoservo=servomotor(90);
    if(sensorUS(1)==1)
     break;

   
     
 
 
    
   motordc_del(500,200);
   i++;
 }//end while
if(sensorUS(1)==1)
 { Serial.println(F(" MOTORDC AUTONOMO ACCION EVASIVA"));
     motordc_stop();
     
     servomotor(90); 
     
    tone(buzzer, NOTE_A5, 4); 
    tone(buzzer, NOTE_A5, 4); 
   
    motordc_tras(1000,200);
    if(gradoservo < 90)
    {  
      motordc_izq(800,200); //motor derecho adelante
    }
    else{
         if(gradoservo > 90)
         {  
            motordc_der(800,200); //motor izquierdo adelante
  
         }
         
    }
   motordc_stop();
     
 }

   i++;
   Serial.println(F(" PASO NÚMERO: "));
Serial.println(i);

 }//end while

       sensorUS(0);
        servomotor(90); 
        motordc_stop();
         noTone(buzzer);
 mensaje_lcd(F("Modo auto off"));
}
void motordc_izq(unsigned long tdelay, byte vel){
  
Serial.println(F("accionamos rueda derecha y paramos izquierda para girar a izquierda en el pin PWM: "));
Serial.println(ENA);
analogWrite(ENA, vel);
   digitalWrite(IN1, HIGH);
   digitalWrite(IN2, LOW);
   //mensaje_lcd(F("Gira izq"));
   if(tdelay!=0)
   delay(tdelay);
}

void motordc_der(unsigned long tdelay, byte vel){
  
Serial.println(F("accionamos rueda izquierda y paramos derecha para girar a derecha en el pin PWM: "));
Serial.println(ENB);
   analogWrite(ENB, vel);
   digitalWrite(IN3, LOW);
   digitalWrite(IN4, HIGH);
   //mensaje_lcd(F("Gira der"));
   if(tdelay!=0)
   delay(tdelay);
 
}

void motordc_tras(unsigned long tdelay, byte vel){
  
Serial.println(F("accionamos dos ruedas hacia detrás"));
  analogWrite(ENB, vel);
   digitalWrite(IN3, HIGH);
   digitalWrite(IN4, LOW);


   analogWrite(ENA, vel);
   digitalWrite(IN1, LOW);
   digitalWrite(IN2, HIGH);
   //mensaje_lcd(F("Mueve atras"));
   if(tdelay!=0)
   delay(tdelay); 

}

void motordc_del(unsigned long tdelay, byte vel){

Serial.println(F("accionamos dos ruedas hacia delante"));                                                                                                                                                  
 
    analogWrite(ENB, vel);
   digitalWrite(IN3, LOW);
   digitalWrite(IN4, HIGH);
   
 analogWrite(ENA, vel);
   digitalWrite(IN1, HIGH);
   digitalWrite(IN2, LOW);
     //  mensaje_lcd(F("Mueve del"));
   if(tdelay!=0)
   delay(tdelay);
   
}

void motordc_stop(){
    

    digitalWrite(ENA, LOW);
    digitalWrite(ENB, LOW);
      digitalWrite(IN1, LOW);
   digitalWrite(IN2, LOW);
     digitalWrite(IN3, LOW);
   digitalWrite(IN4, LOW);
  // mensaje_lcd(F("Parado"));
}

byte servomotor(byte grado){
 
/*if(switch_servo==1)
{
  for (int i=90;i<=180;i++){
   servo1.write(i);
   delay(temposervo);
  }
  for (int i=180;i>=0;i--)
 {
   servo1.write(i);
   delay(temposervo);
 }

 for (int i=0;i<=90;i++){
   servo1.write(i);
   delay(temposervo);
  }
 
}
*/
servo1.write(grado);
delay(temposervo);
return servo1.read();  
}


void moduloBluetooth()
{
  
  //Bluetooth Serial Controller es la app que envía comandos al módulo bluetooh que luego utilizamos para realizar acciones
  //se conecta al pin 10 y 11 de arduino 
  //TXD 10
  //RXD 11 
  //REQUIERE LA LILBRERÍA #include <SoftwareSerial.h>
  //y la declaración del objeto SoftwareSerial miBT(10,11);
 //Serial.println("entrando modulo bluetooth");
  if(miBT.available()){   //lee BT y envía a Arduino
  // if(Serial.available()){
    //Serial.write(miBT.read()); //modo maestro
    //delay(15);
    //Serial.println("Bluetooh disponible");
    DATO = miBT.read();
    //DATO = Serial.read(); //codigo si utilizamos pines 0 y 1 a 9600 baudios
    
    Serial.print(F("DATO: "));
    Serial.println(DATO);
    switch(DATO){
    case '1':
      Serial.println(F("Pulsamos el 1"));
      melodia_r2d3();
       break;
    case '2':   
        
       motordc_del(0,185);
       
      //Serial.print("DATO: ");
      //Serial.println(DATO);
      break;
     case '3':
        song();
        break;
    case '4':
      
       motordc_izq(0,200);
       break;
    case '5':
       
        motordc_stop();
         
        break;
    case '6':
      
        motordc_der(0,200);
        break;
     case '8':
     
      motordc_tras(0,185);
        break;
     case '7':
        
           motordc_autonomo(5); //5 pasos
           break;
    }
 
  //if(Serial.available()) modo esclavo
   //miBT.write(Serial.read()); //lee Arduino y envia a BT
  } 
}


//void debug_bluetooth()
//{

  //if(miBT.available()){   //lee BT y envía a Arduino
//requiere conexión a pines digitales y no los preconstruidos para el puerto serie 0 y 1
  //Serial.write(miBT.read()); //modo maestro
    //delay(15);
    //Serial.println("Bluetooh disponible");
    //}
  //if(Serial.available()) modo esclavo
   //miBT.write(Serial.read()); //lee Arduino y envia a BT
   
//}
/*void mensaje_lcd(String mensaje)
{
   lcd.init();
    lcd.print(mensaje);
}
*/
void  melodia_r2d3()
{
   Serial.print(F("Entre en melodia"));
 // tone(buzzer, 880, 1000);
 // delay(1000);
 // noTone(buzzer);
    mensaje_lcd(F("Melod r2d3"));
        int melody[] = { 
      NOTE_B1,NOTE_A2,NOTE_B1,NOTE_G4,NOTE_A5,NOTE_B4 };
  int noteDurations[] = { 4, 8,8,2,4,4}; 
   // int noteDurations[] = { 8, 8,8,1}; 
    for (int thisNote = 0; thisNote < 6; thisNote++) {  
      // to calculate the note duration, take one second   
      // divided by the note type.  
      //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.  
      int noteDuration = 1000 / noteDurations[thisNote];  
      tone(buzzer, melody[thisNote], noteDuration);  

      // to distinguish the notes, set a minimum time between them.  
      // the note's duration + 30% seems to work well:  
      int pauseBetweenNotes = (int) ( noteDuration * 1.30 );  
      delay(pauseBetweenNotes); 
      
    }  
    noTone(buzzer); 

}

void song()
{
   mensaje_lcd(F("Melod Marcha Im"));
 //lcd.print("Comenzando melodia");

Serial.print(F("Entre en song"));
// sizeof gives the number of bytes, each int value is composed of two bytes (16 bits)
// there are two values per note (pitch and duration), so for each note there are four bytes
int notes = sizeof(melody) / sizeof(melody[0]) / 2;

// this calculates the duration of a whole note in ms
int wholenote = (60000 * 4) / tempo;

int divider = 0, noteDuration = 0;

 for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {

    // calculates the duration of each note
    divider = melody[thisNote + 1];
    if (divider > 0) {
      // regular note, just proceed
      noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
      // dotted notes are represented with negative durations!!
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5; // increases the duration in half for dotted notes
    }

    // we only play the note for 90% of the duration, leaving 10% as a pause
    tone(buzzer, melody[thisNote], noteDuration*0.9);

    // Wait for the specief duration before playing the next note.
    delay(noteDuration);
    
    // stop the waveform generation before the next note.
    noTone(buzzer);
  }

  
}


/*
void sensorth()
{

   //LCD
  TEMPERATURA=dht.readTemperature();
 HUMEDAD=dht.readHumidity();
Serial.print(" TEMPERAT.: ");
 Serial.print(TEMPERATURA);
 Serial.print(" HUMEDAD ");

   Serial.print(HUMEDAD);
 /* lcd.init();
 lcd.print("TEM:");
 lcd.print(TEMPERATURA);
 lcd.print("C");
 //lcd.setCursor(0,1);
 lcd.print(" HUM:");
 lcd.print(HUMEDAD);
 lcd.print("%");
 lcd.setCursor(0,1);
 */
 /* if(DISTANCIAMIN<30){
 lcd.print("U DIS DET:");
  lcd.print(DISTANCIAMIN);
 lcd.print("CM");
 }
*/
//}
