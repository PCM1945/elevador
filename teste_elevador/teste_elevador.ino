#include <PID_v1.h>
/*
ponteh
10 pwm
12 ina
11 inb
sensor 
trig 5
echo 6
*/


// define as distancias dos andares
#define F1 121.00
#define F2 82.00
#define F3 43.00
#define F4 2.00

// define os pinos para a ponteH
const byte pwm = 10;
const byte ina = 12;
const byte inb = 11;

//Define os pinos para o trigger e echo
const byte trig = 5;
const byte echo = 6;

// define os botões
const byte b1 = 3;
const byte b2 = 2;
const byte b3 = 7;
const byte b4 = 4;

// define variavel que guarda set points
double setpoint = 5.92;

// define as variaveis do botão
bool b1_value;
bool b2_value;
bool b3_value;
bool b4_value;

// define as constantes do PID
double kp = 55;
double ki = 35;
double kd = 70;

// defiene os parametros do pid
double input;
double output;
double echo_time;

// cria o objeto PID
PID myPID(&input, &output, &setpoint, kp,ki,kd, DIRECT);

// funções de movimento;
void mov_down(byte a, byte b){
 digitalWrite(a, LOW);
  digitalWrite(b, HIGH);
}

void mov_up(byte a, byte b){
   digitalWrite(a, HIGH);
   digitalWrite(b, LOW);
}

void stp(byte a, byte b){
   digitalWrite(a, HIGH);
  digitalWrite(b, HIGH); 
}


void setup(){
  myPID.SetMode(AUTOMATIC);
  Serial.begin(9600);
  pinMode(pwm, OUTPUT);
  pinMode(ina, OUTPUT);
  pinMode(inb, OUTPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(b1, INPUT_PULLUP);
  pinMode(b2, INPUT_PULLUP);
  pinMode(b3, INPUT_PULLUP);
  pinMode(b4, INPUT_PULLUP);
}

double dist(){
   double distance;
   // Start a trigger of 10us burst;
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // Distance Calculation
     distance = pulseIn(echo, HIGH);
     distance= distance/58;  
     return distance;
/* 
The speed of sound is
340 m/s or 29 us per cm.
The Ultrasonic burst travels out & back.So
to find the distance of object we divide by 58
*/
}
/*double dist(){
   long microsec = sensor.timing();
   return sensor.convert(microsec, Ultrasonic::CM);
}
*/
void loop(){ 
   input = dist();
   //Serial.println(input);
  // leitura dos botões
   b1_value = digitalRead(b1);
   b2_value = digitalRead(b2);
   b3_value = digitalRead(b3);
   b4_value = digitalRead(b4);

if(!b1_value){
  setpoint =  F1;
}
if(!b2_value){
  setpoint =  F2;
}
if(!b3_value){
  setpoint =  F3;
}
if(!b4_value){
  setpoint =  F4;
}

double vel_motor = map(output, 0, 1000, 255, 0);

myPID.Compute();
if(setpoint - input > 0){
  mov_up(ina, inb); 
  analogWrite(pwm, vel_motor);
}
else if(setpoint - input < 0){
  mov_down(ina, inb);
  analogWrite(pwm, vel_motor);
}
else if(setpoint - input == 0){
  stp(ina,inb);
}
    Serial.print(input);
    Serial.print(" ");
    Serial.print(output);
    Serial.print(" ");
   /* Serial.print(pwm); 
    */
    Serial.print(" ");
    Serial.println(setpoint);
    
}
