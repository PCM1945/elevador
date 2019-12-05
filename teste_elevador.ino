#include <Ultrasonic.h>
/*
ponteh
3 pwm
2 ina
4 inb
sensor 
trig 5
echo 6
*/

// define as distancias dos andares
#define F1 127.35
#define F2 87.00
#define F3 46.00
#define F4 5.92
// define os pinos para a ponteH
const byte pwm = 10;
const byte ina = 12;
const byte inb = 11;
//Define os pinos para o trigger e echo
const byte trig = 5;
const byte echo = 6;
/*// define os botões
const byte b1;
const byte b2;
const byte b3;
const byte b4;
*/
// define variavelde controle do motor
char resp;
// define variavel que guarda set points
float setpont;

void mov_down(byte a, byte b){
  digitalWrite(a, 0);
  digitalWrite(b, 1);
}

void mov_up(byte a, byte b){
  digitalWrite(a, 1);
  digitalWrite(b, 0);
}

void stp(byte a, byte b){
  digitalWrite(a, 1);
  digitalWrite(b, 1);  
}

void setup(){
  Serial.begin(9600);
  pinMode(pwm, OUTPUT);
  pinMode(ina, OUTPUT);
  pinMode(inb, OUTPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
}

//Inicializa o sensor nos pinos definidos acima
Ultrasonic ultrasonic(trig, echo);

float dist(){
  long microsec = ultrasonic.timing();
  return ultrasonic.convert(microsec, Ultrasonic::CM);
}

void loop(){
  /*
  Serial.print("distancia: ");
  Serial.println(dist());
  */
  delay(100);
  mov_down(ina, inb);
  delay(700);
  mov_up(ina, inb);
  delay(700);
  stp(ina, inb);
  delay(700);
}
