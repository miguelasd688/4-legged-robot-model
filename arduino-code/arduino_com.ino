#include <Servo.h>
#include <math.h>


#define MAX_GAMMA 50
///VARIABLES SERVOMOTORES
#define MAX_SERVOS 12
#define MAX_PULSE 2500
#define MIN_PULSE 560
Servo Servos[MAX_SERVOS];


//VARIABLES PARA CONTROLAR EL TIEMPO
unsigned long previousMillis = 0;
const long interval = 20;
unsigned long loopTime;
unsigned long previousLooptime;
double t;

struct angles {
  double tetta;
  double alpha;
  double gamma;
};////vector
struct angles anglesFR;
struct angles anglesFL;
struct angles anglesBR;
struct angles anglesBL;
//VARIABLES PARA RECIVIR EL COMANDO
const byte numChars = 32;
char receivedChars[numChars];
int spaceCounter = 0;

boolean newData = false;

int pulse0, pulse1, pulse2, pulse3, pulse4, pulse5, pulse6, pulse7, pulse8, pulse9, pulse10, pulse11;

char a;
int pulse;


int batterieVoltage;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  pinMode(13,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(9,OUTPUT);
  
  IMUSetup();
  connectServos();
  
  
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;
      t = float(currentMillis) / 1000;
      /////////cuenta el tiempo que tarda el bucle en ejecutarse
      loopTime = currentMillis - previousLooptime;
      Serial.print("<");
      Serial.print(loopTime);
      Serial.print("#");
      previousLooptime = currentMillis;

      readAngles();
          
      recvWithStartEndMarkers();
      
      batterieStatus();
      
      newData = false;
      moveServos(pulse0, pulse1, pulse2, pulse3, pulse4, pulse5, pulse6, pulse7, pulse8, pulse9, pulse10, pulse11);
  }
}

void batterieStatus(){
  batterieVoltage = analogRead(A1);
  
  if (batterieVoltage >= 164){
    digitalWrite(13,HIGH);
    digitalWrite(12,HIGH);
    digitalWrite(11,HIGH);
    digitalWrite(10,HIGH);
    digitalWrite(9,HIGH);
  }
  else if (batterieVoltage >= 154){
    digitalWrite(13,HIGH);
    digitalWrite(12,HIGH);
    digitalWrite(11,HIGH);
    digitalWrite(10,HIGH);
    digitalWrite(9,LOW);
  }
  else if (batterieVoltage >= 144){
    digitalWrite(13,HIGH);
    digitalWrite(12,HIGH);
    digitalWrite(11,HIGH);
    digitalWrite(10,LOW);
    digitalWrite(9,LOW);
  }
  else if (batterieVoltage >= 134){
    digitalWrite(13,HIGH);
    digitalWrite(12,HIGH);
    digitalWrite(11,LOW);
    digitalWrite(10,LOW);
    digitalWrite(9,LOW);
  }
  else if (batterieVoltage >= 124){
    digitalWrite(13,HIGH);
    digitalWrite(12,LOW);
    digitalWrite(11,LOW);
    digitalWrite(10,LOW);
    digitalWrite(9,LOW);
  }
  else{
    digitalWrite(13,LOW);
    digitalWrite(12,LOW);
    digitalWrite(11,LOW);
    digitalWrite(10,LOW);
    digitalWrite(9,LOW);
  }
}

void connectServos() {
                      //FR
  Servos[0].attach(40);//coxa
  Servos[1].attach(38);//femur
  Servos[2].attach(36);//tibia

  Servos[3].attach(42);//FL
  Servos[4].attach(44);//0=550
  Servos[5].attach(46);

  Servos[6].attach(34);//BR
  Servos[7].attach(32);//0=2500
  Servos[8].attach(30);

  Servos[9].attach(48);//BL
  Servos[10].attach(50);//0=550
  Servos[11].attach(52);
}



void moveServos(int pulse0, int pulse1, int pulse2, int pulse3, int pulse4, int pulse5, int pulse6, int pulse7, int pulse8, int pulse9, int pulse10, int pulse11) { 

  Servos[0].writeMicroseconds(pulse0);
  Servos[1].writeMicroseconds(pulse1);
  Servos[2].writeMicroseconds(pulse2);

  Servos[3].writeMicroseconds(pulse3);
  Servos[4].writeMicroseconds(pulse4);
  Servos[5].writeMicroseconds(pulse5);

  Servos[6].writeMicroseconds(pulse6);
  Servos[7].writeMicroseconds(pulse7);
  Servos[8].writeMicroseconds(pulse8);

  Servos[9].writeMicroseconds(pulse9);
  Servos[10].writeMicroseconds(pulse10);
  Servos[11].writeMicroseconds(pulse11);
}


void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char spaceMarker = '#';
    
    char rc;
 
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();
//        Serial.print("serial In");
        if (recvInProgress == true) {
            if (rc != endMarker && rc != spaceMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else if (rc == spaceMarker ){
              receivedChars[ndx] = '\0';
              if (spaceCounter==0){
                //Serial.println(receivedChars);
                pulse0 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==1){
                //Serial.println(receivedChars);
                pulse1 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==2){
                //Serial.println(receivedChars);
                pulse2 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==3){
                //Serial.println(receivedChars);
                pulse3 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==4){
                //Serial.println(receivedChars);
                pulse4 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==5){
                //Serial.println(receivedChars);
                pulse5 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==6){
                //Seral.println(receivedChars);
                pulse6 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==7){
                //Serial.println(receivedChars);
                pulse7 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==8){
                //Serial.println(receivedChars);
                pulse8 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==9){
                //Serial.println(receivedChars);
                pulse9 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==10){
                //Serial.println(receivedChars);
                pulse10 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
              else if (spaceCounter==11){
                //Serial.println(receivedChars);
                pulse11 = atoi(receivedChars);
                spaceCounter++;
                ndx=0;
              }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                //Serial.println(receivedChars);
                pulse11 = atoi(receivedChars);
                recvInProgress = false;
                ndx = 0;
                spaceCounter=0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}
