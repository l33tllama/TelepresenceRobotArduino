#include <Wire.h>
#include <NewPing.h>
#define SLAVE_ADDRESS 0x04
#define E1 5
#define M1 4
#define E2 6
#define M2 7
int number = 0;
int state = 0;

enum CONTROL_STATE {
  READY, BUSY 
};
/*
 * 0bxx01 - 1 - RIGHT FWD
 * 0bxx10 - 2 - RIGHT BACK
 * 0bxx00 - 3 - RIGHT STOP
 * 0b01xx - 4 - LEFT FWD
 * 0b10xx - 5 - LEFT BACK
 * 0b00xx - 6 - LEFT STOP
 */

#define B_RIGHT_FWD   0b01
#define B_RIGHT_BACK  0b10
#define B_LEFT_FWD  0b1000
#define B_LEFT_BACK 0b0100

enum MOTOR_CODES {
  LEFT_FWD,
  LEFT_BACK,
  LEFT_STOP,
  RIGHT_FWD,
  RIGHT_BACK,
  RIGHT_STOP
};

NewPing sonar(12, 11, 200);

short left_motor_state = 2;
short right_motor_state = 5;
short control_state;
int last_left_state = 2;
int last_right_state = 4;
unsigned int m1speed = 255;
unsigned int m2speed = 255;

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600); // start serial for output
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  control_state = READY;
  last_left_state = -1;
  last_right_state = -1;
  randomSeed(analogRead(0));
  
  Serial.println("Ready!");

}

void loop() {
  powerMotor();
  checkLastCommand();

}
// callback for received data
void receiveData(int byteCount){
  
  while(Wire.available()) {
   
    number = Wire.read();
    Serial.print("data received: ");
    Serial.println(number);

    // ready to take motor controls
    if(number & B_LEFT_FWD == B_LEFT_FWD){
      if(control_state == READY){
        left_motor_state = LEFT_FWD;  
      } else {
        last_left_state = LEFT_FWD;
      }
    }
    if(number & B_LEFT_BACK == B_LEFT_BACK){
      if(control_state == READY){
        left_motor_state = LEFT_BACK;  
      } else {
        last_left_state = LEFT_BACK;
      }
    }
    if((((number >> 2) & 3) | 0) == 0){
      if(control_state == READY){
        left_motor_state = LEFT_STOP;  
      } else {
        last_left_state = LEFT_STOP;
      }
    }
    if(number & B_RIGHT_FWD == B_RIGHT_FWD){
      if(control_state == READY){
        right_motor_state = RIGHT_FWD;  
      } else {
        last_right_state = RIGHT_FWD;
      }
    }
    if(number & B_RIGHT_BACK == B_RIGHT_BACK){
      if(control_state == READY){
        right_motor_state = RIGHT_BACK;  
      } else {
        last_right_state = RIGHT_BACK;
      }
      
    }
    if((number & 3 ) | 0 == 0){
      if(control_state == READY){
        right_motor_state = RIGHT_STOP;  
      } else {
        last_right_state = RIGHT_STOP;
      }
    }
        
    if (number == 1){
      
      if (state == 0){
        digitalWrite(13, HIGH); // set the LED on
        state = 1;
      }
      else{
        digitalWrite(13, LOW); // set the LED off
        state = 0;
      }
    }
  }
}

// Power motor based on read states from I2C
void powerMotor(){
  if(control_state == READY){
    switch(left_motor_state){
      case LEFT_FWD:
        M1FWD();
        break;
      case LEFT_BACK:
        M1BACK();
        break;
      case LEFT_STOP:
        M1STOP();
        break;
    }
    switch(right_motor_state){
      case RIGHT_FWD:
        M2FWD();
        break;
      case RIGHT_BACK:
        M2BACK();
        break;
      case RIGHT_STOP:
        M2STOP();
        break;
    }
  }
}

void checkLastCommand(){
  if(last_left_state > 0){
    left_motor_state = last_left_state;
    last_left_state = -1;
  }
  if(last_right_state > 0){
    right_motor_state = last_right_state;
    last_right_state = -1;
  }
}

inline void setM1Speed(int spd){
  m1speed = (int)((float)spd / 100.0f) * 255;
  
}

inline void setM2Speed(int spd){
  m2speed = (int)((float)spd / 100.0f) * 255;
}

inline void M1FWD(){
  digitalWrite(M1, LOW);
  analogWrite(E1, m1speed);
}
inline void M1BACK(){
  digitalWrite(M1, HIGH);
  analogWrite(E1, m1speed);
}
inline void M1STOP(){
  digitalWrite(M1, LOW);
  digitalWrite(E1, LOW);
}
inline void M2FWD(){
  digitalWrite(M2, LOW);
  analogWrite(E2, m2speed);
}
inline void M2BACK(){
  digitalWrite(M2, HIGH);
  analogWrite(E2, m2speed);
}
inline void M2STOP(){
  digitalWrite(M2, LOW);
  digitalWrite(E2, LOW);
}

void obstacleAvoid(){
  // TODO
  unsigned int cm = sonar.ping_cm();
  if(cm < 10){
    setM1Speed(25);
    setM1Speed(25);
    delay(100);
    M1STOP();
    M2STOP();
    delay(100);
    setM1Speed(100);
    setM2Speed(100);
    M1BACK();
    M2BACK();
    delay(1500);
    M1STOP();
    M2STOP();
    delay(100);
    int fwdorback = random(10);
    if(fwdorback < 5){
      M1FWD();
      M2BACK();  
    } else {
      M1BACK();
      M2FWD();
    }
    delay(500);
    
  }
  
}

// callback for sending data
void sendData(){
  Wire.write(number);
}
