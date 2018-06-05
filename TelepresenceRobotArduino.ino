#include <Wire.h>

#define SLAVE_ADDRESS 0x04
int number = 0;
int state = 0;

enum CONTROL_STATE {
  READY, BUSY 
};
enum MOTOR_CODES {
  LEFT_FWD,
  LEFT_BACK,
  LEFT_STOP,
  RIGHT_FWD,
  RIGHT_BACK,
  RIGHT_STOP
};

short left_motor_state;
short right_motor_state;
short control_state;
int last_left_state;
int last_right_state;

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
    if(control_state == READY){
      if (number > 0 && number < 3){
        left_motor_state = number;
      }
      if(number > 3 && number < 7){
        right_motor_state = number;
      }
    // save last motor state when not busy        
    } else if (control_state == BUSY){
      if(number > 0 && number < 3) {
        last_left_state = number;  
      } else if (number >= 3 && number < 7){
        last_right_state = number;
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
        break;
      case LEFT_BACK:
        break;
      case LEFT_STOP:
        break;
    }
    switch(right_motor_state){
      case RIGHT_FWD:
        break;
      case RIGHT_BACK:
        break;
      case RIGHT_STOP:
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

void obstacleAvoid(){
  // TODO
}

// callback for sending data
void sendData(){
  Wire.write(number);
}
