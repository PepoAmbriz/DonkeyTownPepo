#include <Wire.h> 
#define mRF 10 
#define mRR 9
#define mLF 5
#define mLR 6
#define intR 3
#define intL 2
#define gear_ratio 700.0
#define motorL 0
#define motorR 1 
#define Rratio 1.692
#define NOPOWERBANK false
#define DEBUG false

int rev_cnt[] = {0,0}; 
int rev_cnt_tx[] = {0,0};
float f[] = {0,0};
float fs[] = {0,0};
float fe_i[] = {0,0};  
volatile unsigned long last_time, cur_time;
#if NOPOWERBANK
const byte voltPin = A6; 
float volt = 0; 
#endif

float kp = 5.0;
float ki = 4.0;

union recvData{
  byte bdata[8]; 
  float fdata[2]; //left_revs, right_revs
}rData; 

union sendData{
  byte bdata[12];
  float fdata[3]; //left_cnt, rigth_cnt, volt
}sData;  

void setup() {
  pinMode(mRF, OUTPUT);
  pinMode(mRR, OUTPUT);
  pinMode(mLF, OUTPUT); 
  pinMode(mLR, OUTPUT);
  stopMotors(); 
  pinMode(intL, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(intL),callbackL,RISING); 
  pinMode(intR, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(intR),callbackR,RISING); 
  #if DEBUG
  Serial.begin(9600); //Debug only
  #endif
  Wire.begin(8); //i2c address 
  Wire.onReceive(on_receive_callback); 
  Wire.onRequest(on_request_callback);
  
  last_time = micros();
  cur_time  = last_time; 
}

void loop() {
  // put your main code here, to run repeatedly:
  cur_time = micros(); 
  float dt = ((float)cur_time-(float)last_time)/1000000.0;
  compute_f(motorL,dt);
  compute_f(motorR,dt);
  fe_i[motorL] = set_speed(fs[motorL],dt,fe_i[motorL],motorL);
  fe_i[motorR] = set_speed(fs[motorR],dt,fe_i[motorR],motorR);
  #if DEBUG
   Serial.print(f[motorL]);
   Serial.print(",");
   Serial.println(f[motorR]);
  #endif
  #if NOPOWERBANK
  volt = Rratio*5.0*(((float)analogRead(voltPin))/1023.0);  //Discarded if using powerbank
  #endif
  last_time = cur_time;  
  delay(20); 
}

void callbackL(){
  callback_m(motorL); 
}
void callbackR(){
  callback_m(motorR); 
}
void callback_m(byte motor){
  rev_cnt[motor]++;
  rev_cnt_tx[motor]++;
}

int ctrl_pi(float p,float pi){
  return min(255,max(-255,kp*p+ki*pi)); 
}

float set_speed(float vs, float dt, float ei, byte motor){
  if(vs==0.0){
    drive_motor(0,0,motor);
    return 0.0; 
  }
  float this_f = f[motor];
  float e = vs-this_f;  
  float aux_ei = ei+e*dt; 
  int duty_cycle = ctrl_pi(e,aux_ei); 
  if(duty_cycle<0){ 
    drive_motor(0,0,motor);
  } else { 
    drive_motor(duty_cycle,0,motor); 
  }
  return aux_ei; 
}

void drive_motor(int dutyF, int dutyR, byte motor){
  if(motor==motorL){
    analogWrite(mLF,dutyF);
    analogWrite(mLR,dutyR);
  } else if(motor==motorR){
    analogWrite(mRF,dutyF);
    analogWrite(mRR,dutyR);
  } else{
    stopMotors(); 
  }
}

void on_receive_callback(int a){
  if(a==3){
    byte bdata[3];
    for(int i=0; i<3;i++)
      bdata[i] = Wire.read();
    kp = (float)bdata[1]/10.0;
    ki = (float)bdata[2]/10.0;
    #if DEBUG
    Serial.println("Gains");
    Serial.print(kp);
    Serial.print(", ");
    Serial.println(ki);
    #endif
  }
  if(a==9){
    Wire.read();
    for(int i=0;i<8;++i)
      rData.bdata[i] = Wire.read(); 
    fs[motorL] = rData.fdata[motorL]; 
    fs[motorR] = rData.fdata[motorR];
    #if DEBUG
    Serial.println("fs: ");
    Serial.print(fs[motorL]);
    Serial.print(", ");
    Serial.println(fs[motorR]);
    #endif
  }  
  Wire.begin(); 
}

void on_request_callback(){
  sData.fdata[0] = (float)rev_cnt_tx[0]/gear_ratio;
  rev_cnt_tx[0] = 0; 
  sData.fdata[1] = (float)rev_cnt_tx[1]/gear_ratio;
  rev_cnt_tx[1] = 0;
  //sData.fdata[0] = (float)fs[0];  //echo version
  //sData.fdata[1] = (float)fs[1];  //echo version
  #if NOPOWERBANK
  sData.fdata[2] = (float)volt; 
  #else 
  sData.fdata[2] = 89.1928309;  
  #endif
  Wire.write(sData.bdata, 12); 
  Wire.begin(); 
}

void stopMotors(void){
  analogWrite(mLF,0);
  analogWrite(mLR,0);
  analogWrite(mRF,0);
  analogWrite(mRR,0); 
}

void compute_f(byte motor, float dt){
  f[motor] = 60.0*(float)rev_cnt[motor]/(gear_ratio*dt);
  rev_cnt[motor] = 0;
}
