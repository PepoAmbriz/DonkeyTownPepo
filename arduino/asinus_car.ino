#include <Wire.h> 
#define mRF 10 
#define mRR 9
#define mLF 5
#define mLR 6
#define intR 3
#define intL 2
#define ratio 75
#define motorL 0
#define motorR 1 
#define Rratio 1.692
#define NOPOWERBANK false
#define DEBUG true

float f[] = {0,0}; 
float fs[] = {0,0};
float fe_i[] = {0,0}; 
volatile unsigned long time_m []= {0,0}; 
volatile unsigned long last_time, cur_time;
#if NOPOWERBANK
const byte voltPin = A6; 
float volt = 0; 
#endif
int idlcnt[] = {0,0}; 

union recvData{
  byte bdata[8]; 
  float fdata[2]; 
}rData; 

union sendData{
  byte bdata[12];
  float fdata[3]; 
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
  time_m[0] = last_time;
  time_m[1] = last_time;
}

void loop() {
  // put your main code here, to run repeatedly:
  cur_time = micros(); 
  float dt = ((float)cur_time-(float)last_time)/1000000;
  fe_i[motorL] = set_speed(fs[motorL],dt,fe_i[motorL],motorL);
  fe_i[motorR] = set_speed(fs[motorR],dt,fe_i[motorR],motorR);
  #if DEBUG
  Serial.print(f[motorL]); //Debug only
  Serial.print(","); //Debug only
  Serial.println(f[motorR]); //Debug only
  #endif
  #if NOPOWERBANK
  volt = Rratio*5.0*(((float)analogRead(voltPin))/1023.0);  //Discarded if using powerbank
  #endif
  if(idlcnt[motorL]++>(fs[motorL]<5.0)) f[motorL] = 0; 
  if(idlcnt[motorR]++>(fs[motorR]<5.0)) f[motorR] = 0;  
  last_time = cur_time;  
  delay(10); 
}

void callbackL(){
  callback_m(motorL); 
}
void callbackR(){
  callback_m(motorR); 
}
void callback_m(byte motor){
  volatile unsigned long auxtime; 
  auxtime = micros(); 
  f[motor] = (6000000/ratio)/((float)auxtime-(float)time_m[motor]); 
  idlcnt[motor] = 0; 
  time_m[motor] = auxtime; 
}
int ctrl_pi(float p,float pi){
  return min(255,max(-255,5.0*p+4.0*pi)); 
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

void sendData(float volt){
  #if NOPOWERBANK
  Serial.print(volt,4);
  #else
  Serial.print(5.0,4);
  #endif
  Serial.println(" V"); 
  
  Serial.print(f[motorL]);
  Serial.println(" fL");
  Serial.print(f[motorR]);
  Serial.println(" fR");
}

void on_receive_callback(int a){
  #if DEBUG
  //Serial.println(a); //Debug only
  #endif
  if(a==9){
    Wire.read();
    for(int i=0;i<8;++i)
      rData.bdata[i] = Wire.read(); 
    fs[motorL] = rData.fdata[motorL]; 
    fs[motorR] = rData.fdata[motorR];
  }  
  Wire.begin(); 
}

void on_request_callback(){
  #if DEBUG
  //Serial.println("Sending data"); 
  #endif
  sData.fdata[0] = (float)f[0]; 
  //sData.fdata[0] = (float)fs[0];  //echo version
  sData.fdata[1] = (float)f[1];
  //sData.fdata[1] = (float)fs[1];  //echo version
  #if NOPOWERBANK
  sData.fdata[2] = (float)volt; 
  #else
  //sData.fdata[2] = (float)5.0;  
  sData.fdata[2] = 89.1928309;  
  #endif
  Wire.write(sData.bdata, 12); //not including voltage measure while using powerbank 
  Wire.begin(); 
}

void stopMotors(void){
  analogWrite(mLF,0);
  analogWrite(mLR,0);
  analogWrite(mRF,0);
  analogWrite(mRR,0); 
}