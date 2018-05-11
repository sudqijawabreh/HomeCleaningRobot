#include<Arduino.h>
#define motor_r 0
#define motor_l 1
#define motor_r1 5
#define motor_r2 6
#define motor_l1 9
#define motor_l2 8
#define encoder_r1 2
#define encoder_r2 3
#define encoder_l1 19
#define encoder_l2 18
#define forward
#define backward
#define R 0.02
#define L 0.2
#define tickR 945

double th=PI/2;
int flag=0;
long ticks_r=0;
long ticks_l=0;
long e_p_r1=0;
long e_p_r2=0;
long e_p_l1=0;
long e_p_l2=0;
long t_init=0;
long t_p=0;
double e_p=0;
double eK=0;
long dt=0;
double vel_r=0;
double vel_l=0;
long p_ticks_r=0;
long p_ticks_l=0;
double x_g=0;
double y_g=1;
double x_p=0;
double y_p=0;
double theta_p=0;
int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
void setup(){
    pinMode(encoder_r1,INPUT);
    pinMode(encoder_r2,INPUT);

    pinMode(motor_r1,OUTPUT);
    pinMode(motor_r2,OUTPUT);
    attachInterrupt(digitalPinToInterrupt(encoder_r1),isr_encoder_r1,CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder_r2),isr_encoder_r2,CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder_l1),isr_encoder_l1,CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder_l2),isr_encoder_l2,CHANGE);
    Serial.begin(9600);
    Serial2.begin(9600);
    Serial2.print("a\n");
    delay(2000);

}
void runMotor(int m1,int m2,double speed,int dir){
    if(speed>100)
        speed=100;
    if(speed<-100)
      speed=-100;
   if(dir){
     int value=(speed/100.0)*127;
     /*
     Serial.print("inside\t");
     Serial.print(ticks_r);
    Serial.print("\t");
     Serial.print(value);
     Serial.print("\t");
     Serial.print(eK);
      Serial.print("\t");
      Serial.print(th-ticks_r);
        Serial.print("\t");
     Serial.print("down");
     Serial.print("\n");*/
       digitalWrite(m2,LOW);
       analogWrite(m1,value);
   } 
   else if(dir==0){
    int value=(speed/100.0)*127;
    /* Serial.print("inside\t");
     Serial.print(ticks_r);
    Serial.print("\t");
     Serial.print(value);
      Serial.print("\t");
     Serial.print(eK);
      Serial.print("\t");
      Serial.print(th-ticks_r);
     Serial.print("\t");
     Serial.print("up");
     Serial.print("\n");*/
 
       digitalWrite(m1,LOW);
       analogWrite(m2,value);
       
   }

}
void runSpeed(int m,double speed){
  int dir=0;
  if(speed<0){
    speed=speed*-1;
    dir=1;
  }
  else
    dir=0;
  if(m==motor_r){
    runMotor(motor_r1,motor_r2,speed,dir);
   
  }
  else if(m==motor_l){
    runMotor(motor_l1,motor_l2,speed,dir);
  }
   
}
void calculateSpeed(double v,double w){
  vel_r = (2*v+w*L)/(2*R);
  vel_l = (2*v-w*L)/(2*R);
}
void updatePosition(){
  double m_per_tick=(2*PI*R)/tickR;
  double d_right=(ticks_r-p_ticks_r)*m_per_tick; 
  double d_left=(ticks_l-p_ticks_l)*m_per_tick; 
  double d_center=(d_left+d_right)/2;
  double phi=(d_right-d_left)/L;
  double x_dt=d_center*cos(theta_p);
  double y_dt=d_center*sin(theta_p);
  double theta_dt=phi;

  theta_p+=theta_dt;
  theta_p=atan2(sin(theta_p),cos(theta_p));
  x_p+=x_dt;
  y_p+=y_dt;
  p_ticks_r=ticks_r;
  p_ticks_l=ticks_l;
}

void loop(){
  long p_pid=50;
  float d_pid=0.0009;
  float i_pid=0.00001;
  i_pid=0;
  
  /*if(Serial2.available()){
    long new_p=p_pid;
    float new_d=d_pid;
    float new_i=i_pid;
    char y=Serial2.read();
    if(y!='c'&&y!='r')
    return;
    if(y=='r'){
      x_p=0;
      y_p=0;
      theta_p=0;
      Serial2.println("restart ...........");
      delay(1000);
      return;
    }
     
 
    Serial2.println("choose 1 for p 2 for d 3 for i");
    while(!Serial2.available());
    char choice=Serial2.read();
    switch(choice){
      case '1':
      {
        Serial2.print("enter new P value: ");
        while(!Serial2.available());
         new_p=Serial2.parseInt();
        break;
      }
      case '2':
      {
        Serial2.print("enter new D value: ");
        while(!Serial2.available());
        new_d=Serial2.parseFloat();
        break;
      }
      
      case '3':
      {
        Serial2.print("enter new I value: ");
        while(!Serial2.available());
        new_i=Serial2.parseFloat();
        
      break;
      }
      default:
        return;
      
    }
   
   Serial2.print("new p value= ");Serial2.println(new_p);
   Serial2.print("new d value= ");Serial2.println(new_d,6);
   Serial2.print("new i value= ");Serial2.println(new_i,6);
   Serial2.println("press y to confirm");
   while(!Serial2.available());
   char x=Serial2.read();
   
   if(x=='y'){
     p_pid=new_p;
     d_pid=new_d;
     i_pid=new_i;
   } 
   Serial2.print("new p value= ");Serial2.println(p_pid);
   Serial2.print("new p value= ");Serial2.println(d_pid);
   Serial2.print("new p value= ");Serial2.println(i_pid);
  }*/
  /*runSpeed(motor_l,50);
  runSpeed(motor_r,50); 
  Serial.print(ticks_r);
  Serial.print("\t");
  Serial.print(ticks_l);
  Serial.print("\n");
  return;*/
 /* if(Serial2.available()){
    Serial.print(Serial2.read());
  }*/
  int t=millis();
  double v=1;
  
  dt=t-t_p;
   double u_x=x_g-x_p;
   double u_y=y_g-y_p;
   
  //double e_k=th-theta_p;
  double e_k=atan2(u_y,u_x)-theta_p;
  e_k=atan2(sin(e_k),cos(e_k));
  double eD=(e_k-e_p)/dt;
  eK+=e_k*dt;
  double w=(e_k*p_pid)+(eD*d_pid)+(eK*i_pid);
  if(u_x<0.02&&u_y<0.02){
    v=0;
    w=0;
  }
    
  calculateSpeed(v,w);
  runSpeed(motor_r,vel_r);
  runSpeed(motor_l,vel_l);  
updatePosition(); 
Serial.print(freeRam());
Serial.print("\t");
Serial.print((theta_p*180)/PI);
Serial.print("\t");
Serial.print(e_k*p_pid);
Serial.print("\t"); 
Serial.print(eD*d_pid);
Serial.print("\t");
Serial.print(eK*i_pid);
Serial.print("\t");
Serial.print(p_ticks_r);
Serial.print("\t");
Serial.print(p_ticks_l);
Serial.print("\t");
Serial.print(x_p,6);
Serial.print("\t");
Serial.print(y_p,6);
Serial.print("\r\n");



Serial2.print((theta_p*180)/PI);
Serial2.print("\t");
Serial2.print(e_k);
Serial2.print("\t");
Serial2.print(x_p,4);
Serial2.print("\t");
Serial2.print(y_p,4);
Serial2.print("\t");
Serial2.print(x_p);
Serial2.print("\t");
Serial2.print(y_p);
Serial2.print("\t");
Serial2.print(theta_p);
Serial2.print("\t");
Serial2.print(x_g);
Serial2.print("\r\n");
 /*Serial.print(x_p); 
 Serial.print("\t");
 Serial.print(y_p); 
 Serial.print("\t");
 Serial.print(theta_p);
   Serial.print("\t");
   Serial.print(ticks_r);
      Serial.print("\t");
   Serial.print(ticks_l);
 Serial.print("\n");*/
 /*Serial.print(ticks_r) ;
 Serial.print("\t");
 Serial.print(ticks_l) ;
 Serial.print("\n");*/
  
 
    //Serial.print(e_k*0.001*127/100.0);
   // Serial.print("\n");
    
//    runSpeed(0,e_k*0.1+eD*0.0016+eK*0.00002);
  
    t_p=t;
    e_p=e_k;
}
void readEncoder_r(){
   int e1=digitalRead(encoder_r1);
  int e2=digitalRead(encoder_r2);
  
  if(e1==0&&e2==0){
     if(e_p_r1==0&&e_p_r2==1)
      ticks_r++;
     else if(e_p_r1==1&&e_p_r2==0)
      ticks_r--;
  }
  else if(e1==0&&e2==1){
     if(e_p_r1==1&&e_p_r2==1)
      ticks_r++;
     else if(e_p_r1==0&&e_p_r2==0)
      ticks_r--;
  }
  else if(e1==1&&e2==0){
     if(e_p_r1==0&&e_p_r2==0)
      ticks_r++;
     else if(e_p_r1==1&&e_p_r2==1)
      ticks_r--;
  }
    else if(e1==1&&e2==1){
     if(e_p_r1==1&&e_p_r2==0)
      ticks_r++;
     else if(e_p_r1==0&&e_p_r2==1)
      ticks_r--;
  }
  e_p_r1=e1;
  e_p_r2=e2;
}
void readEncoder_l(){
   int e1=digitalRead(encoder_l1);
  int e2=digitalRead(encoder_l2);
  
  if(e1==0&&e2==0){
     if(e_p_l1==0&&e_p_l2==1)
      ticks_l++;
     else if(e_p_l1==1&&e_p_l2==0)
      ticks_l--;
  }
  else if(e1==0&&e2==1){
     if(e_p_l1==1&&e_p_l2==1)
      ticks_l++;
     else if(e_p_l1==0&&e_p_l2==0)
      ticks_l--;
  }
  else if(e1==1&&e2==0){
     if(e_p_l1==0&&e_p_l2==0)
      ticks_l++;
     else if(e_p_l1==1&&e_p_l2==1)
      ticks_l--;
  }
    else if(e1==1&&e2==1){
     if(e_p_l1==1&&e_p_l2==0)
      ticks_l++;
     else if(e_p_l1==0&&e_p_l2==1)
      ticks_l--;
  }
  e_p_l1=e1;
  e_p_l2=e2;

}
void isr_encoder_r1(){
  readEncoder_r();

}
void isr_encoder_r2(){
   readEncoder_r();
}
void isr_encoder_l1(){
    readEncoder_l();
}
void isr_encoder_l2(){
    readEncoder_l();
}
