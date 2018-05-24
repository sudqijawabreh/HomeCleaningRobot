#include <Arduino.h>
const int trigPin = 50;
const int echoPin = 52;
#define fan 44
#define motor_r 0
#define motor_l 1
#define motor_r1 6
#define motor_r2 5
#define motor_l1 8
#define motor_l2 9
#define encoder_r1 3
#define encoder_r2 2 
#define encoder_l1 19
#define encoder_l2 18 
#define forward
#define backward
#define R 0.022
#define L 0.182
#define tickR 945

double u_x = 500;
double u_y = 500;
double th = PI / 2;
int flag = 0;
long ticks_r = 0;
long ticks_l = 0;
long e_p_r1 = 0;
long e_p_r2 = 0;
long e_p_l1 = 0;
long e_p_l2 = 0;
long t_init = 0;
long t_p = 0;
double e_p = 0;
double eK = 0;
long dt = 0;
double vel_r = 0;
double vel_l = 0;
long p_ticks_r = 0;
long p_ticks_l = 0;
double x_g = 0;
double y_g = 0;
double x_p = 0;
double y_p = 0;
float th_g = 0;
double v = 0;
double theta_p = 0;
int freeRam()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}
void setup()
{
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);
  pinMode(encoder_r1, INPUT);
  pinMode(encoder_r2, INPUT);
  pinMode(fan,OUTPUT);
  pinMode(motor_r1, OUTPUT);
  pinMode(motor_r2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_r1), isr_encoder_r1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_r2), isr_encoder_r2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_l1), isr_encoder_l1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_l2), isr_encoder_l2, CHANGE);
  Serial.begin(9600);
  Serial2.begin(9600);
  Serial2.print("a\n");
  /*turn(-90);
  delay(2000);
  moveForward(0.5);
  delay(2000);
  turn(0);
  delay(2000);
  moveForward(0.5);
  delay(2000);
  turn(90);
  moveForward(0.5);
  delay(2000);
  return;*/
  analogWrite(fan,100);
  for(int i=0;i<2;i++){
  turn(-90);
  Serial.println("after turn -90");
  moveToObs();
  //moveForward(1);
  delay(200);
  Serial.println("after forward");
  delay(200);
  turn(0);
  Serial.println("after turn 0");
  delay(200);
  //moveToObs();
  moveForward(0.2);
  Serial.println("after move forward 20");
  delay(200);
  turn(90);
  Serial.println("after turn 90");
  delay(200);
  moveToObs();
  Serial.println("after forward");
  delay(200);
  turn(0);
  Serial.println("after turn 0");
  delay(200);
  //moveToObs();
  moveForward(0.2);
  delay(200);
  
  }
  //moveP(0,0);

  /*
  delay(2000);
  v=0;
  x_g=0;
  y_g=0.6;
  pid_theta();
  Serial.println("after 90");
  delay(2000);
  v=1;
  pid();
  Serial.println("after forward");
  delay(1000);
  v=0;
  x_g=0.6;
  pid_theta();
  Serial.println("after -90");
  v=1;
  pid();
  delay(5000);
  Serial.println("after forward");*/
}
void moveP(float x,float y){
  x_g =x;
  y_g =y;
  v = -1;
  pid();
}
void moveForward(float distance)
{
  x_g =x_p+ cos(th_g) * distance;
  y_g =y_p+ sin(th_g) * distance;
  v = -1;
  pid();
}
void turn(float angle)
{
  th_g = angle * PI / 180.0;
  v = 0;
  pid_theta();
}
void runMotor(int m1, int m2, double speed, int dir)
{
  if (speed > 100)
    speed = 100;
  if (speed < -100)
    speed = -100;
  if (dir)
  {
    int value = (abs(speed) / 100.0) * 127;
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
    digitalWrite(m2, LOW);
    analogWrite(m1, value);
  }
  else if (dir == 0)
  {
    int value = (abs(speed) / 100.0) * 127;
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

    digitalWrite(m1, LOW);
    analogWrite(m2, value);
  }
}
void runSpeed(int m, double speed)
{
  int dir = 0;
  if (speed < 0)
  {
    speed = speed * -1;
    dir = 1;
  }
  else if(speed>0)
    dir = 0;
  if (m == motor_r)
  {
    runMotor(motor_r1, motor_r2, speed, dir);
  }
  else if (m == motor_l)
  {
    runMotor(motor_l1, motor_l2, speed, dir);
  }
}
void calculateSpeed(double v, double w)
{
  vel_r = (2 * v + w * L) / (2 * R);
  vel_l = (2 * v - w * L) / (2 * R);
}
void updatePosition()
{
  double m_per_tick = (2 * PI * R) / tickR;
  double d_right = (ticks_r - p_ticks_r) * m_per_tick;
  double d_left = (ticks_l - p_ticks_l) * m_per_tick;
  double d_center = (d_left + d_right) / 2;
  double phi = (d_right - d_left) / L;
  double x_dt = d_center * cos(theta_p);
  double y_dt = d_center * sin(theta_p);
  double theta_dt = phi;

  theta_p += theta_dt;
  theta_p = atan2(sin(theta_p), cos(theta_p));
  x_p += x_dt;
  y_p += y_dt;
  p_ticks_r = ticks_r;
  p_ticks_l = ticks_l;
}

void loop()
{
}
void menu()
{
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
}
void pid_theta()
{
  u_x = 500;
  u_y = 500;
  long count_locked = 0;
  float theta_previous = theta_p;
  while (true)
  {
    long p_pid = 17;
    float d_pid = 0.001;
    float i_pid = 0.00001;
    //i_pid = 0;
    int t = millis();
    dt = t - t_p;
    u_x = x_g - x_p;
    u_y = y_g - y_p;
    Serial2.print("u: ");
    Serial2.print(u_x);
    Serial2.print("\t");
    Serial2.print(u_y);
    Serial2.print("\r\n");
    //double e_k = th - theta_p;
    double e_k = (th_g - theta_p);
    e_k = atan2(sin(e_k), cos(e_k));
    Serial.print(e_k * 180 / PI);
    double eD = (e_k - e_p) / dt;
    eK += e_k * dt;
    double w = (e_k * p_pid) + (eD * d_pid) + (eK * i_pid);

    if (abs(e_k) < 0.001 || count_locked > 20)
    {
      Serial.println("inside the if");
      v = 0;
      w = 0;
      calculateSpeed(v, w);
      runSpeed(motor_r, vel_r);
      runSpeed(motor_l, vel_l);
      updatePosition();
      printStatus();
      t_p = t;
      e_p = e_k;
      return;
    }
    calculateSpeed(v, w);
    if(w>0)
      vel_r=0;
    else if(w<0)
      vel_l=0;
    runSpeed(motor_r, vel_r);
    runSpeed(motor_l, vel_l);
    updatePosition();
    if (abs(theta_p - theta_previous < 0.00001))
      count_locked++;
    theta_previous = theta_p;
    printStatus();
    t_p = t;
    e_p = e_k;
  }
}
void moveToObs(){
  v=-1;
  while (true)
  {
    long p_pid =20 ;
    float d_pid = 0.0006;
    float i_pid = 0.00001;
    i_pid = 0;
    int t = millis();
    dt = t - t_p;
    //u_x = x_g - x_p;
    //u_y = y_g - y_p;
    Serial2.print("u: ");
    Serial2.print(u_x);
    Serial2.print("\t");
    Serial2.print(u_y);
    Serial2.print("\r\n");
    long distance=getDistance();
    Serial.print("d ");
    Serial2.print("d ");
    Serial.print(distance);
    Serial2.print(distance);
    double e_k = th_g - theta_p;
    //double e_k = atan2(u_y, u_x) - theta_p;
    e_k = atan2(sin(e_k), cos(e_k));
    double eD = (e_k - e_p) / dt;
    eK += e_k * dt;
    double w = (e_k * p_pid) + (eD * d_pid) + (eK * i_pid);

    if (distance<15&&distance>0)
    {
      Serial.println("inside the if");
      v = 0;
      w = 0;
      calculateSpeed(v, w);
      runSpeed(motor_r, vel_r);
      runSpeed(motor_l, vel_l);
      updatePosition();
      printStatus();
      t_p = t;
      e_p = e_k;
      return;
    }
    calculateSpeed(v, w);
    runSpeed(motor_r, vel_r);
    runSpeed(motor_l, vel_l);
    updatePosition();
    printStatus();
    t_p = t;
    e_p = e_k;
  }

}
void pid()
{
  while (true)
  {
    long p_pid = 20;
    float d_pid = 0.0006;
    float i_pid = 0.00001;
    i_pid = 0;
    long distance=getDistance();
    int t = millis();
    dt = t - t_p;
    u_x = x_g - x_p;
    u_y = y_g - y_p;
    Serial2.print("u: ");
    Serial2.print(u_x);
    Serial2.print("\t");
    Serial2.print(u_y);
    Serial2.print("\r\n");
    //double e_k = th - theta_p;
    double e_k = atan2(u_y, u_x) - theta_p;
    e_k = atan2(sin(e_k), cos(e_k));
    double eD = (e_k - e_p) / dt;
    eK += e_k * dt;
    double w = (e_k * p_pid) + (eD * d_pid) + (eK * i_pid);

    if (abs(u_x) < 0.01 && abs(u_y) < 0.01||(distance<15&&distance>0))
    {
      Serial.println("inside the if");
      v = 0;
      w = 0;
      calculateSpeed(v, w);
      runSpeed(motor_r, vel_r);
      runSpeed(motor_l, vel_l);
      updatePosition();
      printStatus();
      t_p = t;
      e_p = e_k;
      return;
    }
    calculateSpeed(v, w);
    runSpeed(motor_r, vel_r);
    runSpeed(motor_l, vel_l);
    updatePosition();
    printStatus();
    t_p = t;
    e_p = e_k;
  }
}
void printStatus()
{

  /*
  Serial2.print(theta_p*180/PI);
  Serial2.print("\t");
  Serial2.print(x_p);
  Serial2.print("\t");
  Serial2.print(y_p);
  Serial2.print("\t");
  Serial2.print("\r\n");*/
  Serial2.print("\t");
  Serial2.print((theta_p * 180) / PI);
  Serial2.print("\t");
  Serial2.print(u_x, 6);
  Serial2.print("\t");
  Serial2.print(u_y, 6);
  Serial2.print("\t");
  Serial2.print(x_p, 6);
  Serial2.print("\t");
  Serial2.print(y_p, 6);
  Serial2.print("\r\n");

  Serial.print("\t");
  Serial.print((theta_p * 180) / PI);
  Serial.print("\t");
  Serial.print(u_x, 6);
  Serial.print("\t");
  Serial.print(u_y, 6);
  Serial.print("\t");
  Serial.print(x_p, 6);
  Serial.print("\t");
  Serial.print(y_p, 6);
  Serial.print("\r\n");
}
void readEncoder_r()
{
  int e1 = digitalRead(encoder_r1);
  int e2 = digitalRead(encoder_r2);

  if (e1 == 0 && e2 == 0)
  {
    if (e_p_r1 == 0 && e_p_r2 == 1)
      ticks_r++;
    else if (e_p_r1 == 1 && e_p_r2 == 0)
      ticks_r--;
  }
  else if (e1 == 0 && e2 == 1)
  {
    if (e_p_r1 == 1 && e_p_r2 == 1)
      ticks_r++;
    else if (e_p_r1 == 0 && e_p_r2 == 0)
      ticks_r--;
  }
  else if (e1 == 1 && e2 == 0)
  {
    if (e_p_r1 == 0 && e_p_r2 == 0)
      ticks_r++;
    else if (e_p_r1 == 1 && e_p_r2 == 1)
      ticks_r--;
  }
  else if (e1 == 1 && e2 == 1)
  {
    if (e_p_r1 == 1 && e_p_r2 == 0)
      ticks_r++;
    else if (e_p_r1 == 0 && e_p_r2 == 1)
      ticks_r--;
  }
  e_p_r1 = e1;
  e_p_r2 = e2;
}
void readEncoder_l()
{
  int e1 = digitalRead(encoder_l1);
  int e2 = digitalRead(encoder_l2);

  if (e1 == 0 && e2 == 0)
  {
    if (e_p_l1 == 0 && e_p_l2 == 1)
      ticks_l++;
    else if (e_p_l1 == 1 && e_p_l2 == 0)
      ticks_l--;
  }
  else if (e1 == 0 && e2 == 1)
  {
    if (e_p_l1 == 1 && e_p_l2 == 1)
      ticks_l++;
    else if (e_p_l1 == 0 && e_p_l2 == 0)
      ticks_l--;
  }
  else if (e1 == 1 && e2 == 0)
  {
    if (e_p_l1 == 0 && e_p_l2 == 0)
      ticks_l++;
    else if (e_p_l1 == 1 && e_p_l2 == 1)
      ticks_l--;
  }
  else if (e1 == 1 && e2 == 1)
  {
    if (e_p_l1 == 1 && e_p_l2 == 0)
      ticks_l++;
    else if (e_p_l1 == 0 && e_p_l2 == 1)
      ticks_l--;
  }
  e_p_l1 = e1;
  e_p_l2 = e2;
}
void isr_encoder_r1()
{
  readEncoder_r();
}
void isr_encoder_r2()
{
  readEncoder_r();
}
void isr_encoder_l1()
{
  readEncoder_l();
}
void isr_encoder_l2()
{
  readEncoder_l();
}
long getDistance()
{
  long duration;
  long distance;
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH,17647);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);
  Serial2.print("Distance: ");
  Serial2.println(distance);
  return distance;
}