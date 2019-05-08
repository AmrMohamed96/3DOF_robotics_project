#include <math.h>
#include <Servo.h>

#define vx_pin A0 //Vx Channel Pin
#define vy_pin A1 //Vy Channel Pin
#define sw_pin 7 //SW1 Channel Pin
#define vz_pin A3 //Vz Channel (Vx of 2nd Joystick)

#define servo1_pin 9
#define servo2_pin 10
#define servo3_pin 11
#define servo4_pin 6

Servo servo1, servo2, servo3, servo4;

int vx_reading = 0; //Joystick X Axis Reading
int vy_reading = 0; //Joystick Y Axis Reading
int vz_reading = 0; //Joystick Z Axis Reading
int sw_state = 0; //Joystick Button

float x_mul, y_mul, z_mul; //Increment Multiplier
float x_pos, y_pos, z_pos; //Absolute X, Y variables

int rate = 1; //Incrementation Rate
int inc_time = 100; //Minimum Incrementation Interval

unsigned long previous_tx = 0; //Previous time for last time x_pos was incremented
unsigned long previous_ty = 0; //Previous time for last time y_pos was incremented
unsigned long previous_tz = 0; //Previous time for last time z_pos was incremented

/* Control Variables for Servos */
float theta1;
float theta2;
float theta3;
float theta4;

/* Robot Parameters */
float r1, r2, r3;
float a1=24.94, a2=82, a3=168.5;
float phi1, phi2, phi3;
float x_max=100, y_max=100, z_max=100;

/* Float Point Mapping Function */
float mapfloat(long x, long in_min, long in_max, long out_min, long out_max){
 return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

/* Function for incrementing x_pos */
void inc_x(){
  unsigned long now = millis();
  if ( now - previous_tx >= inc_time ){
      x_pos = x_pos + (x_mul * rate);
      previous_tx= millis();
  }
}

/* Function for incrementing y_pos */
void inc_y(){
  unsigned long now = millis();
  if ( now - previous_ty >= inc_time ){
      y_pos = y_pos + (y_mul * rate);
      previous_ty = millis();
  }
}

/* Function for incrementing z_pos */
void inc_z(){
  unsigned long now = millis();
  if ( now - previous_tz >= inc_time ){
      z_pos = z_pos + (z_mul * rate);
      previous_tz = millis();
  }
}

void setup() {
  /* Pin Declaration */
  pinMode(vx_pin, INPUT);
  pinMode(vy_pin, INPUT);
  pinMode(vz_pin, INPUT);
  pinMode(sw_pin, INPUT);
  
  digitalWrite(sw_pin, HIGH); //Pull Up Resistor for Switch - HW Requirement

  servo1.attach(servo1_pin);
  servo2.attach(servo2_pin);
  servo3.attach(servo3_pin);
  servo4.attach(servo4_pin);
  
  Serial.begin(9600);
}

void loop() {
  /* Reading HW Values */
  vx_reading = analogRead(vx_pin);
  vy_reading = analogRead(vy_pin);
  sw_state = digitalRead(sw_pin);

  vz_reading = analogRead(vz_pin);

  /* Calculating Multiplier */
  x_mul = mapfloat(vx_reading, 0, 1023, -1, 1);
  y_mul = mapfloat(vy_reading, 0, 1023, -1, 1);
  z_mul = mapfloat(vz_reading, 0, 1023, -1, 1);


  /* Eliminating Noise */
  if( x_mul < 0 && x_mul > -0.02 ) {
    x_mul = 0;
  }
  if( y_mul < 0 && y_mul > -0.02 ) {
    y_mul = 0;
  }
  if( z_mul < 0 && z_mul > -0.02 ) {
    z_mul = 0;
  }
  
  /* Incrementing Global X_Pos and Y_Pos */
  inc_x();
  inc_y();
  inc_z();

  /* Saturation for Position Values */
  if ( x_pos >= x_max ) { 
    x_pos = x_max; 
  } else if ( x_pos <= -x_max ){
    x_pos = -x_max;
  }

  if ( y_pos >= y_max ) { 
    y_pos = y_max; 
  } else if ( y_pos <= -y_max ){
    y_pos = -y_max;
  }

  if ( z_pos >= z_max ) { 
    z_pos = z_max; 
  } else if ( z_pos <= -z_max ){
    z_pos = -z_max;
  }

  /* Printing to send values to Vrep */
  Serial.print(x_pos);
  Serial.print("\t");
  Serial.print(y_pos);
  Serial.print("\t");
  Serial.print(sw_state);
  Serial.println("\t");

  /* Inverse Kinematic Equations */
  theta1 = atan( y_pos / x_pos );

  r1 = sqrt( ( x_pos * x_pos ) + ( y_pos * y_pos ));
  r2 = z_pos - a1;
  phi2 = atan( r2 / r1 );
  phi1 = acos( ( (a3 * a3) - (a2 * a2) - (r3 * r3)) / ( -2 * a2 * r3 ) );
  theta2 = phi2 - phi1;

  r3 = sqrt( (r1 * r1) + (r2 * r2));
  phi3 = acos( ((r3 * r3) - (a2 * a2) - (a3 * a3)) / (-2 * a2 * a3 ));
  theta3 = 180 - phi3;

  /* Servo Actuation */
  servo1.write(theta1);
  servo2.write(theta2);
  servo3.write(theta3);
  servo4.write(theta4);
}
