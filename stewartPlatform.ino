// Stewart Platform

// Libraries:
#include <Servo.h> //servo
#include <Wire.h>    //I2C/LCD
#include <LiquidCrystal_I2C.h> //LCD

//constants for computation of positions of connection points
#define pi  3.14159
#define deg2rad 180/pi
#define deg30 pi/6
//MIN/MAX - servo
#define MAX 230
#define MIN 5
//Positions of servos mounted in opposite direction
#define INV1 1
#define INV2 3
#define INV3 5

//Defined screen resolution (Max: 1024)
#define Xresolution 720 //128
#define Yresolution 1024 //64
//Touch screen pin connections
const int X1 = A10;
const int X2 = A8;
const int Y1 = A11;
const int Y2 = A9;

//LA - the effective length of a servo arm, LBP - the length of the arm connecting the base and the platform
//PPD a distance from the centre of the platform to attachment points (arm attachment point)
//BAD a distance from the centre of the base to the centre of servo rotation points (servo axis)
//z_platf - a height of the platform with respect to the base, 0 is the height of servo arms
//theta_p - an angle between two servo axis points, theta_p - an angle between platform attachment points
const float LA = 13, LBP = 110, PPD = 50, BAD = 60, z_platf = 105.75;
Servo servo [6];
//Zero position of servos, in these positions their arms are set in such a way that they create a right angle with connecting rod.
static int zero[6] = {80+19, 96-19, 90+19, 87-19, 95+19, 80-19};
//In this array is stored requested position for the platform - x,y,z,rot(x),rot(y),rot(z)
static float rpp[6] = {0, 0, 0, 0, 0, 0};
/*Actual degree of rotation of all servo arms, they start at 0 - horizontal, used to reduce
  complexity of calculating new degree of rotation*/
static float theta_a[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//Array of current servo positions
static int servo_pos[6];
//rotation of servo arms with respect to axis x
const float beta[] = {pi / 2, -pi / 2, -pi / 6, 5 * pi / 6, -5 * pi / 6, pi / 6},
                     //maximum servo positions, 90 is horizontal position
                     servo_min = 10, servo_max = 170,
                     //theta_b2s-angle between two servo axis points, theta_p - an angle between platform attachment points
                     //theta_angle-helper variable
                     theta_b2s = radians(43),
                     theta_angle = (pi / 3 - theta_b2s) / 2, theta_p = radians(10.5),
                     //b[][]= (x y z values) the points of rotation of the servo arms (servo axis) (relative to the base coordinate system)
                     //p[][]= (x y z values) the points/the joints connecting the legs with the platform (relative to the platform coordinate system)
b[3][6] = {
  {
    -BAD * cos(deg30 - theta_angle), -BAD * cos(deg30 - theta_angle),
    BAD * sin(theta_angle), BAD * cos(deg30 + theta_angle),
    BAD * cos(deg30 + theta_angle), BAD * sin(theta_angle)
  },
  {
    -BAD * sin(deg30 - theta_angle), BAD * sin(deg30 - theta_angle),
    BAD * cos(theta_angle), BAD * sin(deg30 + theta_angle),
    -BAD * sin(deg30 + theta_angle), -BAD * cos(theta_angle)
  },
  {
    0, 0, 0, 0, 0, 0
  }
},
p[3][6] = {
  {
    -PPD * sin(deg30 + theta_p / 2), -PPD * sin(deg30 + theta_p / 2),
    -PPD * sin(deg30 - theta_p / 2), PPD * cos(theta_p / 2),
    PPD * cos(theta_p / 2), -PPD * sin(deg30 - theta_p / 2),
  }, {
    -PPD * cos(deg30 + theta_p / 2), PPD * cos(deg30 + theta_p / 2),
    PPD * cos(deg30 - theta_p / 2), PPD * sin(theta_p / 2),
    -PPD * sin(theta_p / 2), -PPD * cos(deg30 - theta_p / 2),
  }, {
    0, 0, 0, 0, 0, 0
  }
};

//maximum servo positions, 0 is horizontal position
//      servo_min= 10,servo_max= 165,

//LCD function
LiquidCrystal_I2C lcd(0x27, 16, 2);

//arrays used for servo rotation calculation
//H[]- the center position of the platform can be moved with respect to the base, this is
//translation vector representing this move
static float RMb[3][3], qi[3][6], li[3][6], T[3], H[3] = {0, 0, z_platf}, li_L[6], a[3][6];
static float li_L_prev[6], M[6], N[6], L[6], alpha[6], N0, M0, L0, H0, Alpha0;

//JOYSTICK
const int btn_pin = 7;
const int SW_pin = 2; // a digital pin connected to switch output
const int X_pin = A0;// an analog pin connected to X
const int Y_pin = A1;// an analog pin connected to Y output
const int debounce_delay = 200;
const int joystick_delay = 200;

int xValue;
int yValue;

int counter = 0;
int state1;
int btn_state = HIGH;
int btn_prev = HIGH;
unsigned long last_debounce_time = 0;
unsigned long last_joystickMove_time = 0;
void setup() {
  // put your setup code here, to run once:
  //attachment of servos to PWM digital pins of arduino
  servo[0].attach(3, MIN, MAX);
  servo[1].attach(5, MIN, MAX);
  servo[2].attach(6, MIN, MAX);
  servo[3].attach(9, MIN, MAX);
  servo[4].attach(10, MIN, MAX);
  servo[5].attach(11, MIN, MAX);
  //begin of serial communication
  Serial.begin(9600);
  
  //joystick
  pinMode(btn_pin, INPUT_PULLUP);
  pinMode(SW_pin, INPUT);
  digitalWrite(SW_pin, HIGH);
  state1 = HIGH;
  /*Serial.println("base matrix x y z");
    for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 6; j++)
    {
      Serial.print(b[i][j]);
      Serial.print("  ");
    }
    Serial.println(" ");
    }
    Serial.println("platform matrix x y z");
    for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 6; j++)
    {
      Serial.print(p[i][j]);
      Serial.print("  ");
    }
    Serial.println(" ");
    }*/
  Fmatrix();
  FT();
  Fqi();
  Fli();
  Fli_L();
  Ver_li_L();
  check();
  FAlpha();
  //activating the base position
  setPos(rpp);
  lcd.init();
  lcd.backlight();
}
//function calculating the full rotation matrix relative to the base
void Fmatrix()
{
  float psi = radians(rpp[5]);
  float theta = radians(rpp[4]);
  float phi = radians(rpp[3]);
  RMb[0][0] = cos(psi) * cos(theta);
  RMb[1][0] = -sin(psi) * cos(phi) + cos(psi) * sin(theta) * sin(phi);
  RMb[2][0] = sin(psi) * sin(phi) + cos(psi) * cos(phi) * sin(theta);

  RMb[0][1] = sin(psi) * cos(theta);
  RMb[1][1] = cos(psi) * cos(phi) + sin(psi) * sin(theta) * sin(phi);
  RMb[2][1] = cos(theta) * sin(phi);

  RMb[0][2] = -sin(theta);
  RMb[1][2] = -cos(psi) * sin(phi) + sin(psi) * sin(theta) * cos(phi);
  RMb[2][2] = cos(theta) * cos(phi);
}
//function calculating translation vector - desired move vector + home translation vector
void FT()
{
  T[0] = rpp[0] + H[0];
  T[1] = rpp[1] + H[1];
  T[2] = rpp[2] + H[2];
}
//function calculating the cordinates qi (x,y,z) of the anchor point pi with respect to the Base as a reference
void Fqi()
{
  for (int i = 0; i < 6; i++) {
    qi[0][i] = T[0] + RMb[0][0] * (p[0][i]) + RMb[0][1] * (p[1][i]) + RMb[0][2] * (p[2][i]);
    qi[1][i] = T[1] + RMb[1][0] * (p[0][i]) + RMb[1][1] * (p[1][i]) + RMb[1][2] * (p[2][i]);
    qi[2][i] = T[2] + RMb[2][0] * (p[0][i]) + RMb[2][1] * (p[1][i]) + RMb[2][2] * (p[2][i]);
  }
}

//function calculating li (x, y, z)
void Fli()
{
  for (int i = 0; i < 6; i++) {
    li[0][i] = qi[0][i] - b[0][i];
    li[1][i] = qi[1][i] - b[1][i];
    li[2][i] = qi[2][i] - b[2][i];
  }
}

//functions calculating and returning (if true) length li
void Fli_L() {
  for (int i = 0; i < 6; i++) {
    li_L_prev[i] = li_L[i];
    li_L[i] = sqrt(pow(li[0][i], 2) + pow(li[1][i], 2) + pow(li[2][i], 2));
  }
}

bool Ver_li_L() {
  for (int i = 0; i < 6; i++) {
    if (li_L[i] < (LBP - LA + 1) || li_L[i] > (LBP + LA - 1)) {
      return false;
    }
  }
  return true;
}

void check() {
  if (Ver_li_L() == false) {
    for (int i = 0; i < 6; i++) {
      li_L[i] = li_L_prev[i];
    }
  }
}

//function calculating the angle of the servo
void FAlpha() {

  for (int i = 0; i < 6; i++) {
    //M
    M[i] = 2 * LA * (qi[2][i] - b[2][i]);
    //N
    N[i] = 2 * LA * ((cos(beta[i]) * (qi[0][i] - b[0][i]) + sin(beta[i]) * (qi[1][i] - b[1][i])));
    //L
    L[i] = pow(li_L[i], 2) - (pow(LBP, 2) - pow(LA, 2));
  }
  for (int i = 0; i < 6; i++) {
    //Alpha
    alpha[i] =  (asin(L[i] / sqrt(pow(M[i], 2) + pow(N[i], 2))) - atan(N[i] / M[i])) * 180 / pi;

    //calculation of the points of the arm/leg joint on the i-th servo with coordinates (ai)
    a[0][i] = LA * cos(alpha[i]) * cos(beta[i]) + b[0][i];
    a[1][i] = LA * cos(alpha[i]) * sin(beta[i]) + b[1][i];
    a[2][i] = LA * sin(alpha[i]) + b[2][i];
  }
  //stores previous li_L values
  for (int i = 0; i < 6; i++) {
    li_L_prev[i] = li_L[i];
  }
  //N0
  N0 = 2 * LA * (p[1][0] - b[1][0]);
  //H0
  H0 = sqrt(pow(LBP, 2) + pow(LA, 2) - pow((p[0][0] - b[0][0]), 2) - pow((p[1][0] - b[1][0]), 2)) - p[2][0];
  //M0
  M0 = 2 * LA * H0;
  //L0
  L0 = 2 * pow(LA, 2);
  //Alpha0
  Alpha0 = (asin(L0 / sqrt(pow(N0, 2) + pow(M0, 2))) - atan(N0 / M0)) * 180 / pi;
}

void setPos(float var[]) {
  for (int i = 0; i < 6; i++)
  {
    if (i == INV1 || i == INV2 || i == INV3) {
      servo_pos[i] = constrain(zero[i] - (alpha[i] - Alpha0), MIN, MAX);
    }
    else {
      servo_pos[i] = constrain(zero[i] + (alpha[i] - Alpha0), MIN, MAX);
    }
    servo[i].write(servo_pos[i]);

  }
}
//JOYSTICK functions
int Button(int x) {
  int btn_read = digitalRead(x);
  if ( btn_read != btn_prev) {
    last_debounce_time = millis();
  }

  if (millis() > (last_debounce_time + debounce_delay)) {
    if ( btn_read != btn_state) {
      btn_state = btn_read;
      if ( btn_state == LOW) {
        counter++;
        //Serial.print(counter);
        //Serial.print(" ");
      }
    }
  }
  btn_prev = btn_read;
}
byte StateZ() {
  int a = counter % 2;
  if (a == 0) {
    state1 = HIGH;
  } else {
    state1 = LOW;
  }
  //Serial.println(state1);
  return state1;
}
//function - joystick control
void Control () {

  xValue = map(analogRead(X_pin), 0, 1023, -25, 25);
  yValue = map(analogRead(Y_pin), 0, 1023, -25, 25);
  //Serial.println(xValue);
  if (millis() > (last_joystickMove_time + joystick_delay)) {
    //Serial.println(digitalRead(SW_pin));
    //Serial.println(state1);
    //X+/X-
    if (xValue > 10 && digitalRead(SW_pin) == HIGH && state1 == LOW) {
      if (rpp[0] <= 24) {
        rpp[0]++;
      } else if (rpp[0] < 24) {
        rpp[0] = 25;
      }
    }
    if (xValue < -10 && digitalRead(SW_pin) == HIGH && state1 == LOW) {
      if (rpp[0] >= -24) {
        rpp[0]--;
      } else if (rpp[0] > -24) {
        rpp[0] = -25;
      }
    }
    //Y+/Y-
    if (yValue > 10 && digitalRead(SW_pin) == HIGH && state1 == LOW) {
      if (rpp[1] <= 24) {
        rpp[1]++;
      } else if (rpp[1] < 24) {
        rpp[1] = 25;
      }
    }
    if (yValue < -10 && digitalRead(SW_pin) == HIGH && state1 == LOW) {
      if (rpp[1] >= -24) {
        rpp[1]--;
      } else if (rpp[1] > -24) {
        rpp[1] = -25;
      }
    }
    //Roll X+/X-
    if (xValue > 10 && digitalRead(SW_pin) == LOW && state1 == LOW) {
      if (rpp[3] <= 24) {
        rpp[3]++;
      } else if (rpp[3] < 24) {
        rpp[3] = 25;
      }
    }
    if (xValue < -10 && digitalRead(SW_pin) == LOW && state1 == LOW) {
      if (rpp[3] >= -24) {
        rpp[3]--;
      } else if (rpp[3] > -24) {
        rpp[3] = -25;
      }
    }
    //Pitch Y+/Y-
    if (yValue > 10 && digitalRead(SW_pin) == LOW && state1 == LOW) {
      if (rpp[4] <= 24) {
        rpp[4]++;
      } else if (rpp[4] < 24) {
        rpp[4] = 25;
      }
    }
    if (yValue < -10 && digitalRead(SW_pin) == LOW && state1 == LOW) {
      if (rpp[4] >= -24) {
        rpp[4]--;
      } else if (rpp[4] > -24) {
        rpp[4] = -25;
      }
    }
    //Z+/Z-
    if (xValue > 10 && digitalRead(SW_pin) == HIGH && state1 == HIGH) {
      if (rpp[2] <= 11) {
        rpp[2]++;
      } else if (rpp[2] < 11) {
        rpp[2] = 12;
      }
    }
    if (xValue < -10 && digitalRead(SW_pin) == HIGH && state1 == HIGH) {
      if (rpp[2] >= -12) {
        rpp[2]--;
      } else if (rpp[2] > -12) {
        rpp[2] = -13;
      }
    }
    //Yaw Z+/Z-
    if (yValue > 10 && digitalRead(SW_pin) == LOW && state1 == HIGH) {
      if (rpp[5] <= 22) {
        rpp[5]++;
      } else if (rpp[5] < 22) {
        rpp[5] = 23;
      }
    }
    if (yValue < -10 && digitalRead(SW_pin) == LOW && state1 == HIGH) {
      if (rpp[5] >= -22) {
        rpp[5]--;
      } else if (rpp[5] > -22) {
        rpp[5] = -23;
      }
    }
  }
}


void loop() {
  //measuring time intervals
  uint32_t ts1 = micros();
  Button(btn_pin);
  StateZ();
  Control();
  Fmatrix();
  FT();
  Fqi();
  Fli();
  Fli_L();
  Ver_li_L();
  check();
  FAlpha();
  //activating the base position
  setPos(rpp);
  int lcd_width = 16;
 //TouchScreen
  int X,Y; //Touch Coordinates are stored in X,Y variable
   pinMode(Y1,INPUT);
   pinMode(Y2,INPUT);  
   digitalWrite(Y2,LOW);
   pinMode(X1,OUTPUT);
   digitalWrite(X1,HIGH);
   pinMode(X2,OUTPUT);
   digitalWrite(X2,LOW);
   X = (analogRead(Y1))/(1024/Xresolution); //Reads X axis touch position
    
   pinMode(X1,INPUT);
   pinMode(X2,INPUT);
   digitalWrite(X2,LOW);
   pinMode(Y1,OUTPUT);
   digitalWrite(Y1,HIGH);
   pinMode(Y2,OUTPUT);
   digitalWrite(Y2,LOW);
   Y = (analogRead(X1))/(1024/Yresolution); //Reads Y axis touch position
  
  //Display X and Y on Serial Monitor
   Serial.print("X = ");  
   Serial.print(X);
   Serial.print(" Y = ");
   Serial.println(Y);
//
  lcd.backlight();
  // Clear screen, draw character
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("X:");
  if(rpp[0]<0){
    lcd.print("-");
    lcd.print(round(abs(rpp[0])));
  } else{
    lcd.print(round(rpp[0]), " ");
  }
  lcd.print(" ");
  lcd.print("Y:");
  if(rpp[1]<0){
    lcd.print("-");
    lcd.print(round(abs(rpp[1])), " ");
  } else{
    lcd.print(round(rpp[1]), " ");
  }
  lcd.print(" ");
  lcd.print("Z:");
  if(rpp[2]<0){
    lcd.print("-");
    lcd.print(round(abs(rpp[2])), " ");
  } else{
    lcd.print(round(rpp[2]), " ");
  }
  lcd.print(" ");
  lcd.print(state1);

  lcd.setCursor(0, 1);
  lcd.print("RX:");
  if(rpp[3]<0){
    lcd.print("-");
    lcd.print(round(abs(rpp[3])), " ");
  } else{
    lcd.print(round(rpp[3]), " ");
  }
  lcd.print(" ");
  lcd.print("PY:");
  if(rpp[4]<0){
    lcd.print("-");
    lcd.print(round(abs(rpp[4])), " ");
  } else{
    lcd.print(round(rpp[4]), " ");
  }
  lcd.print(" ");
  lcd.print("YZ:");
  if(rpp[5]<0){
    lcd.print("-");
    lcd.print(round(abs(rpp[5])), " ");
  } else{
    lcd.print(round(rpp[5]), " ");
  }
    uint32_t ts2 = micros();
}
