#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>


//----------------------------------------------------------------------------------------------------------
///////////////
// Calibration:
///////////////

float kp = 1.5;
float ki = 1.0;
float kd = 0.01;

float kpaa = 0.3;
float kiaa = 1.0;
float kdaa = 0.01;

float kpad = 0.8;
float kiad = 1.0;
float kdad = 0.01;

int a_threshold = 0; // acceleration threshold, i.e. stop acceleration pid control if rpm reached within this threshold limit.
int low_rpm_threshold = 400;  // lowest rpm to be controlled by the acceleration pid controller.


//----------------------------------------------------------------------------------------------------------
/////////////
// Variables:
/////////////

long prevT = 0;
// Use the "volatile" directive for variables used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float vFilt = 0;
float vPrev = 0;
float vFiltPrev = 0;

float eintegral = 0;
float ediff = 0;
float eprev = 0;


//----------------------------------------------------------------------------------------------------------
/////////////
// Pin Setup:
/////////////

#define ENCA 3
#define ENCB 2
#define PWM 9
#define IN1 8
#define IN2 7

// Display Setup
LiquidCrystal_I2C lcd(0x3F,20,4);  // set the LCD address to 0x3F for a 20 chars and 4 line display

// Keyboard Setup
const byte ROWS = 4; 
const byte COLS = 4; 

char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte rowPins[ROWS] = {A0, 4, 5, 6}; //r1, r2, r3, r4
byte colPins[COLS] = {10, 11, 12, 13}; //c1, c2, c3, c4

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);


//----------------------------------------------------------------------------------------------------------

void setup()  // Start of setup:
{
  Serial.begin(115200);

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);

  // Turn off motors - Initial state
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  lcd.init();                      // initialize the lcd 
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(3,0);
  lcd.print("Hello!");
  
}  // End of setup.



//----------------------------------------------------------------------------------------------------------

void loop()  // Start of loop:
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Enter the");
  lcd.setCursor(0,1);
  lcd.print("acceleration time");
  int ta = get_num(2,10);
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Enter the rpm");
  int rpm = get_num(1000,5000);
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Enter the rpm time");
  int tr = get_num(1,30);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Enter the");
  lcd.setCursor(0,1);
  lcd.print("deceleration time");
  int td = get_num(2,10);
  
  acc_pid(0,rpm,ta);
  rpm_pid(rpm,tr);
  acc_pid(rpm,0,td);
  read_rpm(2);
  
  clear_data(-1);
}  // End of loop.

//----------------------------------------------------------------------------------------------------------

int get_num(int a, int b){
  String s="";
  while(true){
    char customKey = customKeypad.getKey();
    if(customKey){
      if(customKey=='A' || customKey=='B' || customKey=='C' || customKey=='D'){}
      else if(customKey=='*'){
        if (s.length() > 0) {
          s = s.substring(0, s.length() - 1);
          lcd.setCursor(0,3);
          lcd.print("                    ");
          lcd.setCursor(0,3);
          lcd.print(s);
          }
        }
      else if(customKey=='#'){
        if(s.length()!=0){
          int num = s.toInt();
          if(num>=a && num<=b){return num;}
          else{
            s="";
            lcd.setCursor(0,3);
            lcd.print("                    ");
            lcd.setCursor(0,3);
            lcd.print(a);
            lcd.print(" < number < " );
            lcd.print(b);
           }
         }
         else{
          lcd.setCursor(0,3);
          lcd.print("                    ");
          }
       }
      else if(s.length()>=4){}
      else {
        s=s+customKey;
        lcd.setCursor(0,3);
        lcd.print("                    ");
        lcd.setCursor(0,3);
        lcd.print(s);
      }
      }
  }
}


//----------------------------------------------------------------------------------------------------------

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal); // Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);    
  }
}



//----------------------------------------------------------------------------------------------------------
 
void readEncoder()  // The interrupt runs this to calculate the period between pulses:
{
  // Read the direction ir sensor when ir_interrupt rises
  int b = digitalRead(ENCB);
  
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;

  // Compute count/s
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e6;
  velocity_i = increment/deltaT;
  prevT_i = currT;

}  // End of Pulse_Event.


//----------------------------------------------------------------------------------------------------------

void rpm_pid(int set_rpm, int tr)
{
  long t = millis();
  prevT = micros();
  while(millis()-t<=tr*1.0e3){
    char customKey = customKeypad.getKey();
    if(customKey=='D'){
      setMotor(0,0,PWM,IN1,IN2);
      break;
      }

    // read the velocity.
    float velocity = 0;
    noInterrupts(); // disable interrupts temporarily while reading
    velocity = velocity_i;
    interrupts(); // turn interrupts back on

    // Compute delta t.
    long currT = micros();
    float deltaT = ((float) (currT-prevT))/1.0e6;
    prevT = currT;

    // Convert count/s to RPM
    float v = velocity/5.0*60.0;
  
    // Low-pass filter (25 Hz cutoff)
    vFilt = 0.854*vFilt + 0.0728*v + 0.0728*vPrev;
    vPrev = v;
  
    // Compute the control signal u
    float e = set_rpm-vFilt;
    eintegral = eintegral + e*deltaT;
    ediff = (e - eprev)/deltaT;
    eprev = e;
    
    float u = kp*e + ki*eintegral + kd*ediff;
  
    // Set the motor speed and direction
    int dir = 1;
    if (u<0){
      dir = -1;
    }
    int pwr = (int) fabs(u);
    if(pwr > 255){
      pwr = 255;
    }
    setMotor(dir,pwr,PWM,IN1,IN2);

    Serial.print(vFilt);
    Serial.print(" ");
    Serial.print(set_rpm);
    Serial.println();
    delay(1);
    
  }
  setMotor(0,0,PWM,IN1,IN2);
  clear_data(0);
}


//----------------------------------------------------------------------------------------------------------

void clear_data(int a){
  if(a==1){
    //vFilt = 0;
    //vPrev = 0;
    //eintegral = 0;
    //ediff = 0;
    //eprev = 0;
  }
  if(a==-1){
    eintegral = 0;
    ediff = 0;
    eprev = 0;
    }
}


//----------------------------------------------------------------------------------------------------------

void read_rpm(int t){
  long ts = millis();
  while(millis()-ts<=t*1.0e3){
    char customKey = customKeypad.getKey();
    if(customKey=='D'){
      setMotor(0,0,PWM,IN1,IN2);
      break;
      }

    // read the velocity.
    float velocity = 0;
    noInterrupts(); // disable interrupts temporarily while reading
    velocity = velocity_i;
    interrupts(); // turn interrupts back on

    // Convert count/s to RPM
    float v = velocity/5.0*60.0;
  
    // Low-pass filter (25 Hz cutoff)
    vFilt = 0.854*vFilt + 0.0728*v + 0.0728*vPrev;
    vPrev = v;
    Serial.print(vFilt);
    Serial.print(" ");
    Serial.print(0);
    Serial.println();
    delay(1);
  }
}



//----------------------------------------------------------------------------------------------------------

void acc_pid(int rpm1, int rpm2, int ta)
{
  float a = (float)(rpm2-rpm1)/ta;
  float set_rpm = rpm1;
  
  int pol = 1; // polarity of acceleration.
  if(a<0){pol = -1;}
  
  float kpa = kpaa;
  float kia = kiaa;
  float kda = kdaa;
  if(pol==-1){
    kpa = kpad;
    kia = kiad;
    kda = kdad;
    }
  
  prevT = micros();
  while(set_rpm*pol<=rpm2*pol-a_threshold){
    char customKey = customKeypad.getKey();
    if(customKey=='D'){
      setMotor(0,0,PWM,IN1,IN2);
      break;
      }

    // read the velocity.
    float velocity = 0;
    noInterrupts(); // disable interrupts temporarily while reading
    velocity = velocity_i;
    interrupts(); // turn interrupts back on

    // Compute delta t.
    long currT = micros();
    float deltaT = ((float) (currT-prevT))/1.0e6;
    prevT = currT;

    // Calculate set_rpm
    set_rpm = set_rpm + (a*deltaT);

    // Convert count/s to RPM
    float v = velocity/5.0*60.0;
  
    // Low-pass filter (25 Hz cutoff)
    vFilt = 0.854*vFilt + 0.0728*v + 0.0728*vPrev;    
    vPrev = v;

    if(set_rpm>low_rpm_threshold){
      // Compute the control signal u
      float e = set_rpm-vFilt;
      eintegral = eintegral + e*deltaT;
      ediff = (e - eprev)/deltaT;
      eprev = e;
      
      float u = kpa*e + kia*eintegral + kda*ediff;
    
      // Set the motor speed and direction
      int dir = 1;
      if (u<0){
        dir = -1;
      }
      int pwr = (int) fabs(u);
      if(pwr > 255){
        pwr = 255;
      }
      setMotor(dir,pwr,PWM,IN1,IN2);
    }
  else{setMotor(0,0,PWM,IN1,IN2);}
    
    Serial.print(vFilt);
    Serial.print(" ");
    Serial.print(set_rpm);
    Serial.println();
    delay(1);
    
  }
  setMotor(0,0,PWM,IN1,IN2);
  clear_data(0);
}
