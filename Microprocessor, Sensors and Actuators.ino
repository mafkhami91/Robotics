 /*******************************************************
   Name: Microprocessor, Sensors and Actuators
   Description: Print X,Y, and Z accelerometer readings and the X‐Z, Y‐Z, and X‐Y angles Serial Monitor 
   Email: smafkhami@purdue.edu
   Date: 21/2/2019
   Author: Mahdi Afkhamiaghda
 ********************************************************/

#include <Wire.h>  //Call the I2C library built in Arduino

//Set the address of the register
#define Register_ID 0
#define Register_2D 0x2D  //standby,measurement,sleep,wake_up mode set
#define Register_X0 0x32  //DATAX0, store the value of X0
#define Register_X1 0x33  //DATAX1, store the value of X1
#define Register_Y0 0x34  //DATAY0,  store the value of Y0
#define Register_Y1 0x35  //DATAY1,  store the value of Y1
#define Register_Z0 0x36  //DATAZ0,  store the value of Z0
#define Register_Z1 0x37  //DATAZ1,  store the value of Z1

int ADXAddress = 0x53;  //I2C address
//int reading = 0;
//int val = 0;
int X0, X1, X_out;
int Y0, Y1, Y_out;
int Z1, Z0, Z_out;
double Xg, Yg, Zg;
double XY,YZ, XZ;
const int motorIn1 = 9;  //attach to one of the pin of the motor
const int motorIn2 = 10;  //attach to another pin of the motor
const int redled = 5;  //attach red LED to the arduino
const int greenled = 6;  //attach green LED to the arduino
int pos;                // Mapping  Variable
int pos2;
int pos3;
int pos4;
void setup()
{
  // defining I.O
  analogReference(EXTERNAL);  // Setting AREF to 3.3 V
  pinMode(motorIn1,OUTPUT);  //initialize the motorIn1 pin as output 
  pinMode(motorIn2,OUTPUT);  //initialize the motorIn2 pin as output
  pinMode(redled, OUTPUT);  // Setting the red LED as output
  pinMode(greenled, OUTPUT);  // Setting the green LED as output
  
  Serial.begin(9600);//Set the baud rate of serial monitor as 9600bps
  delay(100);
  Wire.begin();  //Initialize I2C
  delay(100);
  Wire.beginTransmission(ADXAddress); //transmit to device ADXAddress 0x53  
  Wire.write(Register_2D); //
  Wire.write(8);  //measuring enable  
  Wire.endTransmission(); //end transmitting
  Serial.println("(X, Y, Z) -- [X-Y, Y-Z, X-Z]");
  Serial.println("============================");

}
void clockwise(int speed){          //Defining the rotaion of DC Motor
  analogWrite(motorIn1, speed); 
  analogWrite(motorIn2, 0);
}
void counterclockwise(int speed){   //Defining the rotaion of DC Motor
  analogWrite(motorIn1, 0); 
  analogWrite(motorIn2, speed);
}
void loop()
{
  Wire.beginTransmission(ADXAddress);  //transmit to device ADXAddress 0x53  
  Wire.write(Register_X0);  //request the X0 value
  Wire.write(Register_X1);  //request the X1 value
  Wire.endTransmission();  //stop transmitting
  Wire.requestFrom(ADXAddress, 2); //request 2 bytes from device 0x53
  if (Wire.available() <= 2);  //if the received value is less than 2 bytes,then
  {
    X0 = Wire.read();  //receive X0 value
    X1 = Wire.read();  // receive X1 value
    X1 = X1 << 8;   //X1 left shift 8 bits
    X_out = X0 + X1;  //the X_out is 0xX1X0
  }

  Wire.beginTransmission(ADXAddress);  //request 2 bytes from device 0x53
  Wire.write(Register_Y0);
  Wire.write(Register_Y1);
  Wire.endTransmission();
  Wire.requestFrom(ADXAddress, 2);
  if (Wire.available() <= 2);
  {
    Y0 = Wire.read();
    Y1 = Wire.read();
    Y1 = Y1 << 8;
    Y_out = Y0 + Y1;
  }

  Wire.beginTransmission(ADXAddress);  //request 2 bytes from device 0x53
  Wire.write(Register_Z0);
  Wire.write(Register_Z1);
  Wire.endTransmission();
  Wire.requestFrom(ADXAddress, 2);
  if (Wire.available() <= 2);
  {
    Z0 = Wire.read();
    Z1 = Wire.read();
    Z1 = Z1 << 8;
    Z_out = Z0 + Z1;
  }
  Xg = X_out / 256.00; //Convert the output result into the acceleration g, accurate to 2 decimal points.
  Yg = Y_out / 256.00;
  Zg = Z_out / 256.00;
  XY = atan2(Xg, sqrt(Yg*Yg + Zg*Zg)) * 57.3;  // roll for x - radian to degree by dividing 180 by PI
  YZ = atan2(Yg, sqrt(Xg*Xg + Zg*Zg)) * 57.3;  // pitch for y - radian to degree by dividing 180 by PI
  XZ = atan2(sqrt(Xg*Xg + Yg*Yg),Zg) * 57.3;
  Serial.println(""); 
  Serial.print("("); 
  Serial.print(Xg);  //print the value of Xg
  Serial.print(", ");
  Serial.print(Yg);  //print the value of Yg
  Serial.print(", ");
  Serial.print(Zg);  //print the value of Zg
  Serial.print(")");
  Serial.print("\t");
  Serial.print("||");
  Serial.print("\t");
  Serial.print("[");
  Serial.print(XY);  //print the value of X_Y angle
  Serial.print(", "); 
  Serial.print(YZ);  //print the value of Y_Z angle
  Serial.print(", "); 
  Serial.print(XZ);  //print the value of X_Z angle  
  Serial.print("]");

  //Setting the motor speed on X-Y plane 
  if (XY > 10 & XY < 90){                   // 10 instead of 0 to counteract the noise and error effect
         pos = map (XY, 0, 90, 0, 255);  
         clockwise (pos);
  }else if (XY< 0 & XY > - 90){ 
          pos2 = map (XY, 0, -90, 0, 255);   
          counterclockwise(pos2);
  }
  //Setting the motor speed on Y-Z plane
  if (YZ > 10 & YZ < 90){                 // 10 instead of 0 to counteract the noise and error effect
          pos = map (YZ, 0, 90, 0, 255); 
          clockwise(pos);
  }else if(YZ < 0 & YZ > - 90){
         pos = map (YZ, -90, 0, 0, 255);  
         counterclockwise(pos);
  }
  //Setting the LED status on X-Y plane
  if (abs(XY) >10){                        // 10 instead of 0 to counteract the noise and error effect
    pos3 = map (XY, 0, 90, 0, 255);
    analogWrite(redled, pos3);
  }
  //Setting the LED status on Y-Z plane
  if (abs(YZ) >10){                        // 10 instead of 0 to counteract the noise and error effect
    pos4 = map (YZ, 0, 90, 0, 255); 
    analogWrite(greenled, pos4);
  }
  
  delay(1000);  //Delay 1s
}
