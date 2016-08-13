#include <SPI.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <EEPROM.h>
#include <Wire.h>
#include "MotorDC.h"
#include <PID_v1.h>
#include "I2Cdev.h"

MotorDC motor1(5,8);
MotorDC motor2(6,7);

// PID
double angleSetpoint, angleInput, angleOutput;
double angleP = 10, angleI = 100,  angleD = 0.2;
PID anglePID(&angleInput, &angleOutput, &angleSetpoint,angleP,angleI,angleD, DIRECT);//REVERSE DIRECT

// 实例化SPI OLED
#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_RESET 13

Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, -1);

/***************MPU6050变量定义**********************/
double accX, accY, accZ;
double gyroY, gyroZ;
double compAngleY, gyroYangle;
int16_t tempRaw;
uint32_t timer;
uint8_t i2cData[14];


// 函数声明
void (* resetFunc) (void) = 0;
void IMU_fillter(void);           //IMU角度融合

void setup()
{
	Serial.begin(115200);  //蓝牙串口
  	Wire.begin();           //I2C总线

	display.begin(SSD1306_SWITCHCAPVCC);
	display.clearDisplay();
	display.setTextSize(4);
	display.setTextColor(WHITE);
	display.setCursor(40, 2);
	display.print("GO");
	display.display();

	delay(500);

	/*********************MPU6050初始化********************/
	i2cData[0] = 7;
	i2cData[1] = 0x00;
	i2cData[2] = 0x00;
	i2cData[3] = 0x00;
	while (i2cWrite(0x19, i2cData, 4, false));
	while (i2cWrite(0x6B, 0x01, true));
	while (i2cRead(0x75, i2cData, 1));
	if (i2cData[0] != 0x68)
	while (1);
	delay(100);

	while (i2cRead(0x3B, i2cData, 6));
	accX = (i2cData[0] << 8) | i2cData[1];
	accY = (i2cData[2] << 8) | i2cData[3];
	accZ = (i2cData[4] << 8) | i2cData[5];
	tempRaw = (i2cData[6] << 8) | i2cData[7];

	double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
	gyroYangle = pitch;
	compAngleY = pitch;

	timer = micros();


	angleInput = atan2(accX, accZ) * RAD_TO_DEG + 180.0;
  	angleSetpoint = 180;
  	anglePID.SetOutputLimits(-254,254);
  	anglePID.SetSampleTime(10);
  	anglePID.SetMode(AUTOMATIC);
	// 电机停机
  	motor1.setSpeed(0);
  	motor2.setSpeed(0);
}

void loop()
{

	angleInput = atan2(accX, accZ) * RAD_TO_DEG + 180.0;
	anglePID.SetTunings(angleP, angleI, angleD);      // * While most users will set the tunings once in the 
	anglePID.Compute();

	display.clearDisplay();
	display.setTextSize(1);
	display.setCursor(0, 0);

	display.println(atan2(accX, accZ) * RAD_TO_DEG + 180.0);
	display.println(angleOutput);
	display.println(angleP,angleI,angleD);
  	display.display();

	motor1.setSpeed(angleOutput);
	motor2.setSpeed(-angleOutput);

	// resetFunc();
	// delay(10000);

  	IMU_fillter();//3ms
}