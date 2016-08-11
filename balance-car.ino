#include <SPI.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <EEPROM.h>
#include <Wire.h>
//#include "Camera.h"
// #include "Bitmap.h"
#include "MotorDC.h"
// #include <Encoder.h>
#include <PID_v1.h>
#define IMU_OUTPUT;
MotorDC motor1(5,8);
MotorDC motor2(6,7);
#define NONSTART 0x0001    //启动标志
#define TURNING 0x0002     //转向中标志
#define FALLDOWN 0x0004    //摔倒标志
#define COMDONE 0x0020     //帧数据结束标志

double angleSetpoint, angleInput, angleOutput;
PID anglePID(&angleInput, &angleOutput, &angleSetpoint,30,250,0.50, DIRECT);//REVERSE DIRECT

/***************MPU6050变量定义**********************/
double accX, accY, accZ;
double gyroY, gyroZ;
double compAngleY, gyroYangle;
int16_t tempRaw;
uint32_t timer;
uint8_t i2cData[14];

/*********************其他变量**********************/
uint32_t FLAG , CAM_timer , SONIC_timer ;
float distance_cm, joy_x, joy_y;
char comdata[19], data_p; //串口缓存数据
union EEPROM_DATA {       //联合体用于分段储存EEPROM数据
  double data;
  uint8_t dchar[4];
} pre_angle;


/***************PID变量定义**********************/
#ifdef PARAM_DEBUG  //角度环数据
double  P_angle = 0, I_angle = 0, D_angle = 0;
#else
#define P_angle  0
#define I_angle  0
#define D_angle  0
#endif
double  angle_setpoint = 0, angle_output, angle_integral;
uint32_t angle_PID_timer;

#ifdef PARAM_DEBUG  //速度环数据
double  P_speed = 0, I_speed = 0;
#else
#define P_speed  0
#define I_speed  0
#endif
double  speed_setpoint = 0, speed_output, speed_integral;
uint32_t speed_PID_timer;

#ifdef PARAM_DEBUG  //方向环数据
double  P_turn = 0, I_turn = 0;
#else
#define P_turn  0
#define I_turn  0
#endif
double  turn_integral = 0 , turn_output ;

//OLED
#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_RESET 13
//oled spi 实例化OLED
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, -1);

void IMU_fillter(void);           //IMU角度融合
void angle_PID_compute(void);     //角度环计算
void speed_PID_compute(void);     //速度环计算
void write_eeprom(double);        //写入EEPROM

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

	FLAG |= NONSTART;

	angleSetpoint = atan2(accX, accZ) * RAD_TO_DEG + 180.0;

  	// anglePID.SetMode(AUTOMATIC);
  

  	motor1.setSpeed(0);
  	motor2.setSpeed(0);
}

void loop()
{
	IMU_fillter();//3ms
  	angle_PID_compute();

	display.clearDisplay();
	display.setTextSize(1);
	display.setCursor(0, 0);

  	display.println(atan2(accX, accZ) * RAD_TO_DEG + 180.0);
  	angleInput = atan2(accX, accZ) * RAD_TO_DEG + 180.0;

  //Setpoint = 22.0;
  // myPID.SetTunings(kp,ki,kd);

  	display.println(micros());

	// anglePID.SetOutputLimits(-254,254);
  	// anglePID.SetSampleTime(5);
  // myPID.SetMode(AUTOMATIC);
  	// anglePID.Compute();

  	display.println(angleOutput);


  	display.display();

	// motor1.setSpeed(angleOutput);
	// motor2.setSpeed(-angleOutput);


}