#ifndef LINO_BASE_CONFIG_H
#define LINO_BASE_CONFIG_H

//uncomment the base you're building（根据你构建的底盘类型来选择正确的配置
#define LINO_BASE DIFFERENTIAL_DRIVE // 2WD and Tracked robot w/ 2 motors（两区差速和履带式差速转向机器人）
// #define LINO_BASE SKID_STEER      // 4WD robot（4驱滑动转向机器人）注：SKID是滑动的意思
// #define LINO_BASE ACKERMANN       // Car-like steering robot w/ 2 motors （类似汽车舵机转向的 两驱机器人）
// #define LINO_BASE ACKERMANN1      // Car-like steering robot w/ 1 motor （类似汽车舵机转向的 单驱动）
// #define LINO_BASE MECANUM         // Mecanum drive robot （麦克纳姆轮驱动机器人）

//uncomment the motor driver you're using（根据你的电机驱动选择）
#define USE_L298_DRIVER
// #define USE_BTS7960_DRIVER
// #define USE_ESC

//uncomment the IMU you're using（根据你使用的imu）
#define USE_GY85_IMU
// #define USE_MPU6050_IMU
// #define USE_MPU9150_IMU
// #define USE_MPU9250_IMU
// #define UN_USE_IMU //如果不想通过下位机发布imu数据，而是想通过上位机连接串口来获取imu数据，则选择该项

#define DEBUG 0 //是否开启 debug 模式，在命令行打印编码器信息

// 驱动的pid控制参数，有关pid的信息参考 --
#define K_P 1.2 // P constant
#define K_I 1.0 // I constant
#define K_D 0.6 // D constant

//define your robot' specs here（定义你的机器人规格信息）
#define MAX_RPM 50                // motor's maximum RPM（电机的最大转速，减速机减速后的数值）
#define COUNTS_PER_REV 4096       // wheel encoder's no of ticks per rev（车轮编码器的每圈脉冲数）
#define WHEEL_DIAMETER 0.19565    // wheel's diameter in meters（主动轮直径，单位米）
#define PWM_BITS 8                // PWM Resolution of the microcontroller（微控制器的pwd分辨率，根据芯片的位数，如果你是8位的单片机芯片，则是 2**8-1==255 ）
#define LR_WHEELS_DISTANCE 1.0    // distance between left and right wheels （左右轮的距离，单位米）注：如果你是2wd履带车，该值是实际距离*2
#define FR_WHEELS_DISTANCE 0.6    // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN（前后轮的距离，如果你是2WD/ACKERMANN则忽略该参数）
#define MAX_STEERING_ANGLE 0.415  // max steering angle. This only applies to Ackermann steering（车子的最大转向角，该参数只作用于阿卡曼转向底盘）
/*
ROBOT ORIENTATION                   |机器人 方向定义
         FRONT                      |      车头
    MOTOR1  MOTOR2  (2WD/ACKERMANN) |  电机1  电机2  (2WD/阿克曼/两驱履带)
    MOTOR3  MOTOR4  (4WD/MECANUM)   |  电机3  电机4  (4WD/麦克纳姆/四区履带)
         BACK                       |      车尾
*/

/// ENCODER PINS (编码器针脚定义)
#define MOTOR1_ENCODER_A 11
#define MOTOR1_ENCODER_B 12 

#define MOTOR2_ENCODER_A 14
#define MOTOR2_ENCODER_B 15 

#define MOTOR3_ENCODER_A 17
#define MOTOR3_ENCODER_B 16 

#define MOTOR4_ENCODER_A 9
#define MOTOR4_ENCODER_B 10

//MOTOR PINS (L298电机驱动针脚定义)
#ifdef USE_L298_DRIVER 
  #define MOTOR_DRIVER L298 //驱动类型

  #define MOTOR1_PWM 21     //pwm速度针脚
  #define MOTOR1_IN_A 20    //方向控制针脚
  #define MOTOR1_IN_B 1     //方向

  #define MOTOR2_PWM 5
  #define MOTOR2_IN_A 6
  #define MOTOR2_IN_B 8

  #define MOTOR3_PWM 22
  #define MOTOR3_IN_A 23
  #define MOTOR3_IN_B 0

  #define MOTOR4_PWM 4
  #define MOTOR4_IN_A 2
  #define MOTOR4_IN_B 3

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif 
// BTS7960类型驱动针脚定义
#ifdef USE_BTS7960_DRIVER
  #define MOTOR_DRIVER BTS7960  

  #define MOTOR1_PWM 1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR1_IN_A 21
  #define MOTOR1_IN_B 20

  #define MOTOR2_PWM 8 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR2_IN_A 5
  #define MOTOR2_IN_B 6

  #define MOTOR3_PWM 0 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR3_IN_A 22
  #define MOTOR3_IN_B 23

  #define MOTOR4_PWM 2 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR4_IN_A 4
  #define MOTOR4_IN_B 3

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif

// ESC类型驱动针脚定义
#ifdef USE_ESC
  #define MOTOR_DRIVER ESC  

  #define MOTOR1_PWM 1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR1_IN_A 21
  #define MOTOR1_IN_B 20

  #define MOTOR2_PWM 8 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR2_IN_A 5
  #define MOTOR2_IN_B 6

  #define MOTOR3_PWM 0 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR3_IN_A 22
  #define MOTOR3_IN_B 23

  #define MOTOR4_PWM 2 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR4_IN_A 4
  #define MOTOR4_IN_B 3

  #define PWM_MAX 400
  #define PWM_MIN -PWM_MAX
#endif

// 定义阿克曼舵机的针脚
#define STEERING_PIN 7

#endif
