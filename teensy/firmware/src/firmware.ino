#if (ARDUINO >= 100)
    #include <Arduino.h>
#else
    #include <WProgram.h>
#endif

#include <Servo.h>

#include "ros.h"
#include "ros/time.h"
//header file for publishing velocities for odom（用于发布原始速度raw_vel消息的头文件）
#include "lino_msgs/Velocities.h"
//header file for cmd_subscribing to "cmd_vel"（用于接受上位机控制信息cmd_vel消息的头文件）
#include "geometry_msgs/Twist.h"
//header file for pid server（pid参数调节的头文件）
#include "lino_msgs/PID.h"
//header file for imu
#include "lino_msgs/Imu.h"

//基本配置头文件
#include "lino_base_config.h" 
#include "Motor.h"
#include "Kinematics.h"
#include "PID.h"
#include "Imu.h"

#define ENCODER_OPTIMIZE_INTERRUPTS // comment this out on Non-Teensy boards
#include "Encoder.h"

#define IMU_PUBLISH_RATE 20 //hz，imu发布频率
#define COMMAND_RATE 20 //hz，控制的频率
#define DEBUG_RATE 5 //debug打印消息频率

// 初始化各个电机编码器对象，用于获取电机的当前转速rpm
Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV); 
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV); 
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV); 

// 私服舵机控制对象
Servo steering_servo;

// 初始化电机驱动控制器对象，用于控制驱动输出
Controller motor1_controller(Controller::MOTOR_DRIVER, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Controller motor2_controller(Controller::MOTOR_DRIVER, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B); 
Controller motor3_controller(Controller::MOTOR_DRIVER, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Controller motor4_controller(Controller::MOTOR_DRIVER, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

// 初始化各个电机的pid控制对象，用于输出控制驱动器的PWM值
PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

// 初始化运动学模型解算对象，用于车辆速度和轮子转速之间的换算
Kinematics kinematics(Kinematics::LINO_BASE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

// 全局需要达到的速度
float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;
// 全局控制驱动的时间差
unsigned long g_prev_command_time = 0;

//callback function prototypes（订阅速度信息的回调函数）
void commandCallback(const geometry_msgs::Twist& cmd_msg);
void PIDCallback(const lino_msgs::PID& pid); //pid参数调试回调函数

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
ros::Subscriber<lino_msgs::PID> pid_sub("pid", PIDCallback);

// imu发布者
lino_msgs::Imu raw_imu_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);

// 发布原始的速度消息
lino_msgs::Velocities raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

void setup()
{
    steering_servo.attach(STEERING_PIN); //初始化舵机控制对象
    steering_servo.write(90); 
    
    nh.initNode();
    nh.getHardware()->setBaud(57600);
    nh.subscribe(pid_sub);
    nh.subscribe(cmd_sub);
    nh.advertise(raw_vel_pub);
    nh.advertise(raw_imu_pub);

    while (!nh.connected()) //一直等待主节点
    {
        nh.spinOnce();
    }
    nh.loginfo("LINOBASE CONNECTED");
    delay(1);
}

void loop()
{
    static unsigned long prev_control_time = 0;
    static unsigned long prev_imu_time = 0;
    static unsigned long prev_debug_time = 0;
    static bool imu_is_initialized;

    //this block drives the robot based on defined rate（根据上面定义的频率来控制电机）
    if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE))
    {
        moveBase();
        prev_control_time = millis();
    }

    //this block stops the motor when no command is received（如果收到控制命令超多400ms，就让电机停下来）
    if ((millis() - g_prev_command_time) >= 400)
    {
        stopBase();
    }

    //this block publishes the IMU data based on defined rate（根据定义的速率发布imu数据）
    if ((millis() - prev_imu_time) >= (1000 / IMU_PUBLISH_RATE))
    {
        //sanity check if the IMU is connected（检查是否初始化）
        if (!imu_is_initialized)
        {
            imu_is_initialized = initIMU();

            if(imu_is_initialized)
                nh.loginfo("IMU Initialized");
            else
                nh.logfatal("IMU failed to initialize. Check your IMU connection.");
        }
        else
        {
            publishIMU();
        }
        prev_imu_time = millis();
    }

    //this block displays the encoder readings. change DEBUG to 0 if you don't want to display（是否debug编码器的数据）
    if(DEBUG)
    {
        if ((millis() - prev_debug_time) >= (1000 / DEBUG_RATE))
        {
            printDebug();
            prev_debug_time = millis();
        }
    }
    //call all the callbacks waiting to be called（等待所有的回调）
    nh.spinOnce();
}

void PIDCallback(const lino_msgs::PID& pid)
{
    //callback function every time PID constants are received from lino_pid for tuning
    //this callback receives pid object where P,I, and D constants are stored
    motor1_pid.updateConstants(pid.p, pid.i, pid.d);
    motor2_pid.updateConstants(pid.p, pid.i, pid.d);
    motor3_pid.updateConstants(pid.p, pid.i, pid.d);
    motor4_pid.updateConstants(pid.p, pid.i, pid.d);
}

void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
    //callback function every time linear and angular speed is received from 'cmd_vel' topic
    //this callback function receives cmd_msg object where linear and angular speed are stored
    g_req_linear_vel_x = cmd_msg.linear.x;
    g_req_linear_vel_y = cmd_msg.linear.y;
    g_req_angular_vel_z = cmd_msg.angular.z;

    g_prev_command_time = millis();
}

void moveBase()
{
    //get the required rpm for each motor based on required velocities, and base used（根据速度命令计算每个电机需要的转速）
    Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);

    //get the current speed of each motor（计算当前每个电机的转速）
    int current_rpm1 = motor1_encoder.getRPM();
    int current_rpm2 = motor2_encoder.getRPM();
    int current_rpm3 = motor3_encoder.getRPM();
    int current_rpm4 = motor4_encoder.getRPM();

    //根据微控制器的最大PWM调节数来左限幅，防止pid输出数值过大
    //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
    motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
    motor3_controller.spin(motor3_pid.compute(req_rpm.motor3, current_rpm3));  
    motor4_controller.spin(motor4_pid.compute(req_rpm.motor4, current_rpm4));    

    Kinematics::velocities current_vel;

    // 如果是阿克曼转向
    if(kinematics.base_platform == Kinematics::ACKERMANN || kinematics.base_platform == Kinematics::ACKERMANN1)
    {
        float current_steering_angle;
        // 根据要达到的角速度、阿克曼最大的转向角，来重设角速度，并把转角和舵机的转角做映射
        current_steering_angle = steer(g_req_angular_vel_z);
        current_vel = kinematics.getVelocities(current_steering_angle, current_rpm1, current_rpm2);
    }
    else
    {
        current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, current_rpm3, current_rpm4);
    }
    
    //pass velocities to publisher object（将速度转给发布者发布）
    raw_vel_msg.linear_x = current_vel.linear_x;
    raw_vel_msg.linear_y = current_vel.linear_y;
    raw_vel_msg.angular_z = current_vel.angular_z;

    //publish raw_vel_msg
    raw_vel_pub.publish(&raw_vel_msg);
}

//停止电机函数
void stopBase()
{
    g_req_linear_vel_x = 0;
    g_req_linear_vel_y = 0;
    g_req_angular_vel_z = 0;
}

// 发布imu消息的函数
void publishIMU()
{
    //pass accelerometer data to imu object（线加速度）
    raw_imu_msg.linear_acceleration = readAccelerometer();

    //pass gyroscope data to imu object（角速度）
    raw_imu_msg.angular_velocity = readGyroscope();

    //pass accelerometer data to imu object（磁力计)
    raw_imu_msg.magnetic_field = readMagnetometer();

    //publish raw_imu_msg
    raw_imu_pub.publish(&raw_imu_msg);
}

float steer(float steering_angle)
{
    //steering function for ACKERMANN base（要达到目标角速度，舵机需要转到的角度）
    float servo_steering_angle;

    // 根据最大转向角做限幅
    steering_angle = constrain(steering_angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
    // 把角度映射到舵机转向的分量上
    servo_steering_angle = mapFloat(steering_angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE, PI, 0) * (180 / PI);
    // 把舵机转到指定角度
    steering_servo.write(servo_steering_angle);
    // 把限幅后的角度返回，用于发布raw_vel消息
    return steering_angle;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// debug输出函数
void printDebug()
{
    char buffer[50];

    sprintf (buffer, "Encoder FrontLeft  : %ld", motor1_encoder.read());
    nh.loginfo(buffer);
    sprintf (buffer, "Encoder FrontRight : %ld", motor2_encoder.read());
    nh.loginfo(buffer);
    sprintf (buffer, "Encoder RearLeft   : %ld", motor3_encoder.read());
    nh.loginfo(buffer);
    sprintf (buffer, "Encoder RearRight  : %ld", motor4_encoder.read());
    nh.loginfo(buffer);
}
