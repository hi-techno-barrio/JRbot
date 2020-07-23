
/* ------------------------------------------------------------------------------------------------------------- */
//  JRbot  
// Hi-Techno Barrio
// by: Christopher Coballes
/*---------------------------------------------------------------------------------------------------------------*/

#define XENTRINOBOT
#include "xentrino_base_config.h"
#include "xentrino.h"
#include "Encoder.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
 
Velocity M1_Wheel(MAX_RPM); // left motor
Velocity M2_Wheel(MAX_RPM); // right motor 

Encoder M1_Encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B);
Encoder M2_Encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B);

Controller M1_control(Controller::MOTOR_DRIVER, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Controller M2_control(Controller::MOTOR_DRIVER, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B); 

PID M1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D); // left
PID M2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D); // right

float req_linear_vel_x = 0;
float req_linear_vel_y = 0;
float req_angular_vel_z = 0;
unsigned long prev_command_time = 0;

Kinematics Kinematics(MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE); 
 
//initializing all the variables
void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
    //callback function every time linear and angular speed is received from 'cmd_vel' topic
    //this callback function receives cmd_msg object where linear and angular speed are stored
    req_linear_vel_x = cmd_msg.linear.x;
    req_linear_vel_y = cmd_msg.linear.y;
    req_angular_vel_z = cmd_msg.angular.z;

    prev_command_time = millis();
}
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", commandCallback);  
geometry_msgs::Vector3Stamped raw_vel_msg;                                
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);  

void setup() {
  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  nh.advertise(raw_vel_pub); //prepare to publish speed in ROS topic
    if (!nh.connected()){
       //led_indicator'  
       }
    else{
       //led_static ;
    }
}

void loop() {
 static unsigned long prev_control_time = 0;
  //call all the callbacks waiting to be called
   if ((millis() - prev_command_time) >= 400)
    {
       stopBase();
    }
    
   //this block drives the robot based on defined rate
    if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE))
    { 
       move_base(); 
       prev_control_time = millis();
      }
    //this block stops the motor when no command is received
  
    nh.spinOnce();
}

/* ------------------------------------------------------------------------------------------------------------- */
/*                                                                                                               */
/*---------------------------------------------------------------------------------------------------------------*/
void  stopBase()
{
       req_linear_vel_x = 0;
       req_linear_vel_y = 0;
       req_angular_vel_z = 0; 
}

/* ------------------------------------------------------------------------------------------------------------- */
/*                                                                                                               */
/*---------------------------------------------------------------------------------------------------------------*/
void move_base()
{
        Kinematics::rpm req_rpm = Kinematics.expected_RPM(req_linear_vel_x, req_linear_vel_y, req_angular_vel_z);
        int  M1_RPM = M1_Wheel.getRPM(M1_Encoder.read());
        int  M2_RPM  = M2_Wheel.getRPM(M2_Encoder.read());
        
     //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
     //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
        M1_control.spin(M1_pid.compute(req_rpm.motor1, M1_RPM));
        M2_control.spin(M2_pid.compute(req_rpm.motor2, M2_RPM));
    
        Kinematics::velocities current_vel;
        current_vel = Kinematics.getVelocities(M1_RPM,M2_RPM,0,0);
        publishSpeed(current_vel);  
}

/* ------------------------------------------------------------------------------------------------------------- */
//Publish function for odometry, uses a vector type message to send the data 
//(message type is not meant for that but that's easier than creating a specific message type)
/*---------------------------------------------------------------------------------------------------------------*/
void publishSpeed( Kinematics::velocities actual_vel) {
        nh.loginfo("Publishing odometry");
        raw_vel_msg.header.stamp = nh.now();      //timestamp for odometry data
        raw_vel_msg.vector.x = actual_vel.linear_x;    //left wheel speed (in m/s)
        raw_vel_msg.vector.y = actual_vel.linear_y;   //right wheel speed (in m/s)
        raw_vel_msg.vector.z = actual_vel.angular_z;         //looptime, should be the same as specified in LOOPTIME (in s)
        raw_vel_pub.publish(&raw_vel_msg);

}
