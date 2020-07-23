
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

//Kinematics Kinematics(MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE); 
 
//initializing all the variables
#define LOOPTIME                      100     //Looptime in millisecond
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter

unsigned long lastMilli = 0;
const double wheelbase = 0.187;  
double speed_req = 0;                         //Desired linear speed for the robot, in m/s
double angular_speed_req = 0;                 //Desired angular speed for the robot, in rad/s
double speed_req_left = 0;                    //Desired speed for left wheel in m/s
double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_req_right = 0;                   //Desired speed for right wheel in m/s
double speed_act_right = 0;                   //Actual speed for right wheel in m/s

int PWM_leftMotor = 0;                     //PWM command for left motor
int PWM_rightMotor = 0;                    //PWM command for right motor 

//function that will be called when receiving command from host
void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication
  
  speed_req = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message
  angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message
  
  speed_req_left = speed_req - angular_speed_req*(wheelbase/2);     //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  speed_req_right = speed_req + angular_speed_req*(wheelbase/2);    //Calculate the required speed for the right motor to comply with commanded linear and angular speeds
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type

//__________________________________________________________________________

void setup() {

  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  nh.advertise(speed_pub); //prepare to publish speed in ROS topic
    if (!nh.connected()){
       //led_indicator'  
       }
    else{
       //led_static ;
    }
}

//_________________________________________________________________________

void loop() {
  nh.spinOnce();
   // enter timed loop
  if((millis()-lastMilli) >= LOOPTIME)   
  {                                                                
        lastMilli = millis();
          // compute PWM value for left and right motor. 
        speed_act_left = M1_Wheel.getRPM(M1_Encoder.read());
        PWM_leftMotor = M1_pid.compute(speed_req_left, speed_act_left);  
        speed_act_right = M2_Wheel.getRPM(M2_Encoder.read());
        PWM_rightMotor = M2_pid.compute(speed_req_right, speed_act_right);    
                                                   
       
       //Stopping if too much time without command
        if ((noCommLoops >= noCommLoopMax)&& (speed_req_left == 0))
        {  M1_control.spin(0);
        }
        else {   //Going forward or backward 
          M1_control.spin(PWM_leftMotor);
        }
          
        //Stopping if too much time without command or no rignt motor rotation
        if ((noCommLoops >= noCommLoopMax) && (speed_req_right == 0))
        {  M2_control.spin(0);
        }
         //Going forward or backward
        else {                     
           M2_control.spin(PWM_rightMotor);
        }
       
        if((millis()-lastMilli) >= LOOPTIME){         //write an error if execution time of the loop in longer than the specified looptime
          Serial.println(" TOO LONG ");
        }
    
        noCommLoops++;
        if (noCommLoops == 65535){
          noCommLoops = noCommLoopMax;
        }

    
    publishSpeed(LOOPTIME);   //Publish odometry on ROS topic
  }
 }

//Publish function for odometry, uses a vector type message to send the data 
//(message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
  speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
  speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
}
