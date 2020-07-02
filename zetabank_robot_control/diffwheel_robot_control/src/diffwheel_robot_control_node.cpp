#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>

#include <serial/serial.h>

#include <std_msgs/String.h>
#include <std_msgs/Char.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Range.h>

#include <boost/thread/thread.hpp>


//#define __DEBUG
//#define ___DEBUG

#define DEBUG_BASIC

//#define BLDC_MOTOR

//#define DEBUG_MC
//#define _DEBUG_MC
//#define _DEBUG_MC
//#define _DEBUG_MC
//#define DEBUG_OM
//#define DEBUG_IMU
//#define _DEBUG_IMU
//#define DEBUG_US

#define LEFT                                  0
#define RIGHT                                 1 

#define LINEAR                                0
#define ANGULAR                               1

#define MAX_LINEAR_VELOCITY                2.0             // m/s
#define MAX_ANGULAR_VELOCITY               2.0             // rad/s
#define LINEAR_X_MAX_VELOCITY              2.0

#define PI  3.1415926535897932384626433832795
#define MATH_RAD2DEG    57.2957795f
#define MATH_DEG2RAD    0.0174532f

#define CENTER_DIFF                        0.14905       // meter
#define WHEEL_NUM                           2
#define WHEEL_RADIUS                       0.075          // meter
//#define WHEEL_SEPARATION                   0.317          // meter
#define WHEEL_SEPARATION                  0.363          // meter

#ifdef BLDC_MOTOR
#define GEARRATIO                           (26)
#define ENCODER4CH                          (1024*4.0)
#else
#define GEARRATIO                           (49*2.0)
#define ENCODER4CH                          (256*4.0)
#endif

//#define ENCODER_MIN                         -2147483648     // raw
//#define ENCODER_MAX                         2147483648      // raw
#define VELOCITY_UNIT                       2
#define DISTORPM                             ((60.0*GEARRATIO)/(2*PI*WHEEL_RADIUS))
#define RPMTODIS                             (1.0/DISTORPM)

#define CONTROL_MOTOR_SPEED_PERIOD       (1000/50)     //50hz. 20ms
//#define CONTROL_MOTOR_SPEED_PERIOD       (1000/23)     //23hz. 43.5ms

//#define DRIVE_INFOR_PUBLISH_PERIOD       (1000/10)     //10hz. 100ms
#define IMU_PUBLISH_PERIOD               (1000/50)     //20hz. 50ms
#define SONAR_PUBLISH_PERIOD             (1000/5)      //5hz. 200ms

#define VELCNT                           (1000/(CONTROL_MOTOR_SPEED_PERIOD))
#define MAXDIFF_VELLIN                   0.5    // m/s
#define MAXDIFF_VELANG                   0.4    // m/s

#define VELERRCNT                        CONTROL_MOTOR_SPEED_PERIOD  

//#define DRIVE_INFOR_PUBLISH_PERIOD       (1000/17)     //17hz. 58.9ms
//#define DRIVE_INFOR_PUBLISH_PERIOD       (CONTROL_MOTOR_SPEED_PERIOD*2)     //25hz. 40ms
#define DRIVE_INFOR_PUBLISH_PERIOD       (1000/17)     //10hz. 100ms
//#define DRIVE_INFOR_PUBLISH_PERIOD       ((CONTROL_MOTOR_SPEED_PERIOD) + (CONTROL_MOTOR_SPEED_PERIOD/2))    //30hz. 33ms
//#define DRIVE_INFOR_PUBLISH_PERIOD       (1000/25)    //25hz. 40ms
#define ROTTODIS                             ((2.0*PI*WHEEL_RADIUS/GEARRATIO)/ENCODER4CH)
#define PULSETODIST                         ((2.0*PI*WHEEL_RADIUS)/(ENCODER4CH*GEARRATIO))
#define ST                                     0.01


#define STB                   0x0AA
#define EDB                   0x3B

#define SEND_IMUDATA          0x10
#define SEND_IMUDATA_ONCE     0x11
#define SEND_IMUDATA_CONT     0x12
#define STOP_IMUDATA_CONT     0x13
#define SEND_IMUCALIGYRO      0x14

#define SEND_USDATA           0x20
#define SEND_USDATA_ONCE      0x21
#define SEND_USDATA_CONT      0x22
#define STOP_USDATA_CONT      0x23

#define REV_TIMEOUT          10*1000

#define IMU_MINTHETA            0.001

nav_msgs::Odometry odom;
geometry_msgs::Quaternion odom_quat;
geometry_msgs::TransformStamped odom_tf;
sensor_msgs::JointState joint_states;
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg;

sensor_msgs::Range rangeMsg1;
sensor_msgs::Range rangeMsg2;
sensor_msgs::Range rangeMsg3;
sensor_msgs::Range rangeMsg4;
sensor_msgs::Range rangeMsg5;
sensor_msgs::Range rangeMsg6;
sensor_msgs::Range rangeMsg7;


union INTVAL {
  char cval[4];
  int ival;
} int_val;

union UINTVAL {
  char cval[4];
  uint32_t uival;
} uint_val;

union SHORTVAL {
  char cval[2];
  short sval;
} short_val;

union WNTVAL {
  char cval[2];
  uint16_t wval;
} word_val;

union FLOATVAL {
  char cval[4];
  float fval;
} float_val;

char startcnt = 0;
char b_StartFlag = 0;
char rsize = 0;

unsigned char SBRData[500]={0};
int USDataCnt = 0;
char WData[20] = {0};

char MCRData[500]={0};
int MCDataCnt;

float   quat[4];
int16_t gyroData[3];
int16_t accData[3];
int16_t magData[3];

uint16_t imu_getfreq = 50;
uint16_t sonar_getfreq = 5;

float goal_velocity[VELOCITY_UNIT] = {0.0, 0.0};
float goal_velocity_from_cmd[VELOCITY_UNIT] = {0.0, 0.0};
bool teleop_flag = false;
float pre_velocity[VELOCITY_UNIT] = {0.0, 0.0};
unsigned int vel_same_cnt = 0;
unsigned int vel_total_same_cnt = 0;

uint32_t tTime[5];
ros::Time current_time;
uint64_t current_offset;
float prev_wheel_velocity_cmd[2];

std_msgs::String motorctrl_str;

std_msgs::String setsensor_str;

char inData[50];

bool b_getPosVel = false;

char left_motor_dir = '+'; 
char right_motor_dir = '-'; 

float left_motor_vel = 0.0; 
float right_motor_vel = 0.0; 
float pre_left_motor_vel = 0.0;
float pre_right_motor_vel = 0.0;
float left_motor_pos = 0.0; 
float right_motor_pos = 0.0; 
float pre_left_motor_pos = 0.0; 
float pre_right_motor_pos = 0.0; 

//std::string left_motor_vel_str;
//std::string right_motor_vel_str;
char left_motor_pos_str[50] = {0};
char right_motor_pos_str[50] = {0};
char left_motor_vel_str[50] = {0}; 
char right_motor_vel_str[50] = {0}; 

float delta_sl, delta_sr, delta_s;
float delta_vl, delta_vr;
float theta = 0.0, delta_theta = 0.0;
float v = 0.0f, w = 0.0f; // v = translational velocity [m/s], w = rotational velocity [rad/s]

//float g_last_rad[WHEEL_NUM] = {0.0, 0.0};
//float last_velocity[WHEEL_NUM]  = {0.0, 0.0};
//double last_theta = 0.0;

float odom_pose[3];
float odom_vel[3];

float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};
float joint_states_eff[WHEEL_NUM] = {0.0, 0.0};

unsigned long prev_update_time = 0;

float orientation[4];

float imu_theta, imu_prevtheta;
float imu_dtheta;
float Temperature;

int getmc_res;
bool b_SendMCFirst = true;
bool b_MCRevDataOK = false;
int vellinearerrcnt = 0;
int velangerrcnt = 0;

int SBRecvCnt = 0;

float angval = 0.0f;

unsigned long time_now;
unsigned long prevtime; 
unsigned long step_time;

bool b_IMUFirst = true;


void write_callback(const std_msgs::String::ConstPtr& msg);
// Callback function prototypes
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
void imuSensorCallback(const sensor_msgs::Imu& imu_msg);
void magSensorCallback(const sensor_msgs::MagneticField& mag_msg);
void teleOPCallback(const std_msgs::Bool& bool_msg);
void resetCallback(const std_msgs::Empty& reset_msg);


float constrain(float value, float min, float max);
void MotorDriver_Enable();
void MotorDriver_Disable();
void MotorDriver_SetMode();
void MotorDriver_SetSpeed(int leftSpeed, int rightSpeed);
void init_MotorController();
bool controlMotor(float * value);
void updateGoalVelocity(void);
void check_vel_safety(float *velocity);
ros::Time rosNow();
void get_motor_position_data();
void get_motor_velocity_data();
void set_motor_position(int pos);
void reset_position();
int receive_motor_data();

//void initImuMsg();

float rpm2MPS(const char* vel, char dir);
void initOdom(void);
bool calcOdometry(double diff_time);
void updateOdometry(void);
void updateTF(geometry_msgs::TransformStamped & odom_tf);
void initJointStates(void);
void updateJointStates(void);
void publishDriveInformation(void);
void initJointStates(void);
void WheelControl(int* publish_rate);

void initImuMsg();
void receive_sboard_data();

void ParseSBData(int num) ;

serial::Serial serial_comm;

serial::Serial serial_sboard;

int main (int argc, char** argv){

    int rate_b = 1000;

    ros::init(argc, argv, "zetabank_robot_control_node");

    // spawn another thread
    boost::thread thread_b(WheelControl, &rate_b);

    thread_b.join(); 

    while(ros::ok())
    {
        ;
    }

    return 0;
}

void WheelControl(int* publish_rate)
{
    uint32_t t;
    int mc_res;

    uint8_t buff[10] = { STB, STB, STB, SEND_IMUDATA_CONT, 0};

    ros::NodeHandle node_obj;
    ros::Publisher odom_pub;
    ros::Publisher imu_pub;
    ros::Publisher mag_pub;
    ros::Publisher joint_states_pub;
    ros::Publisher read_pub;

	ros::Publisher sonar_pub1;
	ros::Publisher sonar_pub2;
	ros::Publisher sonar_pub3;
	ros::Publisher sonar_pub4;
	ros::Publisher sonar_pub5;
	ros::Publisher sonar_pub6;
	ros::Publisher sonar_pub7;

    tf::TransformBroadcaster odom_broadcaster;

    ros::Rate loop_rate(*publish_rate);
    ros::Rate rev_delay(1000);
    ros::Rate send_delay(500);

    ros::Subscriber write_sub = node_obj.subscribe("write", 1000, write_callback);
    ros::Subscriber imu_sub = node_obj.subscribe("imu", 1000, imuSensorCallback);
    ros::Subscriber mag_sub = node_obj.subscribe("imu/mag", 1000, magSensorCallback);

    ros::Subscriber cmdvel_subscriber = node_obj.subscribe("cmd_vel",10,commandVelocityCallback);
    ros::Subscriber teleop_subscriber = node_obj.subscribe("teleop",10,teleOPCallback);
    ros::Subscriber reset_subscriber = node_obj.subscribe("reset",10,resetCallback);

    imu_pub = node_obj.advertise<sensor_msgs::Imu>("imu", 50);
    mag_pub = node_obj.advertise<sensor_msgs::MagneticField>("imu/mag", 50);
    read_pub = node_obj.advertise<std_msgs::String>("read", 1000);
    odom_pub = node_obj.advertise<nav_msgs::Odometry>("odom", 50);
    joint_states_pub = node_obj.advertise<sensor_msgs::JointState>("joint_states", 50);
    
	sonar_pub1 = node_obj.advertise<sensor_msgs::Range>("sonar1", 50);
	sonar_pub2 = node_obj.advertise<sensor_msgs::Range>("sonar2", 50);
	sonar_pub3 = node_obj.advertise<sensor_msgs::Range>("sonar3", 50);
	sonar_pub4 = node_obj.advertise<sensor_msgs::Range>("sonar4", 50);
	sonar_pub5 = node_obj.advertise<sensor_msgs::Range>("sonar5", 50);
	sonar_pub6 = node_obj.advertise<sensor_msgs::Range>("sonar6", 50);
	sonar_pub7 = node_obj.advertise<sensor_msgs::Range>("sonar7", 50);

    teleop_flag = false;
    vel_total_same_cnt = 0;

    try
    {
        serial_comm.setPort("/dev/ttyUSB-MC");
        serial_comm.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serial_comm.setTimeout(to);
        serial_comm.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open motor control port ");
        return;
    }

    if(serial_comm.isOpen()){
        ROS_INFO_STREAM("Motor Control Serial Port initialized");

        //std_msgs::String test_string;
        //test_string.data = "Send test string!!!";
        //serial_comm.write(test_string.data);
    } else {
        return;
    }

    //unsigned long time_now;
    //unsigned long step_time; // dimension = [msec]
    ros::Time stamp_now;

    // 2019.07.18
    // delay until stable state of robot.
    {

#ifdef DEBUG_BASIC 
        ROS_INFO_STREAM("Holding.....");
#endif
        //ros::Duration(15, 0).sleep();
 
        tTime[0] = (uint32_t)(ros::Time::now().toNSec()/1000000UL);
        //ROS_INFO_STREAM(" start time : " << tTime[0]);

        std::string revData;
        while(1) {
            t = (uint32_t)(ros::Time::now().toNSec()/1000000UL);
            //ROS_INFO_STREAM(" cur time : "<< t );

            if((t- tTime[0])>2000) {
#ifdef DEBUG_BASIC                 
                ROS_INFO_STREAM("Run main process.....");
#endif                
                break;
            }
            
            if(serial_comm.available()) {
                revData = serial_comm.read(serial_comm.available());
            }
       
            //loop_rate.sleep();
        }
    }

    for(int i=0; i<5; i++)
        tTime[i] = 0;

    prev_wheel_velocity_cmd[0] = 0.0f;
    prev_wheel_velocity_cmd[1] = 0.0f;

    initOdom();

    initJointStates();

    init_MotorController();

    ROS_INFO_STREAM("Ready....");    
    
    USDataCnt = 0;
    MCDataCnt = 0;

    b_SendMCFirst = true;

    vellinearerrcnt = 0;
    velangerrcnt = 0;

    SBRecvCnt = 0;

    time_now = 0; 
    step_time = 0;
    prev_update_time = 0;

    b_IMUFirst = true;

    while(ros::ok()){

        t = (uint32_t)(ros::Time::now().toNSec()/1000000UL);

        current_offset = ros::Time::now().toNSec();
        current_time   = ros::Time::now();

#if 1
        if((t - tTime[0]) >= (CONTROL_MOTOR_SPEED_PERIOD)) {
#ifdef _DEBUG_MC        
            ROS_INFO_STREAM("t-tTime[0]:" << (t - tTime[0]) << " CMSP Time : " << CONTROL_MOTOR_SPEED_PERIOD);
#endif            
            updateGoalVelocity();

            //check_vel_safety(goal_velocity);

            controlMotor(goal_velocity);

            tTime[0] = t;
        }
#endif


        if((t- tTime[1]) >= (DRIVE_INFOR_PUBLISH_PERIOD)) {

#ifdef _DEBUG_MC        
            ROS_INFO_STREAM("t-tTime[1]:" << (t - tTime[1]) << "  DIPP Time : " << DRIVE_INFOR_PUBLISH_PERIOD);
#endif

            //get_motor_position_data();
            //send_delay.sleep();
            get_motor_velocity_data();
            //send_delay.sleep();

            unsigned long sttime;
            unsigned long entime; 

            sttime = ros::Time::now().toNSec();

            while(1) {
                mc_res = receive_motor_data();

                if(mc_res==1)
                    break;

                entime = ros::Time::now().toNSec();

                if((entime-sttime)>REV_TIMEOUT) {
                    mc_res = -3;
#ifdef DEBUG_MC        
            ROS_INFO_STREAM("timeout:" << entime-sttime);
#endif
                    break;
                }

                rev_delay.sleep(); 
            }
            
            //mc_res = receive_motor_data();

            //time_now  = ros::Time::now().toNSec();
            //step_time = time_now - prev_update_time; // dimension = [usec]

            if(mc_res == 1) {

                time_now = ros::Time::now().toNSec(); 
                step_time = time_now - prev_update_time;
                prev_update_time = time_now;

                stamp_now = rosNow();
                
                // calculate odometry
                calcOdometry((double)((double)step_time/1000000000UL)); // dimension = [sec]

                // odometry
                updateOdometry();

                //stamp_now = rosNow();
                odom.header.stamp = stamp_now;
                odom_pub.publish(odom);

                // odometry tf
                updateTF(odom_tf);
                odom_tf.header.stamp = stamp_now;
                odom_broadcaster.sendTransform(odom_tf);

                // joint states
                updateJointStates();
                joint_states.header.stamp = stamp_now;
                joint_states_pub.publish(joint_states); 
            }

            tTime[1] = t;

        }

        if((t- tTime[2]) >= (IMU_PUBLISH_PERIOD)) {
#ifdef _DEBUG_IMU        
            ROS_INFO_STREAM("t-tTime[2]:" << (t - tTime[2]));
#endif

            stamp_now = rosNow();

            imu_msg.header.stamp = stamp_now;
            //imu_msg.header.frame_id = "imu_link";
			imu_pub.publish(imu_msg);

            mag_msg.header.stamp    = stamp_now;
            //mag_msg.header.frame_id = "imu_link";
            mag_pub.publish(mag_msg);

            stamp_now = rosNow();


            tTime[2] = t;
        }

        // 2019.06.17
		//receive_sboard_data();

WC_REP2:

        loop_rate.sleep();
        ros::spinOnce();
    }

    MotorDriver_Disable();



#ifdef DEBUG_BASIC
    ROS_INFO_STREAM("End main process...");
#endif

}


void resetCallback(const std_msgs::Empty& reset_msg)
{

    initOdom();

#ifdef DEBUG_OM
    ROS_DEBUG("Reset Odometry");
#endif    
}

void teleOPCallback(const std_msgs::Bool& bool_msg)
{
    if(bool_msg.data == true)
    {
        teleop_flag = true;
#ifdef DEBUG_BASIC        
        ROS_DEBUG("teleop is true.");
#endif        
    } else 
    {
        teleop_flag = false;
#ifdef DEBUG_BASIC        
        ROS_DEBUG("teleop is false.");
#endif        
    }
}

void magSensorCallback(const sensor_msgs::MagneticField& mag_msg1){
    mag_msg = mag_msg1;
}

void imuSensorCallback(const sensor_msgs::Imu& imu_msg1)
{
    imu_msg = imu_msg1;
	orientation[0] = imu_msg1.orientation.w;
	orientation[1] = 0;
	orientation[2] = 0;
	orientation[3] = imu_msg1.orientation.z;

#ifdef _DEBUG_IMU        
    ROS_INFO_STREAM("imu : " << orientation[0] << "," << orientation[1] << "," << orientation[2] << "," << orientation[3]);
#endif    

    imu_theta = atan2f(orientation[1] * orientation[2] + orientation[0] * orientation[3],
                       0.5f - orientation[2] * orientation[2] - orientation[3] * orientation[3] );
    if(b_IMUFirst == true) {
        b_IMUFirst = false;
        imu_prevtheta = imu_theta;
    }
}

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
#ifdef _DEBUG_MC        
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
#endif
    serial_comm.write(msg->data);
}

float constrain(float value, float min, float max)
{
    if(value > max) return max;
    if(value < min) return min;
    if(fabs(value) < 0.001f)
        return 0.0f;
    return value;
}

/*******************************************************************************
  Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist & cmd_vel_msg)
{
    goal_velocity_from_cmd[LINEAR]  = cmd_vel_msg.linear.x;
    goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;

    goal_velocity_from_cmd[LINEAR]  = constrain(
                                      goal_velocity_from_cmd[LINEAR],
                                      (-1) * MAX_LINEAR_VELOCITY,
                                      MAX_LINEAR_VELOCITY
                                    );
    goal_velocity_from_cmd[ANGULAR] = constrain(
                                      goal_velocity_from_cmd[ANGULAR],
                                      (-1) * MAX_ANGULAR_VELOCITY,
                                      MAX_ANGULAR_VELOCITY
                                    );
#ifdef _DEBUG_MC        
	ROS_INFO_STREAM("cmd Linear Speed:" << goal_velocity_from_cmd[LINEAR]);
	ROS_INFO_STREAM("cmd Angular Speed:" << goal_velocity_from_cmd[ANGULAR]);
#endif      

            //updateGoalVelocity();

            //check_vel_safety(goal_velocity);

            //controlMotor(goal_velocity);
}

void MotorDriver_Enable()
{
    motorctrl_str.data = "PE0001!";
    //motorctrl_str.data = "PE0001;";

    if(serial_comm.isOpen()){
        serial_comm.write(motorctrl_str.data);
    }
}

void MotorDriver_Disable()
{
    motorctrl_str.data = "PD0001!";
    //motorctrl_str.data = "PD0001;";

    if(serial_comm.isOpen()){
        serial_comm.write(motorctrl_str.data);
    }
}

void MotorDriver_SetMode()
{
    motorctrl_str.data = "SM0505!";
    //motorctrl_str.data = "SM0505;";

    if(serial_comm.isOpen()){
        serial_comm.write(motorctrl_str.data);
    }
}

void MotorDriver_SetSpeed(int leftSpeed, int rightSpeed)
{
    std::stringstream ss;
    ss.str("");
//#ifdef BLDC_MOTOR
    //ss << "SV" << leftSpeed << "," << rightSpeed << "!";    // None feedback
//#else
//    ss << "SV" << -1*leftSpeed << "," << -1*rightSpeed << "!";    // None feedback
//#endif
    ss << "SV" << leftSpeed << "," << rightSpeed << ";";
    motorctrl_str.data = ss.str();
     ROS_INFO_STREAM("Speed ss.str()" << ss.str().length());

    if(serial_comm.isOpen()){
        serial_comm.write(ss.str());
//#ifdef DEBUG_MC        
        ROS_INFO_STREAM("Set Speed (" << leftSpeed << "," << rightSpeed << ")  " << ss.str());
//#endif        
    }
}
 
void init_MotorController()
{

    ros::Rate idle(2000);

    MotorDriver_Enable();

    idle.sleep();

    MotorDriver_SetMode();

    idle.sleep();

    //set_motor_position(0);
    reset_position();

    idle.sleep();

#ifdef DEBUG_BASIC
    ROS_INFO_STREAM("Initialize motor driver");
#endif    
}

bool controlMotor(float * value)
{

    float wheel_velocity_cmd[2];

    wheel_velocity_cmd[LEFT]  = value[LINEAR] + (value[ANGULAR] * WHEEL_SEPARATION / 2.0f);
    wheel_velocity_cmd[RIGHT] = value[LINEAR] - (value[ANGULAR] * WHEEL_SEPARATION / 2.0f);

    wheel_velocity_cmd[LEFT]  = constrain(wheel_velocity_cmd[LEFT], -LINEAR_X_MAX_VELOCITY, LINEAR_X_MAX_VELOCITY);
    wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT], -LINEAR_X_MAX_VELOCITY, LINEAR_X_MAX_VELOCITY);

    wheel_velocity_cmd[LEFT] = -1.0*wheel_velocity_cmd[LEFT]*DISTORPM;
    wheel_velocity_cmd[RIGHT] = 1.0*wheel_velocity_cmd[RIGHT]*DISTORPM;

    if((prev_wheel_velocity_cmd[LEFT] != wheel_velocity_cmd[LEFT]) || (prev_wheel_velocity_cmd[RIGHT] != wheel_velocity_cmd[RIGHT])) {
        MotorDriver_SetSpeed((int)wheel_velocity_cmd[LEFT], (int)wheel_velocity_cmd[RIGHT]);

        prev_wheel_velocity_cmd[LEFT] = wheel_velocity_cmd[LEFT];
        prev_wheel_velocity_cmd[RIGHT] = wheel_velocity_cmd[RIGHT];

#ifdef _DEBUG_MC        
        ROS_INFO_STREAM("controller vel:(" << wheel_velocity_cmd[LEFT] << "," << wheel_velocity_cmd[RIGHT] << ")");
#endif        
    }

    return true;
}

void updateGoalVelocity(void)
{
    // Recieve goal velocity through ros messages
    goal_velocity[LINEAR]  = goal_velocity_from_cmd[LINEAR];
    goal_velocity[ANGULAR] = goal_velocity_from_cmd[ANGULAR];

#ifdef _DEBUG_MC        
	ROS_INFO_STREAM("goal Linear Speed:" << goal_velocity[LINEAR]);
	ROS_INFO_STREAM("goal Angular Speed:" << goal_velocity[ANGULAR]);
#endif     
}

void check_vel_safety(float *velocity)
{

#if 1    
    if(fabs(pre_velocity[LINEAR]-velocity[LINEAR])>MAXDIFF_VELLIN) {
        if(vellinearerrcnt>VELERRCNT) {
            //velocity[LINEAR] = 0;
            vellinearerrcnt = 0;
            velocity[LINEAR] = pre_velocity[LINEAR];
            //MotorDriver_SetSpeed(0, 0);   

#ifdef _DEBUG_MC
            ROS_DEBUG("Error large linear velocity");    
#endif
        } else {
            vellinearerrcnt++;
        }
    }

    if(fabs(pre_velocity[ANGULAR]-velocity[ANGULAR])>MAXDIFF_VELANG) {
        if(velangerrcnt>VELERRCNT) {
            //velocity[ANGULAR] = 0;
            velangerrcnt = 0;
            velocity[ANGULAR] = pre_velocity[ANGULAR];
            MotorDriver_SetSpeed(0, 0);   

#ifdef _DEBUG_MC
            ROS_DEBUG("Error large angular velocity");    
#endif            

        } else {
            velangerrcnt++;
        }
    }
#endif


    pre_velocity[LINEAR] = velocity[LINEAR];
    pre_velocity[ANGULAR] = velocity[ANGULAR];
  
} 

ros::Time rosNow()
{
    uint32_t sec, nsec;

    
    uint64_t _micros = ros::Time::now().toNSec() - current_offset;


    sec  = (uint32_t)(_micros / 1000000) + current_time.sec;
    nsec = (uint32_t)(_micros % 1000000) + 1000 * (current_time.nsec / 1000);

    if (nsec >= 1e9) {
        sec++,
        nsec--;
    }
    return ros::Time(sec, nsec);
 }

void get_motor_position_data()
{
    std::stringstream ss;
    ss.str("");

    ss << "QP?;";
    motorctrl_str.data = ss.str();

    if(serial_comm.isOpen()) {
        serial_comm.write(motorctrl_str.data);
#ifdef _DEBUG_MC               
        ROS_INFO_STREAM("Get motor position");
#endif        
    }
 }

void get_motor_velocity_data() 
{
    std::stringstream ss;

    ss << "QV?;";
    motorctrl_str.data = ss.str();

    if(serial_comm.isOpen()) {
        serial_comm.write(ss.str());
#ifdef _DEBUG_MC                
        ROS_INFO_STREAM("Get motor velocity");
#endif        
    }    
}

void set_motor_position(int pos)
{
    std::stringstream ss;

    ss << "PA" << 5000000+pos << "," << 5000000+pos << "!";
    motorctrl_str.data = ss.str();

    if(serial_comm.isOpen()) {
        serial_comm.write(motorctrl_str.data);
#ifdef _DEBUG_MC               
        ROS_INFO_STREAM("Set motor position : " << ss.str());
#endif        
    }
 }

void reset_position() 
{
    std::stringstream ss;

    ss << "QEA55A!;";
    motorctrl_str.data = ss.str();

    if(serial_comm.isOpen()) {
        serial_comm.write(motorctrl_str.data);
#ifdef DEBUG_MC                
        ROS_INFO_STREAM("Reset motor position");
#endif        
    }    
}

#if 1
int receive_motor_data()
{
    int str_length = 0;
    int i; 
    int tot_length;

    if(serial_comm.available()) {
        std::string revData;

        revData = serial_comm.read(serial_comm.available());

        str_length = revData.length();
        
#ifdef _DEBUG_MC        
        {            
            ROS_INFO("Receive Data(S):%s  Count:%d",  revData.c_str(), str_length);
        }
#endif
        
        for(i=0; i<revData.length(); i++) {
            MCRData[MCDataCnt] = revData.at(i);
            MCDataCnt++;
        }

        if(MCDataCnt>= 20) {
            MCRData[MCDataCnt] = '\0';

#ifdef _DEBUG_MC        
            {
                std::string rd(MCRData);
                ROS_INFO("Receive Data(F):%s  Count:%d",  rd.c_str(), MCDataCnt);
                //ROS_INFO_STREAM("Receive Data(F):" << rd.c_str());
            }
#endif        

            MCDataCnt = 0;        

            if(MCRData[0]=='Q' && MCRData[1]=='V') {
                left_motor_dir = MCRData[2];
                right_motor_dir = MCRData[11];

                for(int i = 0; i < 7; i++)
                {
                    left_motor_vel_str[i] = MCRData[i+3];
                    right_motor_vel_str[i] = MCRData[i+12];
                }
                
                left_motor_vel_str[7] = '\0';
                right_motor_vel_str[7] = '\0';

#ifdef DEBUG_MC        
                ROS_INFO_STREAM("wheel_vel_str (" << left_motor_dir << left_motor_vel_str << "," << right_motor_dir << right_motor_vel_str << ")");
#endif

#if 1
                left_motor_vel = rpm2MPS(left_motor_vel_str, left_motor_dir);
                right_motor_vel = rpm2MPS(right_motor_vel_str, right_motor_dir);

#ifdef DEBUG_MC        
 
            ROS_INFO_STREAM("wheel_vel (" << -left_motor_vel << "," << right_motor_vel << ")");
#endif 

            for(int i = 0; i < 100; i++)
                MCRData[i] = 0;

            return 1;
#endif


        } else {
            return -2;
        }

    }

}
}
#endif


void initImuMsg()
{
    imu_msg.angular_velocity_covariance[0] = 0.02;
    imu_msg.angular_velocity_covariance[1] = 0;
    imu_msg.angular_velocity_covariance[2] = 0;
    imu_msg.angular_velocity_covariance[3] = 0;
    imu_msg.angular_velocity_covariance[4] = 0.02;
    imu_msg.angular_velocity_covariance[5] = 0;
    imu_msg.angular_velocity_covariance[6] = 0;
    imu_msg.angular_velocity_covariance[7] = 0;
    imu_msg.angular_velocity_covariance[8] = 0.02;

    imu_msg.linear_acceleration_covariance[0] = 0.04;
    imu_msg.linear_acceleration_covariance[1] = 0;
    imu_msg.linear_acceleration_covariance[2] = 0;
    imu_msg.linear_acceleration_covariance[3] = 0;
    imu_msg.linear_acceleration_covariance[4] = 0.04;
    imu_msg.linear_acceleration_covariance[5] = 0;
    imu_msg.linear_acceleration_covariance[6] = 0;
    imu_msg.linear_acceleration_covariance[7] = 0;
    imu_msg.linear_acceleration_covariance[8] = 0.04;

    imu_msg.orientation_covariance[0] = 0.0025;
    imu_msg.orientation_covariance[1] = 0;
    imu_msg.orientation_covariance[2] = 0;
    imu_msg.orientation_covariance[3] = 0;
    imu_msg.orientation_covariance[4] = 0.0025;
    imu_msg.orientation_covariance[5] = 0;
    imu_msg.orientation_covariance[6] = 0;
    imu_msg.orientation_covariance[7] = 0;
    imu_msg.orientation_covariance[8] = 0.0025;    

    mag_msg.magnetic_field_covariance[0] = 0.0048;
    mag_msg.magnetic_field_covariance[1] = 0;
    mag_msg.magnetic_field_covariance[2] = 0;
    mag_msg.magnetic_field_covariance[3] = 0;
    mag_msg.magnetic_field_covariance[4] = 0.0048;
    mag_msg.magnetic_field_covariance[5] = 0;
    mag_msg.magnetic_field_covariance[6] = 0;
    mag_msg.magnetic_field_covariance[7] = 0;
    mag_msg.magnetic_field_covariance[8] = 0.0048;
}



void ParseSBData(int num) 
{
    unsigned char *pData = &SBRData[num];

    switch(*pData) {
        case SEND_IMUDATA:
            float_val.cval[0] = *(pData+1);
            float_val.cval[1] = *(pData+2);
            float_val.cval[2] = *(pData+3);
            float_val.cval[3] = *(pData+4);
            imu_msg.angular_velocity.x = float_val.fval;
            float_val.cval[0] = *(pData+5);
            float_val.cval[1] = *(pData+6);
            float_val.cval[2] = *(pData+7);
            float_val.cval[3] = *(pData+8);
            imu_msg.angular_velocity.y = float_val.fval;
            float_val.cval[0] = *(pData+9);
            float_val.cval[1] = *(pData+10);
            float_val.cval[2] = *(pData+11);
            float_val.cval[3] = *(pData+12);
            imu_msg.angular_velocity.z = float_val.fval;
            float_val.cval[0] = *(pData+13);
            float_val.cval[1] = *(pData+14);
            float_val.cval[2] = *(pData+15);
            float_val.cval[3] = *(pData+16);
            imu_msg.linear_acceleration.x = float_val.fval;
            float_val.cval[0] = *(pData+17);
            float_val.cval[1] = *(pData+18);
            float_val.cval[2] = *(pData+19);
            float_val.cval[3] = *(pData+20);
            imu_msg.linear_acceleration.y = float_val.fval;
            float_val.cval[0] = *(pData+21);
            float_val.cval[1] = *(pData+22);
            float_val.cval[2] = *(pData+23);
            float_val.cval[3] = *(pData+24);
            imu_msg.linear_acceleration.z = float_val.fval;        
            float_val.cval[0] = *(pData+25);
            float_val.cval[1] = *(pData+26);
            float_val.cval[2] = *(pData+27);
            float_val.cval[3] = *(pData+28);
            mag_msg.magnetic_field.x = float_val.fval;
            float_val.cval[0] = *(pData+29);
            float_val.cval[1] = *(pData+30);
            float_val.cval[2] = *(pData+31);
            float_val.cval[3] = *(pData+32);
            mag_msg.magnetic_field.y = float_val.fval;
            float_val.cval[0] = *(pData+33);
            float_val.cval[1] = *(pData+34);
            float_val.cval[2] = *(pData+35);
            float_val.cval[3] = *(pData+36);
            mag_msg.magnetic_field.z = float_val.fval;
            float_val.cval[0] = *(pData+37);
            float_val.cval[1] = *(pData+38);
            float_val.cval[2] = *(pData+39);
            float_val.cval[3] = *(pData+40);
            imu_msg.orientation.w = float_val.fval;
            float_val.cval[0] = *(pData+41);
            float_val.cval[1] = *(pData+42);
            float_val.cval[2] = *(pData+43);
            float_val.cval[3] = *(pData+44);
            imu_msg.orientation.x = float_val.fval;
            float_val.cval[0] = *(pData+45);
            float_val.cval[1] = *(pData+46);
            float_val.cval[2] = *(pData+47);
            float_val.cval[3] = *(pData+48);
            imu_msg.orientation.y = float_val.fval;
            float_val.cval[0] = *(pData+49);
            float_val.cval[1] = *(pData+50);
            float_val.cval[2] = *(pData+51);
            float_val.cval[3] = *(pData+52);
            imu_msg.orientation.z = float_val.fval;
            float_val.cval[0] = *(pData+53);
            float_val.cval[1] = *(pData+54);
            float_val.cval[2] = *(pData+55);
            float_val.cval[3] = *(pData+56);
            Temperature = float_val.fval;

            // 2019.07.29
            imu_msg.orientation.x = 0.0f;
            imu_msg.orientation.y = 0.0f;

            // 2019.07.03
            imu_theta = atan2f(imu_msg.orientation.x * imu_msg.orientation.y + imu_msg.orientation.w * imu_msg.orientation.z,
                0.5f - imu_msg.orientation.y * imu_msg.orientation.y - imu_msg.orientation.z * imu_msg.orientation.z );

                if(b_IMUFirst==true) {
                    b_IMUFirst = false;
                    imu_prevtheta = imu_theta;
                }

#ifdef DEBUG_IMU        
            //ROS_INFO_STREAM("imu gyro(" << imu_msg.angular_velocity.x << "," << imu_msg.angular_velocity.y << "," << imu_msg.angular_velocity.z << ")");
            //ROS_INFO_STREAM("imu acc(" << imu_msg.linear_acceleration.x << "," << imu_msg.linear_acceleration.y << "," << imu_msg.linear_acceleration.z << ")");
            //ROS_INFO_STREAM("imu meg(" << mag_msg.magnetic_field.x << "," << mag_msg.magnetic_field.y << "," << mag_msg.magnetic_field.z << ")");
            //OS_INFO_STREAM("imu orient(" << imu_msg.orientation.w << "," << imu_msg.orientation.x << "," << imu_msg.orientation.y << "," << imu_msg.orientation.z << ")");
            ROS_INFO_STREAM("imu theta(" << imu_theta << ")");
            //ROS_INFO_STREAM("imu temp(" << Temperature << ")");
#endif

            /*imu_msg.header.stamp = rosNow();
            imu_pub.publish(imu_msg);*/

            for(int i=0; i<500; i++) {
                SBRData[i] = 0;
            }

            SBRecvCnt = 0;

            break;
        
        case SEND_USDATA:
            float_val.cval[0] = *(pData+1);
            float_val.cval[1] = *(pData+2);
            float_val.cval[2] = *(pData+3);
            float_val.cval[3] = *(pData+4);
            rangeMsg1.range = float_val.fval;			
            //rangeMsg1.header.stamp = rosNow();
            //sonar_pub1.publish(&rangeMsg1);
            
            float_val.cval[0] = *(pData+5);
            float_val.cval[1] = *(pData+6);
            float_val.cval[2] = *(pData+7);
            float_val.cval[3] = *(pData+8);
            rangeMsg2.range = float_val.fval;			
            //rangeMsg2.header.stamp = rosNow();
            //sonar_pub2.publish(&rangeMsg2);
            
            float_val.cval[0] = *(pData+9);
            float_val.cval[1] = *(pData+10);
            float_val.cval[2] = *(pData+11);
            float_val.cval[3] = *(pData+12);
            rangeMsg3.range = float_val.fval;			
            //rangeMsg3.header.stamp = rosNow();
            //sonar_pub3.publish(&rangeMsg3);
            
            float_val.cval[0] = *(pData+13);
            float_val.cval[1] = *(pData+14);
            float_val.cval[2] = *(pData+15);
            float_val.cval[3] = *(pData+16);
            rangeMsg4.range = float_val.fval;			
            //rangeMsg4.header.stamp = rosNow();
            //sonar_pub4.publish(&rangeMsg4);
            
            float_val.cval[0] = *(pData+17);
            float_val.cval[1] = *(pData+18);
            float_val.cval[2] = *(pData+19);
            float_val.cval[3] = *(pData+20);
            rangeMsg5.range = float_val.fval;			
            //rangeMsg5.header.stamp = rosNow();
            //sonar_pub5.publish(&rangeMsg5);
            
            float_val.cval[0] = *(pData+21);
            float_val.cval[1] = *(pData+22);
            float_val.cval[2] = *(pData+23);
            float_val.cval[3] = *(pData+24);
            rangeMsg6.range = float_val.fval;			
            //rangeMsg6.header.stamp = rosNow();
            //sonar_pub6.publish(&rangeMsg6);
            
            float_val.cval[0] = *(pData+25);
            float_val.cval[1] = *(pData+26);
            float_val.cval[2] = *(pData+27);
            float_val.cval[3] = *(pData+28);
            rangeMsg7.range = float_val.fval;			
            //rangeMsg7.header.stamp = rosNow();
            //sonar_pub7.publish(&rangeMsg7);
            
#ifdef _DEBUG_US        
            ROS_INFO_STREAM("sonar(" << rangeMsg1.range << "," << rangeMsg2.range << "," << rangeMsg3.range \
                                    << rangeMsg4.range << ","  << rangeMsg5.range << "," << rangeMsg6.range  \
                                    << rangeMsg7.range << ")");
#endif

            for(int i=0; i<500; i++) {
                SBRData[i] = 0;
            }
            
            SBRecvCnt = 0;

            break;
    }
}


float rpm2MPS(const char* vel, char dir) 
{  
    // Conversion from RPM composed of bits to m/s
    float vel_;

    vel_= atof(vel);
#ifdef _DEBUG_MC 
     ROS_INFO_STREAM("rmp2MPS vel:" << vel_);
     ROS_INFO_STREAM("RPMTODIS:" << RPMTODIS);
#endif

    if(dir == '+')
        return (float)(vel_ * RPMTODIS);
    else if(dir == '-')
        return (float)(-vel_ * RPMTODIS);
    else 
        return 0.0f;

}

void initOdom(void)
{
    //init_encoder = true;

    b_IMUFirst = true;

    imu_theta = 0.0f;
    imu_prevtheta = 0.0f;
    imu_dtheta = 0.0f;

    for (int index = 0; index < 3; index++)
    {
        odom_pose[index] = 0.0;
        odom_vel[index]  = 0.0;
    }

    //odom_pose[0]  = -1.0f*CENTER_DIFF*cos(odom_pose[2] + (imu_dtheta / 2.0));
    //odom_pose[1]  = -1.0f*CENTER_DIFF*sin(odom_pose[2] + (imu_dtheta / 2.0));


    odom_quat.x = 0.0;
    odom_quat.y = 0.0;
    odom_quat.z = 0.0;

    odom.pose.pose.position.x    = 0.0;
    odom.pose.pose.position.y    = 0.0;
    odom.pose.pose.position.z    = 0.0;

    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = 0.0;
    odom.pose.pose.orientation.w = 0.0;

    odom.twist.twist.linear.x    = 0.0;
    odom.twist.twist.angular.z   = 0.0;
}

bool calcOdometry(double diff_time)
{
 
    if (diff_time == 0)
        return false;

    //wheel_vel_l = -left_motor_vel;
    //wheel_vel_r = right_motor_vel;

#if 0
    delta_sl = -left_motor_pos - pre_left_motor_pos;
    delta_sr = right_motor_pos - pre_right_motor_pos;

    delta_theta = (delta_sr - delta_sl)/WHEEL_SEPARATION;

    theta += delta_theta;
#endif

	// 2019.06.22
#if 0    
	theta = atan2f(imu_msg.orientation.x * imu_msg.orientation.y + imu_msg.orientation.w * imu_msg.orientation.z,
		0.5f - imu_msg.orientation.y * imu_msg.orientation.y - imu_msg.orientation.z * imu_msg.orientation.z );

#ifdef DEBUG_OM        
    ROS_INFO_STREAM("theta:" << theta);
#endif
#endif

#if 0
    // 2019.07.24
    left_motor_vel = (-left_motor_pos - pre_left_motor_pos) / diff_time;
    right_motor_vel = (right_motor_pos - pre_right_motor_pos) / diff_time;


    pre_left_motor_pos = -left_motor_pos;
    pre_right_motor_pos = right_motor_pos;

#ifdef DEBUG_OM        
    ROS_INFO_STREAM("calOdom diff time:" << diff_time);
#endif
#ifdef DEBUG_OM        
    ROS_INFO_STREAM("cal vel(" << left_motor_vel << "," << right_motor_vel << ")");
#endif
    //delta_vl = -left_motor_vel - pre_left_motor_vel;
    //delat_vr = right_motor_vel - pre_right_motor_vel;
#endif

    v = (-left_motor_vel + right_motor_vel) / 2.0;
    w = (right_motor_vel - (-1.0*left_motor_vel))/ WHEEL_SEPARATION;
    //w = (-left_motor_vel - right_motor_vel)/ WHEEL_SEPARATION;

    pre_left_motor_vel  = -left_motor_vel;
    pre_right_motor_vel = right_motor_vel;

    delta_s = (diff_time * v) ;

    //delta_theta = w - odom_pose[2];

#ifdef _DEBUG_OM        
    ROS_INFO_STREAM("odometry(v,w):(" << v << "," << w << ")");
    //ROS_INFO_STREAM("odometry(ds, dth):(" << delta_s << "," << delta_theta << ")");
#endif    
    // must apply delta_theta to imu sensor value
    //delta_theta = theta - last_theta;

    // New algorithm
    imu_dtheta = imu_theta - imu_prevtheta;

    //if(fabs(imu_theta) > 

    //odom_pose[2]  += imu_dtheta;
    //if(fabs(imu_dtheta)>IMU_MINTHETA)
    if(fabs(imu_dtheta)>0.0001)   
        odom_pose[2]  += imu_dtheta;

#ifdef _DEBUG_OM        
    ROS_INFO_STREAM("odometry(ds, th, dth):(" << delta_s << "," << imu_theta << "," << imu_dtheta << ")");
#endif    


    angval = MATH_RAD2DEG*odom_pose[2];

#ifdef _DEBUG_OM        
    ROS_INFO_STREAM("odom_pos2:" << odom_pose[2] << "  imu_dtheta:" << imu_dtheta);
#endif
#ifdef DEBUG_OM            
    ROS_INFO_STREAM("heading angle:(" << angval << ")");
#endif        


    // compute odometric pose
    odom_pose[0]  += delta_s * cos(odom_pose[2] + (imu_dtheta / 2.0));
    odom_pose[1]  += delta_s * sin(odom_pose[2] + (imu_dtheta / 2.0));

    //odom_pose[0]  += (delta_s * cos(odom_pose[2] + (delta_theta / 2.0)) - CENTER_DIFF*cos(odom_pose[2] + (imu_dtheta / 2.0)) );
    //odom_pose[1]  += (delta_s * sin(odom_pose[2] + (delta_theta / 2.0))  - CENTER_DIFF*sin(odom_pose[2] + (imu_dtheta / 2.0)) );

    //odom_pose[0]  += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
    //odom_pose[1]  += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));

    // OK...
    //odom_pose[0]  += (delta_s * cos(odom_pose[2] + (imu_dtheta / 2.0)) - CENTER_DIFF*sin(odom_pose[2] + (imu_dtheta / 2.0)) );
    //odom_pose[1]  += (delta_s * sin(odom_pose[2] + (imu_dtheta / 2.0))  + CENTER_DIFF*cos(odom_pose[2] + (imu_dtheta / 2.0)) );

    //odom_pose[0]  += (delta_s * cos(odom_pose[2] + (delta_theta / 2.0)) - CENTER_DIFF*sin(odom_pose[2] + (delta_theta / 2.0)) );
    //odom_pose[1]  += (delta_s * sin(odom_pose[2] + (delta_theta / 2.0))  + CENTER_DIFF*cos(odom_pose[2] + (delta_theta / 2.0)) );

    //odom_pose[0]  += (delta_s * cos(odom_pose[2] + (delta_theta / 2.0)) + CENTER_DIFF*cos(odom_pose[2] + (delta_theta / 2.0)) );
    //odom_pose[1]  += (delta_s * sin(odom_pose[2] + (delta_theta / 2.0))  + CENTER_DIFF*sin(odom_pose[2] + (delta_theta / 2.0)) );



    //odom_pose[2]  += delta_theta;

    // compute odometric instantaneouse velocity
    odom_vel[0] = v;
    odom_vel[1] = 0.0;
    odom_vel[2] = w;

    //last_theta = theta;

#ifdef DEBUG_OM        
    ROS_INFO_STREAM("odom_pos (" << odom_pose[0] << "," << odom_pose[1] << "," << odom_pose[2] << ")");
    //ROS_INFO_STREAM("odom_vel:(" << odom_vel[0] << "," << odom_vel[1] << "," << odom_vel[2] << ")");
  
    //ROS_INFO_STREAM("imu dtheta:(" << imu_dtheta << ")");
    //ROS_INFO_STREAM("encoder dtheta:(" << delta_theta << ")");

    //ROS_INFO_STREAM("imu theta (" << imu_theta << ")");
    //ROS_INFO_STREAM("encoder theta (" << theta << ")");
#endif

    imu_prevtheta = imu_theta;

    return true;
}

void updateOdometry(void)
{
    odom.header.frame_id       = "odom";
    odom.child_frame_id        = "base_link";

    odom.pose.pose.position.x  = odom_pose[0];
    odom.pose.pose.position.y  = odom_pose[1];
    odom.pose.pose.position.z  = 0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_pose[2]);

    odom.twist.twist.linear.x  = odom_vel[0];
    odom.twist.twist.linear.y  = odom_vel[1];
    odom.twist.twist.angular.z = odom_vel[2];
}

void updateTF(geometry_msgs::TransformStamped & odom_tf)
{
    odom_tf.header = odom.header;
    odom_tf.child_frame_id = "base_footprint";
    odom_tf.transform.translation.x = odom.pose.pose.position.x;
    odom_tf.transform.translation.y = odom.pose.pose.position.y;
    odom_tf.transform.translation.z = odom.pose.pose.position.z;
    odom_tf.transform.rotation = odom.pose.pose.orientation;
}

void initJointStates(void)
{
     joint_states.header.frame_id = "base_link";
 
    joint_states.name.push_back("wheel_left_joint");
    joint_states.name.push_back("wheel_right_joint");

    unsigned int n = joint_states.name.size();
    joint_states.position.resize(n);
    joint_states.velocity.resize(n);
    joint_states.effort.resize(n);
}

void updateJointStates(void)
{
    joint_states_pos[LEFT] = pre_left_motor_pos;
    joint_states_pos[RIGHT] = pre_right_motor_pos;

    joint_states_vel[LEFT] = pre_left_motor_vel;
    joint_states_vel[RIGHT] = pre_right_motor_vel;

    for(int i=0; i<WHEEL_NUM; i++) {
        joint_states.position[i] = joint_states_pos[i];
        joint_states.velocity[i] = joint_states_vel[i];
        joint_states.effort[i] = joint_states_eff[i];
    }
}