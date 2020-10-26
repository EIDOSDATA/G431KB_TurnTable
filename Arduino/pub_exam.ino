#include <ros.h>
#include <encoder_msgs/EncoderInfo.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <util/delay.h>

#define IR_PIN A0

#define BAUDRATE      115200

#define IR_PIN        A0
#define ENCODERA_PIN  2
#define ENCODERB_PIN  3
#define AIN1_PIN      5
#define AIN2_PIN      6

#define COUNTERCLOCKWISE  0
#define CLOCKWISE         1
#define GO                128
#define SLOW              180
#define STOP              255
#define IR_THRESHOLD      850

ros::NodeHandle  nh;

std_msgs::String ctl_msg;
std_msgs::UInt16 ir_msg;
encoder_msgs::EncoderInfo enc_msg;

void controlCallback(const std_msgs::String& ctl_msg);

ros::Subscriber<std_msgs::String> ctl_sub("control_motor", controlCallback);
ros::Publisher ir_value("ir_value", &ir_msg);
ros::Publisher encoder_info("encoder_info", &enc_msg);

bool initFlag = false;
bool edgeFlag = false;
int speed = GO;
int dir = CLOCKWISE;
int encoderPos = 0;

void controlCallback(const std_msgs::String& ctl_msg)
{
  if( strcmp(ctl_msg.data, "stop")==0 )
    speed = STOP;
  else if( strcmp(ctl_msg.data, "go")==0 )
    speed = GO;
  else if( strcmp(ctl_msg.data, "slow")==0 )
    speed = SLOW;
  else if( strcmp(ctl_msg.data, "cw")==0 )
    dir = CLOCKWISE;
  else if( strcmp(ctl_msg.data, "ccw")==0 )
    dir = COUNTERCLOCKWISE;
  else if( strcmp(ctl_msg.data, "init")==0 )
    initMotor();  
  else if( strcmp(ctl_msg.data, "motion")==0 )
    makeMotion();  
}

void initMotor()
{  
  speed = GO;
  analogWrite(AIN1_PIN, speed);
  
  while(speed==GO)
  {
    int res = analogRead(IR_PIN);
    if( (res >= IR_THRESHOLD) && (initFlag==false) )
    {
      //initFlag = true;      
      speed = STOP;
      encoderPos = 0;
    }
  }

  makeMotion();  
}

void makeMotion()
{
  speed = GO;
  analogWrite(AIN1_PIN, speed);
  
  delay(210);
  
  speed = STOP;  
}

void doEncoderB()
{
  edgeFlag = true;
  encoderPos = encoderPos+1;
    
  if(encoderPos >= 2169) //2174
  {
    encoderPos = 0;
    //speed = STOP;
  }
  
  enc_msg.encoderValue = encoderPos;
  enc_msg.timeStamp = millis();
}

void setup()
{  
  //ENCODER init
  //pinMode(ENCODERA_PIN, INPUT);
  pinMode(ENCODERB_PIN, INPUT);
  //attachInterrupt(0, doEncoderA, CHANGE);
  attachInterrupt(1, doEncoderB, RISING);

  //Motor Init
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);

  //ROS init
  nh.initNode();
  nh.subscribe(ctl_sub);
  nh.advertise(ir_value);
  nh.advertise(encoder_info);

  enc_msg.header.frame_id = "encoder_link";
}

void loop()
{
  digitalWrite(AIN2_PIN, dir);
  analogWrite(AIN1_PIN, speed);

  int res = analogRead(IR_PIN);
  ir_msg.data = res;  
  ir_value.publish(&ir_msg); 
  
  if(edgeFlag == true)
  {
    encoder_info.publish(&enc_msg);
    edgeFlag = false;
  }

//  if(speed==STOP)
//    encoder_info.publish(&enc_msg);
    
//  Serial.println(encoderPos);
//  if(encoderPos >= 1560) {
//    Serial.println(encoderPos);
//    encoderPos = encoderPos%1560;
//  }
    
  nh.spinOnce();
  //delay(1);
}
