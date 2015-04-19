/*
 * rosserial Dreamster Example
 * Subscribes to cmd_vel and publishes /ultrasound
 */

#define USE_USBCON

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>

int leftIN1 = 5;
int leftIN2 = 6;
int rightIN1 = 9;
int rightIN2 = 10;
int TriggerPinA = A3;
int EchoPinA = 8;

ros::NodeHandle nh;

sensor_msgs::Range range_msg;

void messageCb( const geometry_msgs::Twist& cmd_msg) {

  if ( cmd_msg.angular.z == 0 && cmd_msg.linear.x == 0 ) {
    move_Dreamster(0, 0);
  } else {
    if ( cmd_msg.angular.z < -0 ) {  // atras
      move_Dreamster(-100, 100);
    } else if ( cmd_msg.angular.z > 0 ) { // adelante
      move_Dreamster(100, -100);
    } else if ( cmd_msg.linear.x < 0.0 ) { // derecha
      move_Dreamster(-100, -100);
    } else if ( cmd_msg.linear.x > 0.0 ) {   // izquierda
      move_Dreamster(100, 100);
    }
  }
}

ros::Subscriber<geometry_msgs::Twist> sub_vel("cmd_vel", messageCb );
ros::Publisher pub_range( "/ultrasound", &range_msg);

char frameid[] = "/ultrasound";

void setup()
{
  pinMode(13, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(8, INPUT);
  pinMode(leftIN1, OUTPUT);
  pinMode(leftIN2, OUTPUT);
  pinMode(rightIN1, OUTPUT);
  pinMode(rightIN2, OUTPUT);

  nh.initNode();
  nh.advertise(pub_range);
  nh.subscribe(sub_vel);

  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.79;  // fake
  range_msg.min_range = 0.0;
  range_msg.max_range = 3.0;
}

long range_time;

void loop()
{
  if ( millis() >= range_time ) {
    int r = 0;
    digitalWrite(TriggerPinA, LOW);
    delayMicroseconds(2);
    digitalWrite(TriggerPinA, HIGH);
    delayMicroseconds(10);
    digitalWrite(TriggerPinA, LOW);

    range_msg.range = ((pulseIn(EchoPinA, HIGH) / 2.9) / 200);
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    range_time =  millis() + 50;
  }
  nh.spinOnce();

}

void move_Dreamster(int lSpeed, int rSpeed) {
  if (lSpeed == 0) {
    digitalWrite(leftIN1, LOW);
    digitalWrite(leftIN2, LOW);

  } else if (lSpeed < 0) {
    digitalWrite(leftIN1, LOW);
    analogWrite(leftIN2, map(-lSpeed, 0, 100, 0, 255));
  } else {
    digitalWrite(leftIN2, LOW);
    analogWrite(leftIN1, map(lSpeed, 0, 100, 0, 255));
  }

  if (rSpeed == 0) {
    digitalWrite(rightIN1, LOW);
    digitalWrite(rightIN2, LOW);

  } else if (rSpeed < 0) {
    digitalWrite(rightIN1, LOW);
    analogWrite(rightIN2, map(-rSpeed, 0, 100, 0, 255));
  } else {
    digitalWrite(rightIN2, LOW);
    analogWrite(rightIN1, map(rSpeed, 0, 100, 0, 255));
  }

}
