
#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/UInt8MultiArray.h"
#include "sensor_msgs/JointState.h"

#include "std_msgs/String.h"
#include "std_msgs/Int8.h"

// darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

#include <string>
using namespace std;



/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
uint _DataArray[6];

//stores all the angles and matches with the labels of the cards
uint _mappingAngle[181];

string _currentLabel = "none";
string _firstDetectedLabel = "none";

uint _baseAngle=0;
uint _Elbow=10;
uint _Shoulder=95;

bool _movingUp=true;
bool _gripperDown=false;
bool _gripperUP=false;
bool _imageDetectedTwice=false;
bool _searching=true;
bool _moveToStack=false;
bool _dropCard=true;

void NumObjectsCallback(const std_msgs::Int8::ConstPtr& msg)
{
  ROS_INFO("I heard: [%d]",msg->data);

}

 void labelCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
 {
   ROS_INFO("I heard: [%s]", msg->boundingBoxes[0].Class.c_str());
 }

//needs the detected Label (0-10) as input 
void matchAngleCard(uint baseAngle, uint memoryLabel)
{
  _mappingAngle[baseAngle]=memoryLabel;
}

void movingInCircle(uint& baseAngle, bool& movingUp)
{  
    if(baseAngle==170)
    {
      movingUp=false;
    }

    if(baseAngle==0)
    {
      movingUp=true;
    }

    if (movingUp==true)
    {
      baseAngle++;
    }

    if (movingUp==false)
    {
      baseAngle--;
    }

    _DataArray[0]=baseAngle; //Base
}

void movingToStack(uint& baseAngle, bool& moveToStack, bool& dropCard)
{
	if(baseAngle<180)
	{
		baseAngle++;
	}

	if(baseAngle==180)
	{
		moveToStack=false;
		dropCard=true;
	}
}

void grabCard(uint& Shoulder, uint& Elbow)
{
	if(Shoulder>90)
	{
		Shoulder--;
	}
	if (Elbow>5)
	{
		Elbow--;
	}

    _DataArray[1]=Shoulder; //Shoulder
    _DataArray[2]=Elbow; //Elbow

    //move gripper into standard position
    if(Shoulder==90 && Elbow == 5)
    	Shoulder=95;
    	Elbow=10;
    	_DataArray[1]=Shoulder; //Shoulder
 	    _DataArray[2]=Elbow; //Elbow
 	    //now Move card to stack
 	    _moveToStack=true;


    //eventually add entry for magnet Implementation
}

void DropCard(uint& Shoulder, uint& Elbow, bool& searching, bool& movingUp, bool& dropCard)
{
	if(Shoulder>90)
	{
		Shoulder--;
	}
	if (Elbow>5)
	{
		Elbow--;
	}
	
    _DataArray[1]=Shoulder; //Shoulder
    _DataArray[2]=Elbow; //Elbow

    //move gripper into standard position
    if(Shoulder==90 && Elbow == 5)
    	Shoulder=95;
    	Elbow=10;
    	_DataArray[1]=Shoulder; //Shoulder
 	    _DataArray[2]=Elbow; //Elbow

 	    dropCard=false;
 	    searching=true;
 	    movingUp=false;
    //eventually add entry for magnet Implementation
}

int main(int argc, char **argv)
{

	//intialization
  _DataArray[0]=_baseAngle ; //Base
  _DataArray[1]=_Elbow; //Shoulder
  _DataArray[2]=_Shoulder; //Elbow
  _DataArray[3]=uint(10); //WristVertical
  _DataArray[4]=uint(90); //WristRotation
  _DataArray[5]=uint(73); //Gripper Closed

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "parse_and_publish");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  

   ros::Subscriber sub1 = n.subscribe("darknet_ros/found_object", 1, NumObjectsCallback);

   ros::Subscriber sub2 = n.subscribe("/darknet_ros/bounding_boxes", 1000, labelCallback);


   //darknet_ros/found_object ([std_msgs::Int8])
   //rostopic echo /darknet_ros/bounding_boxes/boundingBoxes[0]/Class

  ros::Publisher pub = n.advertise<std_msgs::UInt8MultiArray>("joint_array", 6);
  
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    std_msgs::UInt8MultiArray array;
    //Clear array
    array.data.clear();

    //search for images if not detected twice
    //is default true
    if(_searching==true)
    {
      movingInCircle(_baseAngle, _movingUp);
      //moving up and down from 0 to 170°
    }


    //grab the card where it is currently above
    if(_firstDetectedLabel!="none" )
    {

      //stop searching
      _searching=false;
      //grab Card
      grabCard(_Shoulder, _Elbow);
    }

    if(_currentLabel==_firstDetectedLabel && _currentLabel!="none")
    {	
      //stop searching
      _searching=false;
      //grab Card
      grabCard(_Shoulder, _Elbow);
    }

    if(_moveToStack=true)
    {
       movingToStack(_baseAngle, _moveToStack, _dropCard);
       //runs to 180°
       //sets move to stack to false when arrived
       //sets drop Cart to true
    }

    if(_dropCard==true)
    {
    	DropCard(_Shoulder, _Elbow, _searching, _movingUp, _dropCard);
    	//sets searching to true
    	//moving up to false -> move direction 0
    	//sets dropCard false
    }




    //for loop, pushing data in the size of the array
    for (int i = 0; i < 6; i++)
    {
    //assign array a random number between 0 and 255.
     array.data.push_back(_DataArray[i]);
    }
    pub.publish(array);

    ros::spinOnce();

    loop_rate.sleep();
  }
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  

  return 0;
}