
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

string _CurrentLabel = "none";
string _FirstDetectedLabel = "none";

uint _BaseAngle=0;
uint _Elbow=10;
uint _Shoulder=95;
uint _NumObjects=0;

bool _MovingUp=true;
bool _Searching=true;
bool _MoveToStack=false;
bool _MoveToSearchPosition=false;
bool _DropCard=false;
bool _GrabCard=false;

//updated cyclicly
void NumObjectsCallback(const std_msgs::Int8::ConstPtr& msg)
{
  if(_Searching==true)
  {
  //ROS_INFO("I heard: [%d]",msg->data);
  _NumObjects = (uint)msg->data;
}
}

//updated if new data arrives
void labelCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
  if(_Searching==true)
  {
  // ROS_INFO("I heard: [%s]", msg->boundingBoxes[0].Class.c_str());
  _CurrentLabel =  msg->boundingBoxes[0].Class.c_str();
  }
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

void grabCard(uint& Shoulder, uint& Elbow, bool& moveToStack, bool& grabCard)
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
  {
  	Shoulder=95;
  	Elbow=10;
  	_DataArray[1]=Shoulder; //Shoulder
	  _DataArray[2]=Elbow; //Elbow
	    //now Move card to stack
	  moveToStack = true;
    grabCard = false;
  }

    //eventually add entry for magnet Implementation
}

void DropCard(uint& Shoulder, uint& Elbow, bool& moveToSearchPosition, bool& movingUp, bool& dropCard)
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
    {
    	Shoulder=95;
    	Elbow=10;
    	_DataArray[1]=Shoulder; //Shoulder
 	    _DataArray[2]=Elbow; //Elbow

 	    dropCard=false;
 	    moveToSearchPosition=true;
 	    movingUp=false;
    }
    //eventually add entry for magnet Implementation
}

void MoveToSearchPosition(uint& baseAngle, bool& searching, bool& moveToSearchPosition)
{
  if(baseAngle>170)
  {
    baseAngle--;
  }
  if(baseAngle==170)
  {
    //now we listen to camera input again
    searching=true;
    moveToSearchPosition=false;
  }
}

int main(int argc, char **argv)
{

	//intialization
  _DataArray[0]=_BaseAngle ; //Base
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


    if(_NumObjects==1 && _FirstDetectedLabel == "none" && _Searching==true && _CurrentLabel!="none")
    {
      _FirstDetectedLabel=_CurrentLabel;
      _CurrentLabel="none";
      ROS_INFO("I found the first Memory Card");
      _Searching = false;
      _GrabCard = true;
    }



    if(_CurrentLabel==_FirstDetectedLabel && _CurrentLabel!="none" && _Searching==true)
    {	
      _FirstDetectedLabel="none";
      _CurrentLabel="none";
      //stop searching
      _Searching=false;
      //grab Card
      _GrabCard = true;
    }


    if(_Searching==true)
    {
      movingInCircle(_BaseAngle, _MovingUp);
      //moving up and down from 0 to 170°
    }
 
    //grab the card where it is currently above
    if(_GrabCard==true )
    {
      //grab Card and move it to stack
      grabCard(_Shoulder, _Elbow, _MoveToStack, _GrabCard);
    }

    if(_MoveToStack==true)
    {
       movingToStack(_BaseAngle, _MoveToStack, _DropCard);
       //runs to 180°
       //sets move to stack to false when arrived
       //sets drop Cart to true
    }

    if(_DropCard==true)
    {
    	DropCard(_Shoulder, _Elbow, _MoveToSearchPosition, _MovingUp, _DropCard);
    	//sets searching to true
    	//moving up to false -> move direction 0
    	//sets dropCard false
    }

    if(_MoveToSearchPosition==true)
    {
      MoveToSearchPosition(_BaseAngle, _Searching, _MoveToSearchPosition);
    }

    ROS_INFO(" ");
    ROS_INFO("NumObjects: [%d]; Current Label: [%s]; _FirstDetectedLabel: [%s]", _NumObjects, _CurrentLabel.c_str(), _FirstDetectedLabel.c_str());
    ROS_INFO("_MovingUp: [%d]; _Searching: [%d], MoveToStack: [%d], DropCard: [%d], GrabCard: [%d], MoveToSearchPosition: [%d]", _MovingUp,_Searching,_MoveToStack, _DropCard, _GrabCard, _MoveToSearchPosition);
    ROS_INFO("_BaseAngle: [%d] , _Elbow: [%d], _Shoulder: [%d]",_BaseAngle,_Elbow,_Shoulder);

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