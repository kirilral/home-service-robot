#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <math.h>

enum state { pickup, moving, dropoff };

// Store the x,y information from the odometry
float posX, posY;

void positionCallback(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
  ROS_INFO("Seq: [%d]", odomMsg->header.seq);
  posX = odomMsg->pose.pose.position.x;
  posY = odomMsg->pose.pose.position.y;
}

float euclideanDistance(float x1, float y1, float x2, float y2)
{
    return sqrt(pow(x1-x2, 2) + pow(y1-y2,2));
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odometrySub = n.subscribe("odom", 10,positionCallback);
  uint32_t shape = visualization_msgs::Marker::CUBE;
  visualization_msgs::Marker marker;

  float sourceX = -1.11564;
  float sourceY = 1.4267;

  float goalX = -4.0925; 
  float goalY = 1.4228;

  float distance;
  
  state curState = pickup;

  while(ros::ok())
  {
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    

    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    ROS_INFO("Picking up");
    if(curState==pickup)
    {
      marker.pose.position.x = sourceX;
      marker.pose.position.y = sourceY;
    
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;

      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
    
      distance = euclideanDistance(posX, posY, sourceX, sourceY);
      ROS_INFO("Distance to object: %f", distance);
      if(distance <0.04)
      {
         curState = moving;
         ROS_INFO("Reached object");
      }
      marker_pub.publish(marker);

      marker.lifetime = ros::Duration();
    } 
    else if(curState==moving)
    {
      distance = euclideanDistance(posX, posY, goalX, goalY);
      if(distance < 0.04)
        curState = dropoff;
    }
    else 
    {
      marker.pose.position.x = goalX;
      marker.pose.position.y = goalY;
    
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;

      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
    
      marker_pub.publish(marker);

      marker.lifetime = ros::Duration();


    }
    r.sleep();
  }

}

