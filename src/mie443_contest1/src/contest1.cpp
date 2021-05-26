#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>

// add new header files for subscribing to /odom
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <stdio.h>
#include <cmath>

#include <chrono>

// define conversion macros
#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) * 180./M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)

float angular = 0.0;
float linear = 0.0;
float posX = 0.0, posY = 0.0, yaw = 0.0;
float margin = 0.02; // ~1 degree for margin of error

// define bumper param
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
uint8_t robotState = 1; // 1 searching for maxDist wall, 2 go towards the wall, 3 following wall and finishing

// define laser param
float minLaserDist = std::numeric_limits<float>::infinity();
float minLeftDist = std::numeric_limits<float>::infinity();
float minLeftCentreDist = std::numeric_limits<float>::infinity();
float minRightCentreDist = std::numeric_limits<float>::infinity();
float minRightDist = std::numeric_limits<float>::infinity();
int32_t nLasers = 0, desiredNLasers = 0, desiredAngle = 1;
float maxLaserDist = 0.0, refMaxDist= 0.0, maxDistAngle = 0.0;
float refMinDist = std::numeric_limits<float>::infinity();
float minDistAngle = std::numeric_limits<float>::infinity();
float wallDist[3] = {0.0, 0.0, 0.0};
//float centerDist = 0.0;
float inf = std::numeric_limits<float>::infinity();

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
	//fill with your code
    bumper[msg->bumper] = msg->state;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	//fill with your code
    minLaserDist = std::numeric_limits<float>::infinity();
    minLeftDist = std::numeric_limits<float>::infinity();
    minLeftCentreDist = std::numeric_limits<float>::infinity();
    minRightCentreDist = std::numeric_limits<float>::infinity();
    minRightDist = std::numeric_limits<float>::infinity();
    maxLaserDist = 0.0;

    nLasers = (msg->angle_max - msg->angle_min)/msg->angle_increment;
    desiredNLasers = DEG2RAD(desiredAngle)/msg->angle_increment;

    for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx){
        minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
    }
    // rightmost, middle, leftmost laser distances
    wallDist[2] = msg->ranges[0];
    wallDist[1] = msg->ranges[nLasers/2];
    wallDist[0] = msg->ranges[nLasers - 1];

    for (uint32_t laser_idx = 0; laser_idx <= 159; ++laser_idx){
        minRightDist = std::min(minRightDist, msg->ranges[laser_idx]);
    }
    for (uint32_t laser_idx = 160; laser_idx <= 319; ++laser_idx){
        minRightCentreDist = std::min(minRightCentreDist, msg->ranges[laser_idx]);
    }
    for (uint32_t laser_idx = 320; laser_idx <= 478; ++laser_idx){
        minLeftCentreDist = std::min(minLeftCentreDist, msg->ranges[laser_idx]);
    }
    for (uint32_t laser_idx = 479; laser_idx <= 639; ++laser_idx){
        minLeftDist = std::min(minLeftDist, msg->ranges[laser_idx]);
    }
}

// add odomCallback function
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    posX = msg->pose.pose.position.x; //set robot position
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation); //convert quaternion to yaw angle in radians
    // tf::getYaw(msg->pose.pose.orientation);

    // print robot position and orientation to screen
    //ROS_INFO("Position:(%f,%f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
}

void scan360(ros::Publisher vel_pub, geometry_msgs::Twist vel){
    //adjust positive - turn ccw
    //adjust negative - turn cw
    
    ros::spinOnce();
    std::chrono::time_point<std::chrono::system_clock> scanStart;
    scanStart = std::chrono::system_clock::now();
    uint64_t scanDuration = 0;
    angular = M_PI/8; // rotate for 16 seconds at this speed to perform 360 turn
    while (scanDuration < 16){
        scanDuration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - scanStart).count();
        //secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        ros::spinOnce();
        if (wallDist[1] > refMaxDist){
            maxDistAngle = yaw;
            refMaxDist = wallDist[1];
            ROS_INFO("maxDistUpdated: %f", wallDist[1]);
        }

        if (wallDist[1] < refMinDist){
            minDistAngle = yaw;
            refMinDist = wallDist[1];
            ROS_INFO("minDistUpdated: %f", wallDist[1]);
        }
    }
    angular = 0.0;
    linear = 0.0;
    vel.angular.z = angular;
    vel.linear.x = linear;
    vel_pub.publish(vel);
}

void turnToMaxDist(ros::Publisher vel_pub, geometry_msgs::Twist vel){
    // reset max distance reference for comparison
    refMaxDist = 0.0;
    // cannot call one 360 turn
    scan360(vel_pub, vel);
    ROS_INFO("maxDist: %f, maxDistAngle: %f", refMaxDist, maxDistAngle);
    angular = M_PI / 6;
    linear = 0.0;
    // turn until facing the desired angle
    while (abs(maxDistAngle - yaw) > margin && abs(maxDistAngle - yaw) < (2*M_PI - margin)){
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
        ros::spinOnce();
    }
    angular = 0.0;
    linear = 0.0;
    vel.angular.z = angular;
    vel.linear.x = linear;
    vel_pub.publish(vel);
}

void collisionHandler(ros::Publisher vel_pub, geometry_msgs::Twist vel){
    //align front bumper to wall
    //front bumper aligned to wall - back up
    std::chrono::time_point<std::chrono::system_clock> collisionStart;
    collisionStart = std::chrono::system_clock::now();
    uint64_t collisionDuration = 0;

    ROS_INFO("collision! back up! minLaserDist: %f", minLaserDist);
    while (collisionDuration < 1){
        collisionDuration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - collisionStart).count();
        angular = 0.0;
        linear = -0.1;
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
        //ROS_INFO("startTime: %i, elapsed: %i", stateStartTime, secondsElapsed);
    }
    angular = 0.0;
    linear = 0.0;
    vel.angular.z = angular;
    vel.linear.x = linear;
    vel_pub.publish(vel);

    // Re-orient the robot to be parallel (90 deg) from the min distance wall after backing up
    refMinDist = std::numeric_limits<float>::infinity();
    scan360(vel_pub, vel);
    ROS_INFO("minDist: %f, minDistAngle: %f", refMinDist, minDistAngle);
    ros::spinOnce();

    float currentYaw = yaw;
    float parallelDiff = M_PI;
    if (currentYaw > minDistAngle){
        if (currentYaw - minDistAngle > M_PI){
            // case when left wall closer
            angular = -M_PI / 6;
            linear = 0.0;
            parallelDiff = M_PI + (currentYaw - minDistAngle - (2*M_PI));
        }
        else if (currentYaw - minDistAngle < M_PI){
            // case when right wall closer
            angular = M_PI / 6;
            linear = 0.0;
            parallelDiff = M_PI - currentYaw - minDistAngle;
        }
    }

    else if (minDistAngle > currentYaw){
        if (minDistAngle - currentYaw < M_PI){
            // case when left wall closer
            angular = -M_PI / 6;
            linear = 0.0;
            parallelDiff = M_PI - minDistAngle - currentYaw;
        }
        else if (minDistAngle - currentYaw > M_PI){
            // case when right wall closer
            angular = M_PI / 6;
            linear = 0.0;
            parallelDiff = M_PI + (minDistAngle - currentYaw - (2*M_PI));
        }
    }
    float endRotateTime = (parallelDiff / angular);

    std::chrono::time_point<std::chrono::system_clock> parallelTurnStart;
    parallelTurnStart = std::chrono::system_clock::now();
    uint64_t parallelTurnDuration = 0;
    while (parallelTurnDuration < endRotateTime){
        parallelTurnDuration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - parallelTurnStart).count();
        //secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
    }
    angular = 0.0;
    linear = 0.0;
    vel.angular.z = angular;
    vel.linear.x = linear;
    vel_pub.publish(vel);
}

void leftWallFollower(ros::Publisher vel_pub, geometry_msgs::Twist vel){
    ros::spinOnce();
    float inf = std::numeric_limits<float>::infinity();
    float gap1 = 0.9;
    float gap2 = 0.8;
    if (minRightDist < 0.2){
        // narrow corridor or corner
        //ROS_INFO("gap narrow");
        gap1 = 0.6;
        gap2 = 0.5;
    }
    if (minLeftDist >= gap2 && minLeftDist <= gap1 && minLeftCentreDist > gap2){
        // maintain distance along the straight wall
        linear = 0.25;
        angular = 0.0;
    }
    else if (minLeftCentreDist <= gap2 || minLeftCentreDist == inf){
        // inf when too close, only rotate around
        linear = 0;
        angular = -M_PI/6;
    }
    // cornering inwards
    else if (minLeftDist > gap1 && minLeftCentreDist > gap2){
        linear = 0.15;
        angular = M_PI/18;
    }
    // cornering outwards
    else if ((minLeftCentreDist > gap2 && minLeftDist < gap2) || (minLeftCentreDist > gap2 && minLeftDist == inf)){
        linear = 0.15;
        angular = -M_PI/18;
    }
}

void rightWallFollower(ros::Publisher vel_pub, geometry_msgs::Twist vel){
    ros::spinOnce();
    float inf = std::numeric_limits<float>::infinity();
    float gap1 = 0.9;
    float gap2 = 0.8;
    if (minLeftDist < 0.2){
        // narrow corridor or corner
        //ROS_INFO("gap narrow");
        gap1 = 0.6;
        gap2 = 0.5;
    }
    if (minRightDist >= gap2 && minRightDist <= gap1 && minRightCentreDist > gap2){
        // maintain distance along the straight wall
        linear = 0.25;
        angular = 0.0;
    }
    else if (minRightCentreDist <= gap2 || minRightCentreDist == inf){
        // inf when too close, only rotate around
        linear = 0;
        angular = M_PI/6;
    }
    // cornering inwards
    else if (minRightDist > gap1 && minRightCentreDist > gap2){
        linear = 0.15;
        angular = -M_PI/18;
    }
    // cornering outwards
    else if ((minRightCentreDist > gap2 && minRightDist < gap2) || (minRightCentreDist > gap2 && minRightDist == inf)){
        linear = 0.15;
        angular = M_PI/18;
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    //Node subscribe to bumper/scan topic
    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    //Publish a velocity command
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    //  ros::Rate loop_rate(10);

    // add odom subscriber
    ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    while(ros::ok() && secondsElapsed <= 900) {
        // while ros is running and under 15 minutes
        // spin once = messages received from subsribed topics are processed
        ros::spinOnce();

        //ROS_INFO("Position:(%f,%f), Orientation: %f degrees, Range: %f", posX, posY, RAD2DEG(yaw), minLaserDist);
        //Check for any bumpers hit / run into wall / stuck -> turn to the right if stuck; if nothing in front then advance
        bool any_bumper_pressed=false;
	    for(uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
		    any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
	    }

        if (any_bumper_pressed){
            ROS_INFO("collision start");
            collisionHandler(vel_pub, vel);
            if (robotState == 1){
                robotState = 2;
            }
            else if (robotState == 2){
                robotState = 3;
            }
            else if (robotState == 3){
                robotState = 1;
            }
            ROS_INFO("collision end");
            //turnToMaxDist(vel_pub, vel, M_PI/2);
            //ROS_INFO("3");
            //break;
        }
        
        // First state: check surroundings, search for open space, head towards maxLaserDist
        else if (robotState == 1){
            // once finished, turn to the yaw angle that gave maxLaserDist
            turnToMaxDist(vel_pub, vel);
            // ROS_INFO("maxDistAngle %f, maxLaserDist %f", maxDistAngle, maxLaserDist);
            robotState = 2;
            ROS_INFO("state 1 complete");
        }

        // Second state: go towards the maxDist wall, if bumper pressed: collisionHandler then wall following as shown above
        else if (robotState == 2){
            //ROS_INFO("state 2 start");
            linear = 0.2;
            angular = 0.0;
        }

        // Third state: follow the wall by realigning the robot heading to be parallel to the wall (done in collisionHandler)
        else if (robotState == 3){
            if (secondsElapsed < 150 || (secondsElapsed > 300 && secondsElapsed < 450) || (secondsElapsed > 600 && secondsElapsed < 750)){
                ROS_INFO("now following left");
                leftWallFollower(vel_pub, vel);
            }
            else if ((secondsElapsed > 150 && secondsElapsed < 300) || (secondsElapsed > 450 && secondsElapsed < 600) || (secondsElapsed > 750 && secondsElapsed < 900)){
                ROS_INFO("now following right");
                rightWallFollower(vel_pub, vel);
            }
            // switch between which walls to follow to ensure it doesn't get stuck somewhere due to direction of following
        }
        // publish info to the simulated turtlebot: /teleop topic
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        //loop_rate.sleep();
    }
    ROS_INFO("Time's up!");
    return 0;
}
