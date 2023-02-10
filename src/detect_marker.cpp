/**
* \file detect_marker.cpp
* \brief Node which moves the arm of the robot to detect the ArUco markers in the environment.
* \author Sara Sgambato
* \version 1.0
* \date 04/02/2023
* 
* \details
* 
* * Subscriber to: <BR>
* /robot_assignment/camera1/image_raw topic to get the robot vision through the camera mounted on the arm of the robot
* 
* Publisher to: <BR>
* /robot_assignment/joint1_position_controller/command topic of the 1st joint of the robot's arm
* /robot_assignment/joint2_position_controller/command topic of the 2nd joint of the robot's arm
* /robot_assignment/joint3_position_controller/command topic of the 3rd joint of the robot's arm
* /list_topic topic in which all the markers' ID are published
* 
* Description:
* The node is used to detect the ArUco markers that are in the environment. The joints of the robot's arm are moved via
* the respective topic detect the 7 markers around itself, first on the floor and then on the walls, and once all markers 
* have been detected their ID are published as a list.
**/
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/aruco.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32MultiArray.h>

class DetectAruco
{
    private:
    ros::NodeHandle nh;
    ros::Publisher joint01_pub;
    ros::Publisher joint02_pub;
    ros::Publisher joint03_pub;
    ros::Subscriber camera_sub;

    aruco::MarkerDetector mDetector;
    std::vector<aruco::Marker> markers;
    
    int floor_markers = 0;
    int ceil_markers = 0;
    int current_size = 0;
    int markerSize = 0.05;
    std_msgs::Float64 msg;
    
    public:
    std::vector<int> markerID;
    ros::Publisher ID_pub;
    float current_pose = 0;

    DetectAruco()
    {
        // publishers
        joint01_pub = nh.advertise<std_msgs::Float64>("/robot_assignment/joint1_position_controller/command", 1);
        joint02_pub = nh.advertise<std_msgs::Float64>("/robot_assignment/joint2_position_controller/command", 1);
        joint03_pub = nh.advertise<std_msgs::Float64>("/robot_assignment/joint3_position_controller/command", 1);
        ID_pub = nh.advertise<std_msgs::Int32MultiArray>("/id_list", 1);
        // subscribers
        camera_sub = nh.subscribe<sensor_msgs::Image>("/robot_assignment/camera1/image_raw", 1, &DetectAruco::CameraCallback, this);

        ros::param::get("env/floor_markers", floor_markers);
        ros::param::get("env/ceil_markers", ceil_markers);

    }

    void CameraCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv::Mat inImage_;
        aruco::CameraParameters camParam_;
        camParam_ = aruco::CameraParameters();

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        inImage_ = cv_ptr->image;
    
        // clear out previous detection results
        markers.clear();

        mDetector.detect(inImage_, markers, camParam_, markerSize, false);
        
        for (std::size_t i = 0; i < markers.size(); ++i)
        {
            if(!(std::find(markerID.begin(), markerID.end(), markers.at(i).id) != markerID.end()))
            {
                markerID.push_back(markers.at(i).id);
                std::cout << "The ID of the detected marker is: " << markers.at(i).id << " ";
                std::cout << std::endl;
            }
        }
    }

    void MoveArm(float offset)
    {   
        if(markerID.size() < floor_markers)
        {
            msg.data = 0.3;
            joint02_pub.publish(msg);

            msg.data = current_pose + offset;
            current_pose = msg.data;
            joint01_pub.publish(msg);
        }
        else
        {
            msg.data = -0.3;
            joint02_pub.publish(msg);
            msg.data = -0.2;
            joint03_pub.publish(msg);

            msg.data = current_pose + offset;
            current_pose = msg.data;
            joint01_pub.publish(msg);
        }
    }

    bool FinishDetection()
    {   
        if(markerID.size() == floor_markers + ceil_markers)
        {   
            msg.data = 0.0;
            joint01_pub.publish(msg);
            joint02_pub.publish(msg);
            joint03_pub.publish(msg);
            return true;
        }
        return false;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect_marker");
    
    ros::NodeHandle nh;
    ros::Rate rate(1.5);
    DetectAruco DetectAruco;

    float offset = 0;
    
    while(!DetectAruco.FinishDetection())
    {
        offset = 0.4 + static_cast <float> (rand())/(static_cast <float> (RAND_MAX/(0.8-0.4)));
        DetectAruco.MoveArm(offset);
        ros::spinOnce();
        rate.sleep();
    }
    
    std_msgs::Int32MultiArray _IDList;
    for (int const &i: DetectAruco.markerID)
    {
        _IDList.data.push_back(i);
    }
    DetectAruco.ID_pub.publish(_IDList);
    ros::spinOnce();
    
    return 0;
}