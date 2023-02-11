/**
* \file detect_marker.cpp
* \brief Node which moves the arm of the robot to detect the ArUco markers in the environment.
* \author Sara Sgambato
* \version 1.0
* \date 04/02/2023
* 
* \details
*
* \param [in] floor_markers number of markers on the floor.
* \param [in] ceil_markers number of markers on the top of the walls.
* 
* * Subscriber to: <BR>
* /robot_assignment/camera1/image_raw topic to get the robot vision through the camera mounted on the arm of the robot
* 
* Publisher to: <BR>
* /robot_assignment/joint1_position_controller/command topic of the 1st joint of the robot's arm
* /robot_assignment/joint2_position_controller/command topic of the 2nd joint of the robot's arm
* /robot_assignment/joint3_position_controller/command topic of the 3rd joint of the robot's arm
* /id_list topic in which all the markers' ID are published
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

/**
* \class DetectAruco
* \brief Class to detect the ArUco markers in the environment.
*
* This class subscribes to the topic relative to what the robot is seeing and it publishes the joints' positions 
* in order for the robot arm to move, and once a marker has been detected its ID is stored inside a variable.
*/
class DetectAruco
{
    private:
    ros::NodeHandle nh;                 /**< ROS node handle */
    ros::Publisher joint01_pub;         /**< ROS publisher to the 1st joint of the robot's arm */
    ros::Publisher joint02_pub;         /**< ROS publisher to the 2nd joint of the robot's arm */
    ros::Publisher joint03_pub;         /**< ROS publisher to the 3rd joint of the robot's arm */   
    ros::Subscriber camera_sub;         /**< ROS subscriber to the camera image */
    aruco::MarkerDetector mDetector;    /**< ArUco marker detector */
    std::vector<aruco::Marker> markers; /**< vector of ArUco markers */
    int floor_markers = 0;              /**< number of markers on the floor */
    int ceil_markers = 0;               /**< number of markers on the top of the walls */
    int markerSize = 0.05;              /**< size of a marker */
    std_msgs::Float64 msg;              /**< message to send the joints' positions */
    
    public:
    std_msgs::Int32MultiArray IDList;   /**< message to send the list of IDs */
    std::vector<int> markerID;          /**< vector of IDs of the markers */
    ros::Publisher ID_pub;              /**< ROS publisher to the list topic */
    float current_pose = 0;             /**< current position of the robot's 1st joint */

    /**
    * \brief Constructor of the ArUco class.
    *
    * In the constructor the publishers and the subscriber are initialized. Also, The constructor takes two parameter relative to
    * the ArUco markers in the environment, the ones on the floor and the ones on the top of the walls.
    */
    DetectAruco()
    {
        // publishers
        joint01_pub = nh.advertise<std_msgs::Float64>("/robot_assignment/joint1_position_controller/command", 1);
        joint02_pub = nh.advertise<std_msgs::Float64>("/robot_assignment/joint2_position_controller/command", 1);
        joint03_pub = nh.advertise<std_msgs::Float64>("/robot_assignment/joint3_position_controller/command", 1);
        ID_pub = nh.advertise<std_msgs::Int32MultiArray>("/id_list", 1);
        // subscribers
        camera_sub = nh.subscribe<sensor_msgs::Image>("/robot_assignment/camera1/image_raw", 1, &DetectAruco::CameraCallback, this);
        // parameters
        ros::param::get("config/floor_markers", floor_markers);
        ros::param::get("config/ceil_markers", ceil_markers);
    }

    /**
    * \brief Callback of the subscriber to the /robot_assignment/camera1/image_raw topic.
    *
    * \param msg image received
    *
    * In this callback the image is processed to see whether there are some markers. If some markers are detected, and they have not
    * been detected before, their IDs are saved into a variable.
    */
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
                IDList.data.push_back(markers.at(i).id);
                std::cout << "The ID of the detected marker is: " << markers.at(i).id << " ";
                std::cout << std::endl;
            }
        }
    }

    /**
    * \brief Member to move the robot's arm.
    *
    * \param offset float number
    *
    * \return The test results
    *
    * The robot arm is moved firstly to detect all the markers on the floor, then it is moved to detect all the markers on the walls.
    * To do so, the joint responsible of rotating around the z axis is moved every time of <offset> radians, until all markers are detected.
    */
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

    /**
    * \brief Member to check if all markers have been detected.
    *
    * \return true if all markers have been detected, false otherwise
    *
    * The size of the list containing the IDs of the detected markers is compared with the total number of markers to detect.
    * Once all markers have been detected, the robot's arm is put back to its initial position.
    */
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

/**
* \brief Main function.
*
* \param argc number of command line arguments
* \param argv array of command line arguments
*
* \return 0 if success, non-zero otherwise
*
* In the main function the robot's arm is moved until all markers have been detected. Every time the arm is moved, it stays in
* position for 0.6 seconds. Once all markers have been detected, the list with the IDs is published.
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect_marker");
    
    ros::NodeHandle nh;
    ros::Rate rate(1.5);
    DetectAruco DetectAruco;

    float offset = 0;
    
    // Continuosly move the arm until all the markers have been detected
    while(!DetectAruco.FinishDetection())
    {
        // add a random offset to the current joints' positions
        offset = 0.4 + static_cast <float> (rand())/(static_cast <float> (RAND_MAX/(0.8-0.4)));
        DetectAruco.MoveArm(offset);
        ros::spinOnce();
        rate.sleep();
    }
    DetectAruco.ID_pub.publish(DetectAruco.IDList);
    // This spin is needed to publish the message
    ros::spinOnce();
    
    return 0;
}