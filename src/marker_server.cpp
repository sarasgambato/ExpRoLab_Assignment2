/**
* \file marker_server.cpp
* \brief Server to build the map through the markers
* \author Carmine Recchiuto
* \version 0.1
* \date 27/01/2023
*
* \details
*
* Services: <BR>
* `/room_info`
*
* Description:
* This node is a server which generates a response based on the request it receives. The request changes based 
* on the AruCo marker that the robot detects, and the response corresponds to a room with all its connections.
*/

#include <ros/ros.h>
#include <ExpRoLab_Assignment2/RoomConnection.h>
#include <ExpRoLab_Assignment2/RoomInformation.h>

/**
* \brief Callback function.
*
* \param req request from the client, the marker's ID
* \param res response of the server, information about a room

* \return true
*
* In this callback function each ID corresponds to a different room. Each time the server receives a request, it gest the ID and sends back
* the response with the room, its connections, and the doors connecting the room to the connections.
*/
bool markerCallback(ExpRoLab_Assignment2::RoomInformation::Request &req, ExpRoLab_Assignment2::RoomInformation::Response &res)
{
	ExpRoLab_Assignment2::RoomConnection conn;
	switch (req.id){
	case 11:
		res.room = "E";
		res.x = 1.5;
		res.y = 8.0;
		conn.connected_to = "C1";
		conn.through_door = "D5";
		res.connections.push_back(conn);
		conn.connected_to = "C2";
		conn.through_door = "D6";
		res.connections.push_back(conn);
		break;
	case 12: 
		res.room = "C1";
		res.x = -1.5;
		res.y = 0.0;
		conn.connected_to = "E";
		conn.through_door = "D5";
		res.connections.push_back(conn);
		conn.connected_to = "C2";
		conn.through_door = "D7";
		res.connections.push_back(conn);
		conn.connected_to = "R1";
		conn.through_door = "D1";
		res.connections.push_back(conn);
		conn.connected_to = "R2";
		conn.through_door = "D2";
		res.connections.push_back(conn);
		break;
	case 13: 
		res.room = "C2";
		res.x = 3.5;
		res.y = 0.0;
		conn.connected_to = "E";
		conn.through_door = "D6";
		res.connections.push_back(conn);
		conn.connected_to = "C1";
		conn.through_door = "D7";
		res.connections.push_back(conn);
		conn.connected_to = "R3";
		conn.through_door = "D3";
		res.connections.push_back(conn);
		conn.connected_to = "R4";
		conn.through_door = "D4";
		res.connections.push_back(conn);
		break;
	case 14: 
		res.room = "R1";
		res.x = -7.0;
		res.y = 3.0;
		conn.connected_to = "C1";
		conn.through_door = "D1";
		res.connections.push_back(conn);
		break;
	case 15: 
		res.room = "R2";
		res.x = -7.0;
		res.y = -4.0;
		conn.connected_to = "C1";
		conn.through_door = "D2";
		res.connections.push_back(conn);
		break;
	case 16: 
		res.room = "R3";
		res.x = 9.0;
		res.y = 3.0;
		conn.connected_to = "C2";
		conn.through_door = "D3";
		res.connections.push_back(conn);
		break;
	case 17: 
		res.room = "R4";
		res.x = 9.0;
		res.y = -4.0;
		conn.connected_to = "C2";
		conn.through_door = "D4";
		res.connections.push_back(conn);
		break;
	default:
		res.room = "No room associated with this marker ID.";
	}
	return true;
}	

/**
* \brief Main function.
*
* \param argc number of command line arguments
* \param argv array of command line arguments
*
* \return 0 if success, non-zero otherwise
*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "marker_server");
	ros::NodeHandle nh;
	ros::ServiceServer oracle = nh.advertiseService( "/room_info", markerCallback);
	ros::spin();
	ros::shutdown();
	return 0;
}