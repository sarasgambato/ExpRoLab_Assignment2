#! /usr/bin/env python3

"""
.. module:: load_ontology
    :platform: Unix
    :synopsis: Python module to load the topological map.

.. moduleauthor:: Sara Sgambato s4648592@studenti.unige.it

This node adds all the wanted individuals in the map and their properties and it creates all the connections based on information received from the /room_info service.

Params:
    :attr:`config/recharge_room`: name and (x,y) coordinates of the recharging room
    
    :attr:`config/robot`: name of the robot

Clients:
    :attr:`armor_client`: client to communicate with the aRMOR server in order to create the ontology.

Services:
    :attr:`load_map`: service to build the ontology
"""

import rospy
import time
from armor_api.armor_client import ArmorClient
from os.path import dirname, realpath
from ExpRoLab_Assignment2.srv import RoomInformation, LoadMap, LoadMapResponse
from ExpRoLab_Assignment2.msg import RoomCoordinate

class LoadMapService():
    """
    Class to build the ontology.
    """

    def __init__(self):
        # Armor stuff
        self.armor_client = ArmorClient("armor_client", "my_ontology")
        self.path = dirname(realpath(__file__))
        # Put the path of the file.owl
        self.path = self.path + "/../../topological_map/"
        # Initializing with buffered manipulation and reasoning
        self.armor_client.utils.load_ref_from_file(self.path + "topological_map.owl", "http://bnc/exp-rob-lab/2022-23", True, "PELLET", False, False)
        self.armor_client.utils.mount_on_ref()
        self.armor_client.utils.set_log_to_terminal(True)

        # Connect to marker_server to retrieve info
        self.get_info = rospy.ServiceProxy("/room_info", RoomInformation)

        # Variables
        self.room_coordinates = []
        self.room_coordinates = RoomCoordinate()
        self.individuals = []
        self.recharge_room = rospy.get_param("config/recharge_room")
        self.robot = rospy.get_param("config/robot")

    def handle_load_map(self, request):
        """
        Function that is called every time the server /load_map receives a request. It gathers information about one room thorugh the
        service /room_info and then manipulates the ontology with the received information.
        
        Args:
            request(Int): ID of a marker
            
        Returns:
            response(RoomCoordinate)
        """

        # wait for the server to be available
        rospy.wait_for_service("/room_info")
       
        try:
            # send a request to the server to get info about the ID
            res = self.get_info(request.marker_id)

            if res.room != "No room associated with this marker ID." and res.room not in self.individuals:
                # store usefull variables
                self.individuals.append(res.room)
                self.room_coordinates.room = res.room
                self.room_coordinates.x = res.x
                self.room_coordinates.y = res.y

                # update the ontology
                self.armor_client.manipulation.add_ind_to_class(res.room, "LOCATION")
                self.armor_client.manipulation.add_dataprop_to_ind('visitedAt', res.room, 'Long', str(int(time.time())))

                # if the room is the recharging one, place the robot there
                if res.room == self.recharge_room["room"]:
                    self.armor_client.manipulation.add_objectprop_to_ind("isIn", self.robot, res.room)
                    self.armor_client.manipulation.add_ind_to_class(res.room, "CORRIDOR")

                # if the first letter is an R, add the individual to the ROOM class
                if res.room[:1] == "R":
                    self.armor_client.manipulation.add_ind_to_class(res.room, "ROOM")

                # if the first letter is a C, add the individual to the CORRIDOR class
                if res.room[:1] == "C":
                    self.armor_client.manipulation.add_ind_to_class(res.room, "CORRIDOR")

                for j in range(0, len(res.connections)):
                    # store usefull variables
                    if res.connections[j].through_door not in self.individuals:
                        self.individuals.append(res.connections[j].through_door)

                    # update the ontology
                    self.armor_client.manipulation.add_objectprop_to_ind('hasDoor', res.room, res.connections[j].through_door)


        except rospy.ServiceException as e:
            print(e)

        # Disjoint all the individuals
        self.armor_client.manipulation.disj_inds(self.individuals)
        # Apply changes
        self.armor_client.utils.apply_buffered_changes()
        self.armor_client.utils.sync_buffered_reasoner()

        return LoadMapResponse(self.room_coordinates)

if __name__== '__main__':

    # Initialize the ROS node
    rospy.init_node('load_ontology')

    # Initialize the LoadMapService class
    load_map = LoadMapService()

    service = rospy.Service('/load_map', LoadMap, load_map.handle_load_map)

    # Spin to keep the node running
    rospy.spin()
