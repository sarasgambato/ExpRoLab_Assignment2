#! /usr/bin/env python3

"""
.. module:: load_ontology
    :platform: Unix
    :synopsis: Python module to load the topological map.

.. moduleauthor:: Sara Sgambato s4648592@studenti.unige.it

This node adds all the wanted individuals in the map and their properties and it creates all the connections based on user inputs.

Clients:
    :attr:`armor_client`: client to communicate with the aRMOR server in order to create the ontology.
"""

import rospy
import time
from std_msgs import Int32MultiArray
from armor_api.armor_client import ArmorClient
from os.path import dirname, realpath
from helper import InterfaceHelper
from ExpRoLab_Assignment2.srv import RoomInformation

class LoadMap:

    def __init__(self):
        # Armor stuff
        self.armor_client = ArmorClient("assignment", "my_ontology")
        self.path = dirname(realpath(__file__))
        # Put the path of the file.owl
        self.path = self.path + "/../../topological_map/"
        # Initializing with buffered manipulation and reasoning
        self.armor_client.utils.load_ref_from_file(self.path + "topological_map.owl", "http://bnc/exp-rob-lab/2022-23", True, "PELLET", False, False)
        self.armor_client.utils.mount_on_ref()
        self.armor_client.utils.set_log_to_terminal(True)

        # Define the subscriber to the topic in which the list with the markers' ID is published
        rospy.Subscriber("/id_list", Int32MultiArray, self.marker_cb)
        # Connect to marker_server to retrieve info
        self.get_info = rospy.ServiceProxy("/room_info", RoomInformation)

        # Variables
        self.locations = []
        self.rooms_coordinates = []
        self.doors = []
        self.individuals = []

    def marker_cb(self, msg):
        self.markers_id = msg.data

        # wait for the server to be available
        rospy.wait_for_service('/room_info')

        for i in range(0, len(self.markers_id)):
            try:
                # send a request to the server to get info about the ID
                res = self.get_info(self.markers_id[i])

                if res.room is not "No room associated with this marker ID." and res.room not in self.individuals:
                    # store usefull variables
                    self.individuals.append(res.room)
                    self.rooms_coordinates.room = res.room
                    self.rooms_coordinates.x = res.x
                    self.rooms_coordinates.y = res.y

                    # update the ontology
                    self.armor_client.manipulation.add_ind_to_class(res.room, 'LOCATION')
                    self.armor_client.manipulation.add_dataprop_to_ind('visitedAt', res.room, 'Long', str(int(time.time())))

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

if __name__== '__main__':

    rospy.init_node('load_ontology')
    rospy.spin()