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

import time
from armor_api.armor_client import ArmorClient
from os.path import dirname, realpath
client = ArmorClient("armor_client", "my_ontology") 

path = dirname(realpath(__file__))
# Put the path of the file.owl
path = path + "/../../topological_map/"

# Initializing with buffered manipulation and reasoning
client.utils.load_ref_from_file(path + "topological_map.owl", "http://bnc/exp-rob-lab/2022-23", True, "PELLET", False, False)

client.utils.mount_on_ref()
client.utils.set_log_to_terminal(True)

def LoadMap():
    """
    Function used to load all the individuals with their properties in the topological map.
    
    Args:
        None
        
    Returns:
        None
    """
    
    rooms = []
    doors = []
    corridors = []

    client.manipulation.add_ind_to_class('E', 'LOCATION')
    client.manipulation.add_objectprop_to_ind('isIn', 'Robot1', 'E')
    print('Robot is in location E waiting to receive information.')

    # Ask the user how many corridors should be created
    n_corridors = input('Insert the number of corridors: ')

    while(n_corridors.isdigit() == False):
        n_corridors = input('Wrong type, please enter a number: ')
    n_corridors = int(n_corridors)

    room_index = 0
    door_index = 0
    # Add all the corridors
    for i in range(0, n_corridors):
        corridors.append('C'+str(i+1))
        client.manipulation.add_ind_to_class(corridors[i], 'LOCATION')
        print('Added corridor '+ corridors[i])
        # Ask the user how many rooms the i-th corridor has
        n_rooms_corridor = input('How many rooms does corridor ' + corridors[i] + ' have? ')
        while(n_rooms_corridor.isdigit() == False):
            n_rooms_corridor = input('Wrong type, please enter a number: ')
        # Add all the rooms and the correspondent doors
        for j in range(0, int(n_rooms_corridor)):
            rooms.append('R'+str(room_index+1))
            doors.append('D'+str(door_index+1))
            client.manipulation.add_ind_to_class(rooms[room_index], 'LOCATION')
            print('Added room ' + rooms[room_index])
            client.manipulation.add_ind_to_class(doors[door_index], 'DOOR')
            client.manipulation.add_dataprop_to_ind('visitedAt', rooms[room_index], 'Long', str(int(time.time())))
            # Connect the i-th corridor with the j-th room
            client.manipulation.add_objectprop_to_ind('hasDoor', rooms[room_index], doors[door_index])
            client.manipulation.add_objectprop_to_ind('hasDoor', corridors[i], doors[door_index])
            print('Added door ' + doors[door_index] + ' connecting corridor ' + corridors[i] + ' with room ' + rooms[room_index])
            door_index += 1
            room_index += 1
        # Create a door connecting the corridor to room E
        doors.append('D'+str(door_index+1))
        client.manipulation.add_ind_to_class(doors[door_index], 'DOOR')
        client.manipulation.add_objectprop_to_ind('hasDoor', 'E', doors[door_index])
        client.manipulation.add_objectprop_to_ind('hasDoor', corridors[i], doors[door_index])
        print('Added door ' + doors[door_index] + ' connecting corridor E with corridor ' + corridors[i])
        door_index += 1
    # Add corridor E to the list of corridors
    corridors.append('E')
    n_corridors += 1

    # Connect all the corridors with a door
    for k in range(0, n_corridors-2):
        doors.append('D'+str(door_index+1))
        client.manipulation.add_ind_to_class(doors[door_index], 'DOOR')
        client.manipulation.add_objectprop_to_ind('hasDoor', corridors[k], doors[door_index])
        client.manipulation.add_objectprop_to_ind('hasDoor', corridors[k+1], doors[door_index])
        print('Added door ' + doors[door_index] + ' connecting corridor ' + corridors [k] + ' with corridor ' + corridors[k+1])
        door_index += 1

    # Disjoint all the individuals
    inds = rooms + corridors + doors
    client.manipulation.disj_inds(inds)
     
    # Apply changes
    client.utils.apply_buffered_changes()
    client.utils.sync_buffered_reasoner()