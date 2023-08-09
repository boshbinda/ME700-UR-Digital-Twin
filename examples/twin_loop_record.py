#!/usr/bin/env python
# Copyright (c) 2016-2022, Universal Robots A/S,
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Universal Robots A/S nor the names of its
#      contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL UNIVERSAL ROBOTS A/S BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import sys
import csv
import datetime
import os
os.chdir("..")
os.chdir("logs")
print(os.getcwd())

time = datetime.datetime.now()
try:
    os.mkdir(f"{time.date()}-{time.hour}:{time.minute}{time.strftime('%p')}")
except:
    pass

os.chdir("..")
os.chdir("examples")

sim_joint_positions = []
sim_joint_currents = []
real_joint_positions = []
real_joint_currents = []
move_counter = 0


sys.path.append("..")
import logging

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

# import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation



plt.ion()
dig_time = []
dig_time.append(0)
phys_data = []
phys_data.append(0)
phys_time = []
phys_time.append(0)
dig_data = []
dig_data.append(0)

ax = plt.subplot()




# logging.basicConfig(level=logging.INFO)

DIGITAL_HOST = "localhost"
PHYSICAL_HOST = "192.168.12.100"
ROBOT_PORT = 30004
config_filename = "control_loop_configuration.xml"

keep_running = True

logging.getLogger().setLevel(logging.INFO)

conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe("state")
setp_names, setp_types = conf.get_recipe("setp")
watchdog_names, watchdog_types = conf.get_recipe("watchdog")

digital_connection = rtde.RTDE(DIGITAL_HOST, ROBOT_PORT)
digital_connection.connect()
print("digital twin connected")

physical_connection = rtde.RTDE(PHYSICAL_HOST, ROBOT_PORT)
physical_connection.connect()
print("physical twin connected")

# get controller version
digital_connection.get_controller_version()
physical_connection.get_controller_version()

# >> implement controller version checksum to make sure controller implements same trajectory



# setup recipes
digital_connection.send_output_setup(state_names, state_types)
digital_setpoint = digital_connection.send_input_setup(setp_names, setp_types)
digital_watchdog = digital_connection.send_input_setup(watchdog_names, watchdog_types)

physical_connection.send_output_setup(state_names, state_types)
physical_setpoint = physical_connection.send_input_setup(setp_names, setp_types)
physical_watchdog = physical_connection.send_input_setup(watchdog_names, watchdog_types)



# Setpoints to move the robot to
setp1 = [-0.12, -0.43, 0.14, 0, 3.11, 0.04]
setp2 = [-0.12, -0.51, 0.21, 0, 3.11, 0.04]


#intialize input registers
digital_setpoint.input_double_register_0 = 0
digital_setpoint.input_double_register_1 = 0
digital_setpoint.input_double_register_2 = 0
digital_setpoint.input_double_register_3 = 0
digital_setpoint.input_double_register_4 = 0
digital_setpoint.input_double_register_5 = 0

physical_setpoint.input_double_register_0 = 0
physical_setpoint.input_double_register_1 = 0
physical_setpoint.input_double_register_2 = 0
physical_setpoint.input_double_register_3 = 0
physical_setpoint.input_double_register_4 = 0
physical_setpoint.input_double_register_5 = 0

# The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz digital_watchdog
digital_watchdog.input_int_register_0 = 0
physical_watchdog.input_int_register_0 = 0


def setp_to_list(sp):
    sp_list = []
    for i in range(0, 6):
        sp_list.append(sp.__dict__["input_double_register_%i" % i])
    return sp_list


def list_to_setp(sp, list):
    for i in range(0, 6):
        sp.__dict__["input_double_register_%i" % i] = list[i]
    return sp


# start data synchronization
if not digital_connection.send_start():
    sys.exit()

if not physical_connection.send_start():
    sys.exit()

# control loop
move_completed = True

#initialize start times separately as simulation time does not reflect real time.
digital_state = digital_connection.receive()
physical_state = physical_connection.receive()
digital_start = digital_state.timestamp
physical_start = physical_state.timestamp


while keep_running:

    # receive the current state
    digital_state = digital_connection.receive()
    physical_state = physical_connection.receive()

    ##################### DATA LOGGING ###############################

    real_joint_positions.append(digital_state.actual_q)
    real_joint_currents.append(digital_state.actual_current)

    sim_joint_positions.append(physical_state.actual_q)
    sim_joint_currents.append(physical_state.actual_current)

    ###############3########## LIVE DATA PLOTTING LOOP ########################################
    plt.figure(1)

    #find current time step separately
    dig_time.append(digital_state.timestamp - digital_start)
    phys_time.append(physical_state.timestamp - physical_start)

    #find data to plot
    dig_data.append(digital_state.actual_qd[2])
    phys_data.append(physical_state.actual_qd[2])

    #plot data
    plt.plot(dig_time, dig_data, label = "sim")
    plt.plot(phys_time, phys_data, label = "real")

    #update plot graphic
    plt.pause(0.001)
    ###########################################################################################

    if digital_state is None or physical_state is None:
        break

    # move sequence between waypoints
    if move_completed and digital_state.output_int_register_0 == 1 and physical_state.output_int_register_0 == 1:
        move_completed = False
        move_counter += 1

       ############ WRITE DATA OUTPUT BETWEEN WAYPOINTS TO CSV ##############################
        
        os.chdir("..")
        os.chdir("logs")
        os.chdir(f"{time.date()}-{time.hour}:{time.minute}{time.strftime('%p')}")

        log_filename = f"movement_{move_counter}.csv"

        log_data = zip(dig_time, sim_joint_positions, sim_joint_currents, phys_time, real_joint_positions, real_joint_currents)
    
        with open(log_filename, 'w') as log:
            log_writer = csv.writer(log)
            # log_writer.writerow() #log column headers
            log_writer.writerows(log_data)

        #check size of data output arays because they don't necessarily have to be the same size although im pretty sure they are.
        sim_joint_positions = []
        sim_joint_currents = []

        real_joint_positions = []
        real_joint_currents = []

        os.chdir("..")
        os.chdir("..")
        os.chdir("examples")


        ######################################################################################

        #generate new setpoint
        new_setp = setp1 if setp_to_list(digital_setpoint) == setp2 else setp2
        list_to_setp(digital_setpoint, new_setp)
        print("New pose = " + str(new_setp))
        print("")


        # send new setpoint
        digital_connection.send(digital_setpoint)
        physical_connection.send(digital_setpoint)
    
        digital_watchdog.input_int_register_0 = 1
        physical_watchdog.input_int_register_0 = 1

        #clear plot data and time axes
        digital_start = digital_state.timestamp
        physical_start = physical_state.timestamp
        dig_time = []
        phys_data = []
        dig_data = []
        phys_time = []
        plt.clf()

    elif not move_completed and digital_state.output_int_register_0 == 0 and physical_state.output_int_register_0 == 0:
        print("Move to confirmed pose = " + str(digital_state.target_q))
        print("Move to confirmed pose = " + str(physical_state.target_q))
        print("")
        move_completed = True
        digital_watchdog.input_int_register_0 = 0
        physical_watchdog.input_int_register_0 = 0

    # kick digital_watchdog
    digital_connection.send(digital_watchdog)
    physical_connection.send(digital_watchdog)


#shutdown sequence
plt.ioff()
plt.show()
digital_connection.send_pause()
physical_connection.send_pause()

digital_connection.disconnect()
physical_connection.disconnect()
