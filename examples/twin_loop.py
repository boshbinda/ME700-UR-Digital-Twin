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

sys.path.append("..")
import logging

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation

start = time.time()
counter = 0

plt.ion()
x = []
x.append(0)
y = []
y.append(0)
a = []
a.append(0)
b = []
c = []

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

physical_connection = rtde.RTDE(PHYSICAL_HOST, ROBOT_PORT)
physical_connection.connect()

# get controller version
digital_connection.get_controller_version()
physical_connection.get_controller_version()

# >> implement controller version checksum to make sure controller implements same trajectory



# setup recipes
digital_connection.send_output_setup(state_names, state_types)
setp = digital_connection.send_input_setup(setp_names, setp_types)
watchdog = digital_connection.send_input_setup(watchdog_names, watchdog_types)

physical_connection.send_output_setup(state_names, state_types)
# setp = physical_connection.send_input_setup(setp_names, setp_types)
# watchdog = physical_connection.send_input_setup(watchdog_names, watchdog_types)



# Setpoints to move the robot to
setp1 = [-0.12, -0.43, 0.14, 0, 3.11, 0.04]
setp2 = [-0.12, -0.51, 0.21, 0, 3.11, 0.04]

# setp1 = [-0.12, -0.43, 0.14, 0, 3.11, 0.04]
# setp2 = [-1.5, -0.81, 0.21, 0, 3.11, 0.04]

# setp1 = [-0.12, -0.43, 0.14, 0, 3.11, 2.0]
# setp2 = [-0.12, -0.51, 0.21, 0, 3.11, 1.0]

setp.input_double_register_0 = 0
setp.input_double_register_1 = 0
setp.input_double_register_2 = 0
setp.input_double_register_3 = 0
setp.input_double_register_4 = 0
setp.input_double_register_5 = 0

# The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog
watchdog.input_int_register_0 = 0


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

# control loop
move_completed = True

while keep_running:
   
    # receive the current state
    digital_state = digital_connection.receive()
    physical_state = physical_connection.receive()
    
    # print(f"joint positions = {state.actual_q}")
    # print(f"currents = {state.actual_current}")
    # print(f"target_currents = {state.target_current}")
    # print(f"target_moments = {state.target_moment}")
    # print(f"target_moments = {state.target_moment[1]}")

    # current_moment_ratio = []

    # for i in range(len(digital_state.target_current)):
    #     if digital_state.actual_current[i] == 0:
    #         current_moment_ratio.append("inf")
    #     else:
    #         current_moment_ratio.append(digital_state.target_moment[i] / digital_state.actual_current[i])
   
    # print(f"torque/current ratios = {current_moment_ratio}")



    # plt.figure(1)
    # x.append(time.time()-start)
    # y.append(digital_state.actual_current[1] * 10)
    # a.append(digital_state.target_moment[1])



    # plt.plot(x,a, label = "Target Joint Torques")
    
    # plt.plot(x,y, label = "Joint Currents")
    

    
    # plt.pause(0.001)



    # print(f"voltages = {state.actual_joint_voltage}")

    if digital_state is None or physical_state is None:
        break

    # do something...
    if move_completed and digital_state.output_int_register_0 == 1:
        move_completed = False
        new_setp = setp1 if setp_to_list(setp) == setp2 else setp2
        list_to_setp(setp, new_setp)
        print("New pose = " + str(new_setp))
        print("")
        # send new setpoint
        digital_connection.send(setp)
        physical_connection.send(setp)
        watchdog.input_int_register_0 = 1

        # counter += 1
        # if counter == 2:
        #     counter = 0 
        # x = []
        # y = []
        # a = []
        # plt.clf()

    elif not move_completed and digital_state.output_int_register_0 == 0:
        print("Move to confirmed pose = " + str(digital_state.target_q))
        print("Move to confirmed pose = " + str(physical_state.target_q))
        print("")
        move_completed = True
        watchdog.input_int_register_0 = 0

    # kick watchdog
    digital_connection.send(watchdog)
    physical_connection.send(watchdog)


# plt.ioff()
# plt.show()
digital_connection.send_pause()
physical_connection.send_pause()

digital_connection.disconnect()
physical_connection.disconnect()
