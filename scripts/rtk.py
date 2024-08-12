#! /usr/bin/env python3

"""
pygnssutils - rtk_example.py

*** FOR ILLUSTRATION ONLY - NOT FOR PRODUCTION USE ***

RUN FROM WITHIN /examples FOLDER:

python3 rtk_example.py

PLEASE RESPECT THE TERMS OF USE OF ANY NTRIP CASTER YOU
USE WITH THIS EXAMPLE - INAPPROPRIATE USE CAN RESULT IN
YOUR NTRIP USER ACCOUNT OR IP BEING TEMPORARILY BLOCKED.

This example illustrates how to use the UBXReader and
GNSSNTRIPClient classes to get RTCM3 or SPARTN RTK data
from a designated NTRIP caster/mountpoint and apply it
to an RTK-compatible u-blox GNSS receiver (e.g. ZED-F9P)
connected to a local serial port (USB or UART1).

GNSSNTRIPClient receives RTCM3 or SPARTN data from the NTRIP
caster and outputs it to a message queue. An example
GNSSSkeletonApp class reads data from this queue and sends
it to the receiver, while reading and parsing data from the
receiver and printing it to the terminal.

GNSSNtripClient optionally sends NMEA GGA position sentences
to the caster at a prescribed interval, using either fixed
reference coordinates or live coordinates from the receiver.

NB: Some NTRIP casters may stop sending RTK data after a while
if they're not receiving legitimate NMEA GGA position updates
from the client.

Created on 5 Jun 2022

:author: semuadmin
:copyright: SEMU Consulting © 2022
:license: BSD 3-Clause
"""

# pylint: disable=invalid-name

from sys import argv
from queue import Queue, Empty
from threading import Event
from time import sleep

from pygnssutils import VERBOSITY_LOW, GNSSNTRIPClient
from gnssapp import GNSSSkeletonApp

CONNECTED = 1

import rospy
#from csi_utils.aoa_node_main import aoa_node
import numpy as np
import os
import sys
from sensor_msgs.msg import NavSatFix

class publishGPS(object): # main(**kwargs):
    
    def __init__(self):
        rospy.loginfo("Initializing GPS node")
        # GNSS receiver serial port parameters - AMEND AS REQUIRED:
        self.SERIAL_PORT = "/dev/ttyUSB0"
        self.BAUDRATE = 38400
        self.TIMEOUT = 10
    
        # NTRIP caster parameters - AMEND AS REQUIRED:
        # Ideally, mountpoint should be <30 km from location.
        self.IPPROT = "IPv4"  # or "IPv6"
        self.NTRIP_SERVER = "rtk2go.com"
        self.NTRIP_PORT = 2101
        self.HTTPS = 0  # 0 for HTTP, 1 for HTTPS
        self.FLOWINFO = 0  # for IPv6
        self.SCOPEID = 0  # for IPv6
        self.MOUNTPOINT = "seattle-wa" #"LFPWD_HV" # leave blank to retrieve sourcetable
        self.NTRIP_USER = "paolo.a.torrado@gmail.com"
        self.NTRIP_PASSWORD = ""
        self.DATATYPE = "RTCM"  # "RTCM" or "SPARTN"
    
        # NMEA GGA sentence status - AMEND AS REQUIRED:
        self.GGAMODE = 0  # use fixed reference position (0 = use live position)
        self.GGAINT = 60  # interval in seconds (-1 = do not send NMEA GGA sentences)
        # Fixed reference coordinates (only used when GGAMODE = 1) - AMEND AS REQUIRED:
        self.REFLAT = 51.176534
        self.REFLON = -2.15453
        self.REFALT = 40.8542
        self.REFSEP = 26.1743
    
        self.recv_queue = Queue()  # data from receiver placed on this queue
        self.send_queue = Queue()  # data to receiver placed on this queue
        self.stop_event = Event()
        self.verbosity = 0  # 0 - no output, 1 - print identities, 2 - print full message

        rospy.loginfo("Initializing GPS publishing")
        self.gps_pub = rospy.Publisher('/gps', NavSatFix, queue_size=1)
        rospy.sleep(8)
        rospy.loginfo("Initialized")

    def run(self):
        try:
            rospy.loginfo("Starting GNSS reader/writer on...")
            with GNSSSkeletonApp(
                self.SERIAL_PORT,
                self.BAUDRATE,
                self.TIMEOUT,
                stopevent=self.stop_event,
                recvqueue=self.recv_queue,
                sendqueue=self.send_queue,
                verbosity=self.verbosity,
                idonly=True,
                enableubx=True,
                showstatus=True,
            ) as gna:
                gna.run()
                rospy.sleep(2)  # wait for receiver to output at least 1 navigation solution

                rospy.loginfo("Starting NTRIP client on...")
                with GNSSNTRIPClient(gna, verbosity=VERBOSITY_LOW) as gnc:
                    streaming = gnc.run(
                        ipprot=self.IPPROT,
                        server=self.NTRIP_SERVER,
                        port=self.NTRIP_PORT,
                        https=self.HTTPS,
                        flowinfo=self.FLOWINFO,
                        scopeid=self.SCOPEID,
                        mountpoint=self.MOUNTPOINT,
                        ntripuser=self.NTRIP_USER,
                        ntrippassword=self.NTRIP_PASSWORD,
                        reflat=self.REFLAT,
                        reflon=self.REFLON,
                        refalt=self.REFALT,
                        refsep=self.REFSEP,
                        ggamode=self.GGAMODE,
                        ggainterval=self.GGAINT,
                        datatype=self.DATATYPE,
                        output=self.send_queue,  # send NTRIP data to receiver
                    )

                    while (
                        streaming and not self.stop_event.is_set() and not rospy.is_shutdown()
                    ):  # run until user presses CTRL-C
                        if self.recv_queue is not None:
                            # consume any received GNSS data from queue
                            rospy.loginfo("Consuming any received GNSS data from queue")
                            try:
                                while not self.recv_queue.empty():
                                    (_, parsed_data) = self.recv_queue.get(False)
                                    #if verbosity == 1:
                                    #    print(f"GNSS>> {parsed_data.identity}")
                                    #elif verbosity == 2:
                                    print(parsed_data)
                                    
                                    if parsed_data:
                                        if hasattr(parsed_data, "lat") and hasattr(parsed_data, "lon"):
                                            gpsmsg=NavSatFix()
                                            gpsmsg.header.stamp = rospy.Time.now()
                                            gpsmsg.header.frame_id = "rtkgps"
                                            gpsmsg.latitude = parsed_data.lat
                                            gpsmsg.longitude = parsed_data.lon
                                            gpsmsg.altitude = parsed_data.height * 0.001
                                            if parsed_data.gnssFixOk:
                                              gpsmsg.status.status = 2
                                            else:
                                              gpsmsg.status.status = -1
                                            self.gps_pub.publish(gpsmsg)
	                        
                                self.recv_queue.task_done()

                            except Empty:
                                pass
                        rospy.sleep(1)
                    rospy.sleep(1)

        except KeyboardInterrupt:
            self.stop_event.set()
            rospy.loginfo("Terminated by user")


if __name__ == "__main__":

    rospy.init_node('gps_node', anonymous=True)


    #create node wrapper
    gps_publisher = publishGPS()

    gps_publisher.run()

