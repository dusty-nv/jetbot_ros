#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import asyncio
import pygazebo

from pygazebo.msg import poses_stamped_pb2
from pygazebo.msg import diagnostics_pb2


'''
gz topic -l
'''

class GazeboMessageSubscriber: 

    def __init__(self, host, port, timeout=30):
        self.host = host
        self.port = port
        self.loop = asyncio.get_event_loop()
        self.running = False
        self.timeout = timeout

    async def connect(self):
        connected = False
        for i in range(self.timeout):
            try:
                print('awaiting gazebo connection')
                self.manager = await pygazebo.connect((self.host, self.port))
                print('namespaces')
                print(self.manager.namespaces())
                print('publications')
                print(self.manager.publications())
                connected = True
                break
            except Exception as e:
                print(e)
                pass
            await asyncio.sleep(1)

        if not connected: 
            raise Exception("Timeout connecting to Gazebo.")
            
        print('subscribing to topic')
        #self.poses_subscriber = self.manager.subscribe('/gazebo/default/pose/info', 'gazebo.msgs.PosesStamped', self.poses_callback)
        #self.subscriber = self.manager.subscribe('/gazebo/default/log/status', 'gazebo.msgs.LogStatus', self.status_callback)
        self.user_cmd_stats_subscriber = self.manager.subscribe('/gazebo/default/user_cmd_stats', 'gazebo.msgs.UserCmdStats', self.user_cmd_stats_callback)
        self.subscriber = self.manager.subscribe('/gazebo/default/diagnostics', 'gazebo.msgs.Diagnostics', self.diagnostics_callback)
        print('waiting for subscriber connection')
        
        #await self.poses_subscriber.wait_for_connection()
        await self.user_cmd_stats_subscriber.wait_for_connection()
        await self.subscriber.wait_for_connection()
        
        #  gz joint -m 'simple_diff' -j right_wheel_hinge --vel-t 0
        
        # run loop
        self.running = True
        
        while self.running:
            await asyncio.sleep(0.1)

    def user_cmd_stats_callback(self, data):
        print('user_cmd_stats_callback')
        
    def diagnostics_callback(self, data):
        print('diagnostics_callback')
        diagnostics_msg = diagnostics_pb2.Diagnostics()
        diagnostics_msg.ParseFromString(data)
        print(diagnostics_msg)
        
    def status_callback(self, data):
        print('status_callback')
        print(data)
        
    def poses_callback(self, data):
        print('poses_callback')
        self.poses_stamped = pygazebo.msg.v9.poses_stamped_pb2.PosesStamped()
        self.poses_stamped.ParseFromString(data)
        
 
subscriber = GazeboMessageSubscriber('localhost', 11346) 

loop = asyncio.get_event_loop()
loop.run_until_complete(subscriber.connect())
    