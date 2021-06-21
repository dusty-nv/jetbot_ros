#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import asyncio
import pygazebo

from pygazebo.msg import poses_stamped_pb2
from pygazebo.msg import diagnostics_pb2


'''
gz topic -l
gz joint -m 'simple_diff' -j right_wheel_hinge --vel-t 0
'''

parser = argparse.ArgumentParser()

parser.add_argument('--host', default='localhost', type=str)
parser.add_argument('--port', default=11346, type=int)
parser.add_argument('--retry', default=30, type=int)

args = parser.parse_args()
print(args)


def gazebo_connect(host='localhost', port=11346, retry=30):
    async def _connect(host, port, retry):
        connected = False
        for i in range(retry):
            try:
                print(f'waiting for gazebo connection {host}:{port} (attempt={i+1})')
                manager = await pygazebo.connect((host, port))
                connected = True
                print(f'connected to gazebo server {host}:{port}')
                break
            except Exception as e:
                print(e)
                pass
            await asyncio.sleep(1)

        if not connected: 
            raise Exception("Timeout connecting to Gazebo.")
            
        return manager

    return asyncio.get_event_loop().run_until_complete(
            _connect(host, port, retry))

def gazebo_advertise(manager, topic_name, msg_type):
    async def _advertise(manager, topic_name, msg_type):
        return await manager.advertise(topic_name, msg_type)
        
    return asyncio.get_event_loop().run_until_complete(
        _advertise(manager, topic_name, msg_type))
        
def gazebo_subscribe(manager, topic_name, msg_type, callback):
    async def _subscribe(manager, topic_name, msg_type, callback):
        subscriber = manager.subscribe(topic_name, msg_type, callback)
        await subscriber.wait_for_connection()
        return subscriber
    
    return asyncio.get_event_loop().run_until_complete(
            _subscribe(manager, topic_name, msg_type, callback))

# connect to gazebo server    
manager = gazebo_connect(args.host, args.port, args.retry)

print('namespaces')
print('  ', manager.namespaces())

print('publications')
for topic in manager.publications():
    print('  ', topic)

# subscribe to topics
def diagnostics_callback(data):
    print('diagnostics message:')
    diagnostics_msg = diagnostics_pb2.Diagnostics()
    diagnostics_msg.ParseFromString(data)
    print(diagnostics_msg)
  
subscriber = gazebo_subscribe(manager, '/gazebo/default/diagnostics', 'gazebo.msgs.Diagnostics', diagnostics_callback)
 
# main loop
async def run():
    while True:
        await asyncio.sleep(0.1)
 
asyncio.get_event_loop().run_until_complete(run())

    