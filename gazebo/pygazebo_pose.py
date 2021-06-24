#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import signal
import argparse
import asyncio
import pygazebo

import numpy as np
from PIL import Image

from pynput import keyboard

'''
gz topic -l
gz joint -m 'simple_diff' -j right_wheel_hinge --vel-t 0
gz world --reset-all

renice -n 15 $(pgrep gzclient)
'''

parser = argparse.ArgumentParser()

parser.add_argument('--host', default='localhost', type=str)
parser.add_argument('--port', default=11346, type=int)
parser.add_argument('--retry', default=30, type=int)

parser.add_argument('--robot', default='simple_diff', type=str)
parser.add_argument('--camera', default='camera_link', type=str)
parser.add_argument('--left-wheel', default='left_wheel_hinge', type=str)
parser.add_argument('--right-wheel', default='right_wheel_hinge', type=str)
parser.add_argument('--max-speed', default=2.0, type=float)

args = parser.parse_args()
print(args)


#
# gazebo connection
#
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
        print(f'advertising {topic_name} ({msg_type})')
        return await manager.advertise(topic_name, msg_type)
        
    return asyncio.get_event_loop().run_until_complete(
        _advertise(manager, topic_name, msg_type))
        
def gazebo_subscribe(manager, topic_name, msg_type, callback):
    async def _subscribe(manager, topic_name, msg_type, callback):
        print(f'subscribing to {topic_name} ({msg_type})')
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

def on_model_info(data):
    msg = pygazebo.msg.model_pb2.Model()
    msg.ParseFromString(data)
    
    print('model info:')
    print(msg)

def on_pose_info(data):
    msg = pygazebo.msg.poses_stamped_pb2.PosesStamped()
    msg.ParseFromString(data)
    
    print('pose info:')
    print(msg)
    
    
model_subscriber = gazebo_subscribe(manager, '/gazebo/default/model/info', 'gazebo.msgs.Model', on_model_info)
pose_subscriber = gazebo_subscribe(manager, '/gazebo/default/pose/info', 'gazebo.msgs.PosesStamped', on_pose_info)

# advertise topics
pose_publisher = gazebo_advertise(manager, '/gazebo/default/model/modify', 'gazebo.msgs.Model')

    

def set_pose(position=(0.0,0.0,0.0), orientation=(0.0,0.0,1.0,1.0)):
    global wheel_speed
    changed_speed = False
    
    model_msg = pygazebo.msg.model_pb2.Model()
    #pose_msg = pygazebo.msg.pose_pb2.Pose()
    #vec_msg = pygazebo.msg.vector3d_pb2.Vector3d()
    
    #vec_msg.x = position[0]
    #vec_msg.y = position[1]
    #vec_msg.z = position[2]
    
    model_msg.pose.position.x = position[0]
    model_msg.pose.position.y = position[1]
    model_msg.pose.position.z = position[2]
    
    model_msg.pose.orientation.x = orientation[0]
    model_msg.pose.orientation.y = orientation[1]
    model_msg.pose.orientation.z = orientation[2]
    model_msg.pose.orientation.w = orientation[3]
    
    model_msg.name = args.robot
    
    print(f"setting pose to pos={position} orientation={orientation}")
    print(model_msg)
    
    pose_publisher.publish(model_msg)


#
# main loop
#
run_signal = True

def signal_handler(sig, frame):
    global run_signal
    run_signal = False
    print('Ctrl+C pressed, exiting...')

signal.signal(signal.SIGINT, signal_handler)

# main loop
async def run():

    x = 0.0
    
    while run_signal:
        set_pose(position=(x,0.01,0.01))
        x += 0.1
        await asyncio.sleep(0.1)
 
    print('Shutting down, stopping robot...')
    await asyncio.sleep(1.0)
    
asyncio.get_event_loop().run_until_complete(run())

    