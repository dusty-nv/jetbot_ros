#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import asyncio
import pygazebo

import numpy as np

'''
gz topic -l
gz joint -m 'simple_diff' -j right_wheel_hinge --vel-t 0
gz world --reset-all
'''

parser = argparse.ArgumentParser()

parser.add_argument('--host', default='localhost', type=str)
parser.add_argument('--port', default=11346, type=int)
parser.add_argument('--retry', default=30, type=int)

parser.add_argument('--model', default='simple_diff', type=str)
parser.add_argument('--camera', default='camera_link', type=str)
parser.add_argument('--left-wheel', default='left_wheel_hinge', type=str)
parser.add_argument('--right-wheel', default='right_wheel_hinge', type=str)

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

# subscribe to topics
def on_image(data):
    print('image message:')
    msg = pygazebo.msg.image_stamped_pb2.ImageStamped()
    msg.ParseFromString(data)
    print(msg.time)
    print(f'width={msg.image.width} height={msg.image.height} pixel_format={msg.image.pixel_format} step={msg.image.step}')
    
    img = np.frombuffer(msg.image.data, dtype=np.uint8)
    img = np.reshape(img, (msg.image.height, msg.image.width, 3))
    
    print(img.shape)
    print('')

image_subscriber = gazebo_subscribe(manager, f'/gazebo/default/{args.model}/{args.camera}/camera/image', 'gazebo.msgs.ImageStamped', on_image)
 
# advertise topics
joint_publisher = gazebo_advertise(manager, f'/gazebo/default/{args.model}/joint_cmd', 'gazebo.msgs.JointCmd')

def set_wheel_speed(left, right):
    left_msg = pygazebo.msg.joint_cmd_pb2.JointCmd()
    left_msg.name = f'{args.model}::{args.left_wheel}'
    left_msg.velocity.target = left
    
    right_msg = pygazebo.msg.joint_cmd_pb2.JointCmd()
    right_msg.name = f'{args.model}::{args.right_wheel}'
    right_msg.velocity.target = right
    
    joint_publisher.publish(left_msg)
    joint_publisher.publish(right_msg)
    
# main loop
async def run():
    while True:
        set_wheel_speed(0.0, 0.0)
        await asyncio.sleep(0.1)
 
asyncio.get_event_loop().run_until_complete(run())

    