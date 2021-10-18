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
parser.add_argument('--acceleration', default=0.01, type=float)

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

# advertise topics
joint_publisher = gazebo_advertise(manager, f'/gazebo/default/{args.robot}/joint_cmd', 'gazebo.msgs.JointCmd')

    
#
# keyboard handler
#
key_states = {}

def on_press(key):
    global key_states

    try:
        key_states[key.char] = True
    except AttributeError:
        key_states[key] = True


def on_release(key):
    global key_states

    try:
        key_states[key.char] = False
    except AttributeError:
        key_states[key] = False

keyboard_listener = keyboard.Listener(on_press=on_press, on_release=on_release)
keyboard_listener.start()


#
# robot control
#
wheel_speed = {'left': 0.0, 'right': 0.0}

def set_wheel_speed(left, right):
    global wheel_speed
    changed_speed = False
    
    left = min(max(left, -args.max_speed), args.max_speed)
    right = min(max(right, -args.max_speed), args.max_speed)
    
    if wheel_speed['left'] != left:
        left_msg = pygazebo.msg.joint_cmd_pb2.JointCmd()
        left_msg.name = f'{args.robot}::{args.left_wheel}'
        left_msg.velocity.target = left
        joint_publisher.publish(left_msg)
        wheel_speed['left'] = left
        changed_speed = True
        
    if wheel_speed['right'] != right:
        right_msg = pygazebo.msg.joint_cmd_pb2.JointCmd()
        right_msg.name = f'{args.robot}::{args.right_wheel}'
        right_msg.velocity.target = right
        joint_publisher.publish(right_msg)
        wheel_speed['right'] = right
        changed_speed = True
        
    if changed_speed:
        print(f"set_wheel_speed({left}, {right})")
        
def teleop():
    
    left = wheel_speed['left']
    right = wheel_speed['right']
    
    if key_states.get(keyboard.Key.left) or key_states.get('a'):
        set_wheel_speed(left - args.acceleration, right + args.acceleration)
    elif key_states.get(keyboard.Key.right) or key_states.get('d'):
        set_wheel_speed(left + args.acceleration, right - args.acceleration)
    elif key_states.get(keyboard.Key.up) or key_states.get('w'):
        set_wheel_speed(left + args.acceleration, right + args.acceleration)
    elif key_states.get(keyboard.Key.down) or key_states.get('s'):
        set_wheel_speed(left - args.acceleration, right - args.acceleration)
    else:
        def to_zero(vel):
            if vel < -args.acceleration:  vel += args.acceleration
            elif vel > args.acceleration: vel -= args.acceleration
            else:                         vel = 0
            return vel
            
        set_wheel_speed(to_zero(left), to_zero(right))
    """   
    if key_states.get(keyboard.Key.left) or key_states.get('a'):
        set_wheel_speed(-args.max_speed, args.max_speed)
    elif key_states.get(keyboard.Key.right) or key_states.get('d'):
        set_wheel_speed(args.max_speed, -args.max_speed)
    elif key_states.get(keyboard.Key.up) or key_states.get('w'):
        set_wheel_speed(args.max_speed, args.max_speed)
    elif key_states.get(keyboard.Key.down) or key_states.get('s'):
        set_wheel_speed(-args.max_speed, -args.max_speed)
    else:
        set_wheel_speed(0.0, 0.0)
    """
    
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
    while run_signal:
        teleop()
        await asyncio.sleep(0.1)
 
    print('Shutting down, stopping robot...')
    set_wheel_speed(0,0)
    await asyncio.sleep(1.0)
    
asyncio.get_event_loop().run_until_complete(run())

    