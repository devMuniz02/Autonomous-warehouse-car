#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/TurboPi/')
import HiwonderSDK.mecanum as mecanum
import Camera
import socket
import threading
import cv2
import numpy as np
import imutils, base64
import subprocess
import re
import serial
import signal
import time

from robot_movements import (
    move_forward, move_backward, turn_left, turn_right, 
    slide_left, slide_right, DROPOFF, PICKUP, 
    TURN1, TURN2, HOME, stop
)

# -- Configuración general --
turn1_count = 0
turn2_count = 0
start_time = 0
wait_time = 0.6

# -- Manejo de señales --
def signal_handler(sig, frame):
    stop()
    sys.exit()
signal.signal(signal.SIGINT, signal_handler)

# -- UDP movement control --
def movement_control_listener(message='stop'):
    if message == "stop" or message is None:
        stop()
    elif message == "forward":
        move_forward()
        start_time = time.time()
        while time.time()-start_time < wait_time:
            pass
        stop()
    elif message == "backward":
        move_backward()
        start_time = time.time()
        while time.time()-start_time < wait_time:
            pass
        stop()
    elif message == "right":
        turn_right()
        start_time = time.time()
        while time.time()-start_time < wait_time/2:
            pass
        stop()
    elif message == "left":
        turn_left()
        start_time = time.time()
        while time.time()-start_time < wait_time/2:
            pass
        stop()
    elif message == "slide right":
        slide_right()
        start_time = time.time()
        while time.time()-start_time < wait_time:
            pass
        stop()
    elif message == "slide left":
        slide_left()
        start_time = time.time()
        while time.time()-start_time < wait_time:
            pass
        stop()
    elif message == "slow forward":
        move_forward(speed=40)
        start_time = time.time()
        while time.time()-start_time < wait_time/2:
            pass
        stop()
    elif message == "slow left":
        turn_left(speed=40)
        start_time = time.time()
        while time.time()-start_time < wait_time/2.5:
            pass
        stop()
    elif message == "slow right":
        turn_right(speed=40)
        start_time = time.time()
        while time.time()-start_time < wait_time/2.5:
            pass
        stop()
    elif message == "DROP-OFF":
        DROPOFF()
    elif message == "PICK-UP":
        PICKUP()
    elif message == "HOME":
        HOME()
    elif message == "TURN1":
        TURN1()
    elif message == "TURN2":
        TURN2()
    else:
        stop()
        print(f"Unknown command received: {message}")
    

# -- Obtener IP local --
def get_ip_address():
    result = subprocess.run(['ifconfig'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    match = re.search(r"wlan0:.*?inet (\d+\.\d+\.\d+\.\d+)", result.stdout, re.DOTALL)
    return match.group(1) if match else None

# -- Captura de cámara y envío --
def capture_and_send():
    frame = camera.frame
    if frame is not None:
        frame = imutils.resize(frame, width=400)
        encoded, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 30])
        image_socket.sendto(buffer.tobytes(), address1)
    threading.Timer(0.05, capture_and_send).start()

def show_camera():
    capture_and_send()

# -- Sensor ultrasónico --
def send_data():
    try:
        distance_data = ser.readline().decode('utf-8').strip()
    except:
        distance_data = "[0,0,0]"
    sensor_socket.sendto(distance_data.encode('utf-8'), address)
    threading.Timer(0.02, send_data).start()

def show_ultrasonic():
    send_data()

# -- Escucha de comando stop dedicado --
def stop_socket_listener():
    stop_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    stop_socket.bind((serverIP, 4444))
    while True:
        data, addr = stop_socket.recvfrom(buffer_size)
        message = data.decode('utf-8')
        if message == "stop":
            stop()
            print(f"Received stop command from {addr}. Stopping robot.")
        else:
            print(f"Received unknown command: {message} from {addr}")

# -- Programa principal --
if __name__ == "__main__":
    movement_port = 3333
    sensor_port = 5555
    image_port = 6666
    stop_port = 4444
    buffer_size = 40000
    serverIP = get_ip_address()

    movement_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    movement_socket.bind((serverIP, movement_port))

    sensor_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sensor_socket.bind((serverIP, sensor_port))

    image_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    image_socket.bind((serverIP, image_port))

    print('Server is UP and Listening...', serverIP)

    stop_thread = threading.Thread(target=stop_socket_listener, daemon=True)
    stop_thread.start()

    message, address = sensor_socket.recvfrom(buffer_size)
    sensor_socket.sendto(b'1', address)

    message, address1 = image_socket.recvfrom(buffer_size)
    image_socket.sendto(b'1', address1)

    message, address2 = movement_socket.recvfrom(buffer_size)
    movement_socket.sendto(b'1', address2)
    movement_socket.settimeout(3)
    print('Client Address for sensor:', address)
    print('Client Address for image:', address1)
    print('Client Address for movement:', address2)

    camera = Camera.Camera()
    camera.camera_open(correction=True)
    ser = serial.Serial('/dev/ttyACM0', 9600)

    threading.Thread(target=show_camera, daemon=True).start()
    threading.Thread(target=show_ultrasonic, daemon=True).start()

    # -- Bucle principal de control --
    while True:
        try:
            data, addr = movement_socket.recvfrom(buffer_size)
            movement = data.decode('utf-8')
            print(f"Received movement command: {movement} from {addr}")
        except socket.timeout:
            movement = "stop"
            print("No command received within 3 seconds. Defaulting to stop.")

        movement_control_listener(movement)

    stop()
    cv2.destroyAllWindows()
    movement_socket.close()
    image_socket.close()
    sensor_socket.close()
