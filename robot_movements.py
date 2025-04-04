#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/TurboPi/')
import time
import HiwonderSDK.mecanum as mecanum

test = False  # Este valor puede ser modificado desde el script principal
turn1_count = 0
turn2_count = 0

def move_forward(speed=40):
    print("Movimiento hacia adelante")
    if not test:
        mecanum.MecanumChassis().set_velocity(speed, 90, 0)

def move_backward(speed=40):
    print("Movimiento hacia atrás")
    if not test:
        mecanum.MecanumChassis().set_velocity(speed, 270, 0)

def turn_left(speed=40):
    print("Giro hacia la izquierda")
    if not test:
        mecanum.MecanumChassis().set_velocity(speed, 90, -0.05)

def turn_right(speed=40):
    print("Giro hacia la derecha")
    if not test:
        mecanum.MecanumChassis().set_velocity(speed, 90, 0.05)

def slide_left(speed=40):
    print("Deslizarse hacia la izquierda")
    if not test:
        mecanum.MecanumChassis().set_velocity(speed, 180, 0)

def slide_right(speed=40):
    print("Deslizarse hacia la derecha")
    if not test:
        mecanum.MecanumChassis().set_velocity(speed, 0, 0)

def stop():
    print("Detener robot")
    if not test:
        mecanum.MecanumChassis().set_velocity(0, 0, 0)

def DROPOFF(speed=40):
    print("DROP OFF CARGO")
    print("Giro 180 grados")
    if not test:
        start_time = time.time()
        while time.time()-start_time < 1.3:
            mecanum.MecanumChassis().set_velocity(40,90,0)
        start_time = time.time()
        while time.time()-start_time < 2.25:
            mecanum.MecanumChassis().set_velocity(0,0,0.3)
    stop()

def PICKUP(speed=40):
    print("PICK UP CARGO")
    print("Giro 180 grados")
    if not test:
        start_time = time.time()
        while time.time()-start_time < 1.3:
            mecanum.MecanumChassis().set_velocity(40,90,0)
        start_time = time.time()
        while time.time()-start_time < 2.25:
            mecanum.MecanumChassis().set_velocity(0,0,0.3)
    stop()

def TURN1(speed=40):
    global turn1_count
    turn1_count += 1
    print(f"TURN 1 - llamada #{turn1_count}")
    if turn1_count <= 2:
        print("Giro 90° a la izquierda")
        if not test:
            start_time = time.time()
            while time.time()-start_time < 0.8:
                mecanum.MecanumChassis().set_velocity(40,90,0)
            start_time = time.time()
            while time.time()-start_time < 1.30:
                mecanum.MecanumChassis().set_velocity(30,90,-0.3)
    else: 
        if not test:
            start_time = time.time()
            while time.time()-start_time < 1.30:
                    move_forward(speed)
        turn1_count = 0
    stop()

def TURN2(speed=40):
    global turn2_count
    turn2_count += 1
    print(f"TURN 2 - llamada #{turn2_count}")
    if turn2_count == 1:
        print("Giro 90° a la izquierda")
        if not test:
            start_time = time.time()
            while time.time()-start_time < 0.8:
                mecanum.MecanumChassis().set_velocity(40,90,0)
            start_time = time.time()
            while time.time()-start_time < 1.30:
                mecanum.MecanumChassis().set_velocity(30,90,-0.3)
    else:
        print("Giro 90° a la derecha")
        if not test:
            start_time = time.time()
            while time.time()-start_time < 0.8:
                mecanum.MecanumChassis().set_velocity(40,90,0)
            start_time = time.time()
            while time.time()-start_time < 1.30:
                mecanum.MecanumChassis().set_velocity(30,90,0.3)
        turn2_count = 0
    stop()

def HOME(speed=40):
    print("HOME")
    print("Giro 180 grados")
    if not test:
        start_time = time.time()
        while time.time()-start_time < 1.1:
            mecanum.MecanumChassis().set_velocity(40,90,0)
        start_time = time.time()
        while time.time()-start_time < 2.25:
            mecanum.MecanumChassis().set_velocity(0,0,-0.3)
    stop()
