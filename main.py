#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.iodevices import Ev3devSensor
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time

ev3 = EV3Brick()

Err_old = 0
timer_ = time.time()

color_r = ColorSensor(Port.S3)
color_l = ColorSensor(Port.S4)
hitech_l = Ev3devSensor(Port.S2)
hitech_r = Ev3devSensor(Port.S1)

motor_r = Motor(Port.C)
motor_l = Motor(Port.D)
motor_m = Motor(Port.B)
motor_b = Motor(Port.A)    

#Дальше бога нет
#Есть только много МНОГОФУНКЦИОНАЛЬНЫХ функций для езды 

def PD_reg(Kp, Kd, Speed):
    global Err_old
    global timer_
    Err = color_l.reflection() - color_r.reflection()
    motor_l.run(-1 * (Speed + (Kp * Err + Kd * (Err - Err_old))))
    motor_r.run(Speed - (Kp * Err + Kd * (Err - Err_old)))
    if (time.time() - timer_) > 0.05:
        timer_ = time.time()
        Err_old = Err

def PD_time(Kp, Kd, Speed, Time):
    timing = time.time()
    while time.time() - timing < Time:
        PD_reg(Kp, Kd, Speed)

def PD_line_X(Kp, Kd, Speed, line):
    while color_l.reflection() >= line or color_r.reflection() >= line:
        PD_reg(Kp, Kd, Speed)
    motor_l.hold()
    motor_r.hold()

def PD_line_T(Kp, Kd, Speed, line):
    while color_l.reflection() >= line and color_r.reflection() >= line:
        PD_reg(Kp, Kd, Speed)
    motor_l.brake()
    motor_r.brake()

def PD_line_f_n(Kp, Kd, Speed, line):
    while color_l.reflection() >= line and color_r.reflection() >= line:
        PD_reg(Kp, Kd, Speed)
    
    motor_l.run(-1*Speed)
    motor_r.run(Speed)
    wait(200)

def PD_line_f_x(Kp, Kd, Speed, Speed_t, line, move):
    PD_line_X(Kp, Kd, Speed, line)
    motor_l.run_angle(-1 * Speed_t, move, wait=False)
    motor_r.run_angle(Speed_t, move)
    motor_l.stop()
    motor_r.stop()

def PD_line_f_t(Kp, Kd, Speed, Speed_t, line, move):
    PD_line_T(Kp, Kd, Speed, line)
    motor_l.run_angle(-1 * Speed_t, move, wait=False)
    motor_r.run_angle(Speed_t, move)
    motor_l.stop()
    motor_r.stop()

def PD_line_l_t(Kp, Kd, Speed, Speed_t, line, line_m, move_f, move_t, correct, correct_t):
    PD_line_f_t(Kp, Kd, Speed, Speed_t, line, move_f)
    motor_l.run_angle(Speed_t, move_t, then=Stop.COAST, wait=False)
    motor_r.run_angle(Speed_t, move_t, then=Stop.COAST)
    while color_r.reflection() >= line_m:
        motor_l.run(Speed_t)
        motor_r.run(Speed_t)
    motor_l.brake()
    motor_r.brake()  
    while color_l.reflection() >= line_m + correct:
        motor_l.run(-1 * (Speed_t + correct_t)
        motor_r.run(-1 * (Speed_t + correct_t)
    motor_l.hold()
    motor_r.hold()

def PD_line_l_x(Kp, Kd, Speed, Speed_t, line, line_m, move_f, move_t, correct, correct_t):
    PD_line_f_x(Kp, Kd, Speed, Speed_t, line, move_f)
    motor_l.run_angle(Speed_t, move_t, then=Stop.COAST, wait=False)
    motor_r.run_angle(Speed_t, move_t, then=Stop.COAST)
    while color_r.reflection() >= line_m:
        motor_l.run(Speed_t)
        motor_r.run(Speed_t)
    motor_l.brake()
    motor_r.brake()  
    while color_l.reflection() >= line_m + correct:
        motor_l.run(-1 * (Speed_t + correct_t)
        motor_r.run(-1 * (Speed_t + correct_t)
    motor_l.hold()
    motor_r.hold()

def PD_line_r_x(Kp, Kd, Speed, Speed_t, line, line_m, move_f, move_t, correct, correct_t):
    PD_line_f_x(Kp, Kd, Speed, Speed_t, line, move_f)
    motor_l.run_angle(-1 * Speed_t, move_t, then=Stop.COAST, wait=False)
    motor_r.run_angle(-1 * Speed_t, move_t, then=Stop.COAST)
    while color_l.reflection() >= line_m:
        motor_l.run(-1 * Speed_t)
        motor_r.run(-1 * Speed_t)
    motor_l.brake()
    motor_r.brake()
    while color_r.reflection() >= line_m + correct:
        motor_l.run(Speed_t + correct_t)
        motor_r.run(Speed_t + correct_t)
    motor_l.hold()
    motor_r.hold() 

def PD_line_r_t(Kp, Kd, Speed, Speed_t, line, line_m, move_f, move_t, correct, correct_t):
    PD_line_f_t(Kp, Kd, Speed, Speed_t, line, move_f)
    motor_l.run_angle(-1 * Speed_t, move_t, then=Stop.COAST, wait=False)
    motor_r.run_angle(-1 * Speed_t, move_t, then=Stop.COAST)
    while color_l.reflection() >= line_m:
        motor_l.run(-1 * Speed_t)
        motor_r.run(-1 * Speed_t)
    motor_l.brake()
    motor_r.brake()
    while color_r.reflection() >= line_m + correct:
        motor_l.run(Speed_t + correct_t)
        motor_r.run(Speed_t + correct_t)
    motor_l.hold()
    motor_r.hold()  

def Move(Speedl, Speedr, anglel, angler):
    motor_l.reset_angle(0)
    motor_r.reset_angle(0)
    flag_r = 1
    flag_l = 1
    while abs(motor_l.angle()) < abs(anglel) or abs(motor_r.angle()) < abs(angler):
        if abs(motor_r.angle()) > abs(angler):
            flag_r = 0
        if abs(motor_l.angle()) > abs(anglel):
            flag_l = 0
        motor_l.run(-1*Speedl*flag_l)
        motor_r.run(Speedr*flag_r)


#PD_line_l(2, 1.2, 400, 350, 27, 30, 105, 150, 50, -50)

#(2, 2, 960, 0, 0) - max speed
#(1.7, 1.2, 400, 0, 0) - normal speed

#Начало программы
#Поднятие мотора и поворот на 45
motor_m.run_target(400, 90, wait=False)
Move(400, 400, 270, 40)
Move(400, 400, 120, 120)
motor_r.stop()
motor_l.stop()
#Начинается взятие бутылок

PD_time(2, 1.2, 350, 0.68)#Проезд до первой банки
motor_r.stop()
motor_l.stop()
motor_b.run_time(-400, 1000)
Move(-400, -400, 150, 150)#Отъезд назад после взятия
#Первая бутылка
motor_r.hold()
motor_l.hold()
Move(300, -300, 50, 50)#Поворот до второй банки
motor_r.hold()
motor_l.hold()
motor_b.run_target(200, 1)
Move(400, 400, 270, 270)#Проезд до второй банки
motor_r.stop()
motor_l.stop()
motor_b.run_time(-400, 1000)
Move(300, 0, 330, 0)
#Бутылки взяты
PD_time(2, 1.2, 400, 0.5)
ev3.speaker.beep()
PD_line_r_x(2, 1.2, 400, 300, 35, 20, 125, 110, 40, -50)    #Саша, есть предположение, что я переборщил с количеством параметров...

"""
Move(-300, -300, 90, 90)
Move(200, -200, 205, 205)
motor_r.stop()
motor_l.stop()
motor_r.run_angle(-300, 290)
motor_l.run_angle(300, 240)
motor_b.run_time(960, 300)
motor_m.run_angle(-150, 30)
Move(-150, -150, 120, 120) #Отъезд назад при взятии бутылки
motor_r.stop()
motor_l.stop()
Move(35, 100, 105, 300)
motor_r.stop()
motor_l.stop()

motor_m.run_angle(150, 30)
motor_b.run_time(-960, 500)
Move(300,-300, 70, 70)
motor_r.stop()
motor_l.stop()
motor_b.stop()
motor_m.run_angle(-150, 18)
Move(0, 300, 0, 300)
"""
"""


Move(260, 240, 300, 300)
motor_l.hold()
motor_r.hold()
motor_b.run_time(-960, 1000)
Move(-400, -400, 170, 170)
Move(0, -240, 0, 260)
motor_b.run_time(300, 260)
Move(240, 240, 300, 300)
"""

"""
ev3.speaker.beep()
PD_line(1.7, 1, 400, 0, 0, 20)
sum_ = 0
for i in range(0, 101):
    sum_ += hitech_l.read('COLOR')[0]

if 3 <= sum_/100 <= 5:
    print("Green") 

if 11 <= sum_/100 <= 17:
    print("White")

while True:
    e = hitech_l.read('COLOR')[0]
    print(e)
    wait(500)
print(sum_)
"""