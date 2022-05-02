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
import threading

ev3 = EV3Brick()

#Переменные
Err_old = 0             #ПД-регулятор
timer_ = time.time()    #ПД-регулятор
Thread_ = False         #Флаг для отключения параллельного считывания (Лож - работает/ Истина - не работает)
left_array = []         #Список обнаруженных цветов ЛЕВЫМ датчиком
right_array = []        #Список обнаруженных цветов ПРАВЫМ датчиком
sum_ = None             #Направление (Лево или Право)((0, 1, 2 или 3))

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

def PD_reg(Kp, Kd, Speed): #ПД-регулятор
    global Err_old
    global timer_
    Err = color_l.reflection() - color_r.reflection()
    motor_l.run(-1 * (Speed + (Kp * Err + Kd * (Err - Err_old))))
    motor_r.run(Speed - (Kp * Err + Kd * (Err - Err_old)))
    if (time.time() - timer_) > 0.05:
        timer_ = time.time()
        Err_old = Err

def PD_time(Kp, Kd, Speed, Time): #ПД-регулятор, который работает указанное время
    timing = time.time()
    while time.time() - timing < Time:
        PD_reg(Kp, Kd, Speed)

def PD_line_X(Kp, Kd, Speed, line): #Езда до Х-образной линии (or)
    while color_l.reflection() >= line or color_r.reflection() >= line:
        PD_reg(Kp, Kd, Speed)
    motor_l.hold()
    motor_r.hold()

def PD_line_T(Kp, Kd, Speed, line): #Езда до Т-образной линии (and)
    while color_l.reflection() >= line and color_r.reflection() >= line:
        PD_reg(Kp, Kd, Speed)
    motor_l.brake()
    motor_r.brake()

def PD_line_f_n(Kp, Kd, Speed, line): #Езда до Т-образной линии с проездом вперед без остановки после 
    while color_l.reflection() >= line and color_r.reflection() >= line:
        PD_reg(Kp, Kd, Speed)
    
    motor_l.run(-1*Speed)
    motor_r.run(Speed)
    wait(200)

def PD_line_f_x(Kp, Kd, Speed, Speed_t, line, move): #Езда до Х-образной линии с проездом вперед и остановкой после
    PD_line_X(Kp, Kd, Speed, line)
    motor_l.run_angle(-1 * Speed_t, move, wait=False)
    motor_r.run_angle(Speed_t, move)
    motor_l.stop()
    motor_r.stop()

def PD_line_f_t(Kp, Kd, Speed, Speed_t, line, move): #Езда до Т-образной линии с проездом вперед и остановкой после
    PD_line_T(Kp, Kd, Speed, line)
    motor_l.run_angle(-1 * Speed_t, move, wait=False)
    motor_r.run_angle(Speed_t, move)
    motor_l.stop()
    motor_r.stop()

def PD_line_l_t(Kp, Kd, Speed, Speed_t, line, line_m, move_f, move_t, correct, correct_t): #Поворот до линии на лево на Т-образном перекрёстке
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

def PD_line_l_x(Kp, Kd, Speed, Speed_t, line, line_m, move_f, move_t, correct, correct_t): #Поворот до линии на лево на Х-образном перекрёстке
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

def PD_line_r_x(Kp, Kd, Speed, Speed_t, line, line_m, move_f, move_t, correct, correct_t): #Поворот до линии на право на Т-образном перекрёстке
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

def PD_line_r_t(Kp, Kd, Speed, Speed_t, line, line_m, move_f, move_t, correct, correct_t): #Поворот до линии на право на Х-образном перекрёстке
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

def Move(Speedl, Speedr, anglel, angler): #Движение обоими моторами на определёный угол одновременно и без остановки после
    #Функция была дабавлена, чтобы исключить такие моменты, когда забываешь, что: 
    #1. Для езды вперед/назад на 1 мотор подаётся инвертированная мощьность;
    #2. Для одновременной работы моторов необходимо прописывать в одном из них дополнительный параметр;
    #3. Для езды по градусам без остановки, необходимо прибегать к небольшим хитростям.
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

def Colr(): #СЧИТЫВАНИЕ цветов
    global left_array
    global right_array
    left_array = []
    right_array = []
    while True:
        #Пока цикл работает - списки заполняются
        left_array.append(hitech_l.read('COLOR'))
        right_array.append(hitech_r.read('COLOR'))
        if Thread_ == True:
            break

def logik(): #Определение направления 
    l_white = None
    l_green = 0
    r_white = None
    r_green = 0 
    min_g = 100
    max_g = 0
    min_w = 100

    #Нахождение крайних точек зелёного и белого кубиков
    for i in range(len(left_array)):
        if 3 <= left_array[i] <= 5:
            if left_array[i] < min_g:
                min_g = left_array[i]
            if left_array[i] > max_g:
                max_g = left_array[i]
        
        if 11 <= left_array[i]:
            if left_array[i] < min_w:
                min_w = left_array[i]

    #Нахождение длины зелёного и белого промежутка СЛЕВА
    left_array.sort()
    len_left_g = len(left_array[left_array.index(min_g) : (left_array.index(max_g) + left_array.count(max_g))]) 
    len_left_w = len(left_array[left_array.index(min_w) : len(left_array)])
    #Если длина зелёного больше белого, то это зелёный кубик СЛЕВА
    #Иначе наоборот
    if len_left_g > len_left_w:
        #GREEN LEFT
        l_white = 0
    else:
        #WHITE LEFT
        l_white = 2
    
    min_g = 100
    max_g = 0
    min_w = 100

    #Нахождение крайних точек зелёного и белого кубиков
    for i in range(len(right_array)):
        if 3 <= right_array[i] <= 5:
            if right_array[i] < min_g:
                min_g = right_array[i]
            if right_array[i] > max_g:
                max_g = right_array[i]
        
        if 11 <= right_array[i]:
            if right_array[i] < min_w:
                min_w = right_array[i]

    #Нахождение длины зелёного и белого промежутка СПРАВА
    right_array.sort()
    len_right_g = len(right_array[right_array.index(min_g) : (right_array.index(max_g) + right_array.count(max_g))]) 
    len_right_w = len(right_array[right_array.index(min_w) : len(right_array)])
    #Если длина зелёного больше белого, то это зелёный кубик СПРАВА
    #Иначе наоборот
    if len_right_g > len_right_w:
        #GREEN RIGHT
        r_white = 0
    else:
        #WHITE RIGHT
        r_white = 1
    
    sum_ = r_green + l_green + r_white + l_white #Направления: 0, 1, 2 или 3

#PD_line_l(2, 1.2, 400, 350, 27, 30, 105, 150, 50, -50)

#(2, 2, 960, 0, 0) - max speed
#(1.7, 1.2, 400, 0, 0) - normal speed

#Начало программы
#Поворот манипулятора и поворот на 45
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

#Запускается параллельное считывание цветов
th = threading.Thread(target=Colr)
th.start()
PD_time(2, 1.2, 400, 3) #ПРИМЕРНОЕ ЗНАЧЕНИЕ
Thread_ = True
#Останавливается считывание
logik() #Определяется направление




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