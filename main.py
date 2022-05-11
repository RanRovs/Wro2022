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

#Переменные
Err_old = 0             #ПД-регулятор
timer_ = time.time()    #ПД-регулятор
left_array = []         #Список обнаруженных цветов ЛЕВЫМ датчиком
right_array = []        #Список обнаруженных цветов ПРАВЫМ датчиком
sum_ = None             #Направление (Лево или Право)((0, 1, 2 или 3))
actions = [0, 0]        #Список расположения кубиков (Белый слева, зелёный справа/Зелёный слева, зелёный справа)
Things = [0, 0, 0]      #Порядок расположения бельевых блоков

color_r = ColorSensor(Port.S3)
color_l = ColorSensor(Port.S4)
hitech_l = Ev3devSensor(Port.S2)
hitech_r = Ev3devSensor(Port.S1)

motor_r = Motor(Port.C)
motor_l = Motor(Port.D)
motor_m = Motor(Port.B)
motor_b = Motor(Port.A)    

#Дальше бога нет
#Есть только много МНОГОФУНКЦИОНАЛЬНЫХ функций 

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
        motor_l.run(-1 * (Speed_t + correct_t))
        motor_r.run(-1 * (Speed_t + correct_t))
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
        motor_l.run(-1 * (Speed_t + correct_t))
        motor_r.run(-1 * (Speed_t + correct_t))
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

def Colr(Kp, Kd, Speed, Time_bef, Time_aft): #СЧИТЫВАНИЕ цветов
    PD_time(Kp, Kd, Speed, Time_bef)
    timer_C = time.time()
    ev3.speaker.beep()
    while (time.time() - timer_C) <= Time_aft:
        #Пока цикл работает - списки заполняются
        PD_reg(Kp, Kd, Speed)
        left_array.append(hitech_l.read('COLOR')[0])
        right_array.append(hitech_r.read('COLOR')[0])            

def logik(): #Обработка цвета и определение направления 
    l_white = None
    l_green = 0
    r_white = None
    r_green = 0 
    min_g = 100
    max_g = 0
    min_w = 100
    global sum_

    #Нахождение крайних точек зелёного и белого кубиков
    for i in range(len(left_array)):
        if 3 <= left_array[i] <= 5 or 12 <= left_array[i] <= 13:
            if left_array[i] < min_g:
                min_g = left_array[i]
            if left_array[i] > max_g:
                max_g = left_array[i]
        
        if 14 <= left_array[i]:
            if left_array[i] < min_w:
                min_w = left_array[i]

    #Нахождение длины зелёного и белого промежутка СЛЕВА
    left_array.sort()
    if (3 <= min_g <= 5 or 12 <= min_g <= 13) and (3 <= max_g <= 5 or 12 <= max_g <= 13):
        len_left_g = len(left_array[left_array.index(min_g) : (left_array.index(max_g) + left_array.count(max_g))]) 
    else:
        len_left_g = 0
    
    if 14 <= min_w <= 17:
        len_left_w = len(left_array[left_array.index(min_w) : len(left_array)])
    else:
        len_left_w = 0
    #Если длина зелёного больше белого, то это зелёный кубик СЛЕВА
    #Иначе наоборот
    if len_left_g > len_left_w:
        #GREEN LEFT
        l_white = 0
        actions[0] = 0
    else:
        #WHITE LEFT
        l_white = 2
        actions[0] = 1
    
    min_g = 100
    max_g = 0
    min_w = 100

    #Нахождение крайних точек зелёного и белого кубиков
    for i in range(len(right_array)):
        if 3 <= right_array[i] <= 5 or 12 <= right_array[i] <= 13:
            if right_array[i] < min_g:
                min_g = right_array[i]
            if right_array[i] > max_g:
                max_g = right_array[i]
        
        if 14 <= right_array[i]:
            if right_array[i] < min_w:
                min_w = right_array[i]

    #Нахождение длины зелёного и белого промежутка СПРАВА
    right_array.sort()
    if (3 <= min_g <= 5 or 12 <= min_g <= 13) and (3 <= max_g <= 5 or 12 <= max_g <= 13):
        len_right_g = len(right_array[right_array.index(min_g) : (right_array.index(max_g) + right_array.count(max_g))]) 
    else:
        len_right_g = 0

    if 14 <= min_w <= 17:
        len_right_w = len(right_array[right_array.index(min_w) : len(right_array)])
    else:
        len_right_w = 0
    #Если длина зелёного больше белого, то это зелёный кубик СПРАВА
    #Иначе наоборот
    if len_right_g > len_right_w:
        #GREEN RIGHT
        r_white = 0
        actions[1] = 0
    else:
        #WHITE RIGHT
        r_white = 1
        actions[1] = 1
    
    sum_ = r_green + l_green + r_white + l_white #Направления: 0, 1, 2 или 3

def Green_room_l(): #Дествия для левой зелёной комнаты
    pass
    #Здесь будет большое кол-во проездов
    #Проезд до бельевого блока

def Green_room_r(): #Дествия для правой зелёной комнаты
    pass
    #Здесь будет большое кол-во проездов
    #Проезд до бельевого блока

def White_room_l(): #Дествия для левой белой комнаты
    pass
    #Как зелёная комната, но белая
    #

def White_room_r(): #Дествия для правой белой комнаты

    PD_time(2, 1, 350, 0.40)
    motor_b.run_time(-960, 2000, wait=False)
    Move(0, 300, 0, 220)
    motor_r.stop()
    motor_l.stop()
    motor_b.run_time(-900, 10000)
    #Как зелёная комната, но белая
    #

def Action(): #Выполнение действий, основываясь на направлении
    if sum_ == 2 or sum_ == 3 or sum_ == 0: #Комната находится слева?
        PD_line_l_x(2, 1.2, 400, 300, 30, 20, 125, 110, 40, -150) #ПРИМЕРНОЕ ЗНАЧЕНИЕ
        #Сначала проверяем левую сторону, так как едем налево 
        if actions[0] == 0:     #Комната зелёного цвета?
            Green_room_l()
        else:
            White_room_l()
        #Одну комнату завершли
        if actions[1] == 0:
            Green_room_r()
        else:
            White_room_r()
        #Вторую комнату завершили
    else:
        PD_line_r_x(2, 1.2, 400, 300, 30, 20, 135, 110, 40, -150) #ПРИМЕРНОЕ ЗНАЧЕНИЕ
        #Проверяем правую сторону, так как едем направо
        if actions[1] == 0:     #Комната зелёного цвета?
            Green_room_r()
        else:
            White_room_r()
        #Одну комнату завершли
        if actions[0] == 0:
            Green_room_l()
        else:
            White_room_l()
        #Вторую комнату завершили

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

PD_time(2, 1, 350, 0.68)#Проезд до первой банки
motor_r.stop()
motor_l.stop()
motor_b.run_time(-800, 1000)
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
PD_line_r_x(2, 1.2, 400, 200, 35, 20, 125, 110, 40, -50)    #Саша, есть предположение, что я переборщил с количеством параметров...

PD_line_l_x(2, 1.2, 400, 300, 30, 20, 125, 110, 40, -150)
#Запускается параллельное считывание цветов
left_array = []
right_array = []
Colr(2, 1.2, 400, 0.9, 0.5)
print(right_array)
print("Ebat")
print(left_array)
ev3.speaker.beep()
#Останавливается считывание
logik() #Определяется направление
Action() #Действия!
print(sum_)
print(actions)

#После этой функции мы сделали задачи в двух комнатах. Остаются остальные две, но они буду выполняться абсолютно идентично предыдущим
#Пэотому фнкции начиная со знака "&" будут повторены для оставшейся половины поля
#Нужно будет просто выехать на определённый перекресток
#И уже после этого пойдёт финальная часть, где нужно будет отвезти бльевые блоки и припарковаться