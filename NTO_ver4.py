import pymurapi as api
import cv2 as cv
import time
import numpy as np
import math

mur = api.mur_init()

# предопределенные диапазоны цветов в формате HSV

colors = {
    'orange':   ((15,  50,  50), ( 30, 255, 255)),
    'dark_red': ((175, 200,  95), (255, 255, 255)),
    'black': ((0, 0, 0), (255, 255, 25)),
    'green': ((65, 70, 41), (121, 255, 255)),
    'purple': ((0, 70, 0), (140, 255, 255)),
    'yellow': ((0, 70, 51), (58, 255, 255))
}

MY_COLOR_TRUBA = 'purple'
MY_COLOR_GREEN = 'green'
MY_COLOR_YELLOW = 'yellow'

# разрешение камер
cam_w = 320
cam_h = 240


    
counter_of_korz = 0 

flag = False

# функция для ограничения значения диапазоном
def clamp(value, min_value, max_value):
    if value < min_value:
        return min_value
    if value > max_value:
        return max_value
    return value

# функция для вычисления угла по двум точкам
def angle_between(p1, p2):
    xDiff = p2[0] - p1[0]
    yDiff = p2[1] - p1[1]
    return math.degrees(math.atan2(yDiff, xDiff) - (np.pi / 2))

# функция для вычисления расстояния от начала координат
def length_from_center(x, iy):
    return math.sqrt(x ** 2 + y ** 2)

# PD-регулятор
class PDRegulator(object):
    _p_gain = 0.0
    _d_gain = 0.0
    _prev_error = 0.0
    _timestamp = 0

    def __init__(self):
        pass

    def set_p_gain(self, value):
        self._p_gain = value

    def set_d_gain(self, value):
        self._d_gain = value

    def process(self, error):
        timestamp = int(round(time.time() * 1000))

        if timestamp == self._timestamp:
            return 0

        output = self._p_gain * error + self._d_gain / (timestamp - self._timestamp) * (error - self._prev_error)
        
        self._timestamp = timestamp
        self._prev_error = error
        return output

# функция для поддержания установленного курса робота
def keep_yaw(yaw_to_set, speed, round0):
    def clamp_angle(angle):
        if angle > 180.0:
            return angle - 360.0
        if angle < -180.0:
            return angle + 360
        return angle

    try:
        error = mur.get_yaw() - yaw_to_set
        error = clamp_angle(error)
        output = keep_yaw.yaw_regulator.process(error)
        if round0 == 0:
            mur.set_motor_power(0, int(clamp(-output + speed, -100, 100)))
            mur.set_motor_power(1, int(clamp(output + speed, -100, 100)))
        else:
            mur.set_motor_power(0, clamp(round0, -100, 100))
            mur.set_motor_power(1, clamp(-round0, -100, 100))
    except AttributeError:
        keep_yaw.yaw_regulator = PDRegulator()
        keep_yaw.yaw_regulator.set_p_gain(0.8)
        keep_yaw.yaw_regulator.set_d_gain(0.6)

# функция для поддержания установленной глубины погружения
def keep_depth(depth_to_set):
    try:
        error = mur.get_depth() - depth_to_set
        output = keep_depth.depth_regulator.process(error)
        output = clamp(output, -100, 100)
        mur.set_motor_power(2, output)
        mur.set_motor_power(3, output)
    except AttributeError:
        keep_depth.depth_regulator = PDRegulator()
        keep_depth.depth_regulator.set_p_gain(45)
        keep_depth.depth_regulator.set_d_gain(5)

# класс для хранения текущего состояния
class AUVContext(object):
    _yaw = 0.0
    _depth = 0.0
    _round0 = 0.0
    _round1 = 0.0
    _speed = 0.0
    _side_speed = 0.0
    _timestamp = 0
    _missions = []
    _min_area = math.inf
    _min_yaw = 0.0
    _stabilization_counter = 0
    _auto_stabilization = True
    time = 0

    def __init__(self):
        pass

    def set_min_circle(self, yaw, area):
        if area < self._min_area:
            self._min_area = area
            self._min_yaw = yaw

    def get_min_circle_yaw(self):
        return self._min_yaw

    def get_yaw(self):
        return self._yaw

    def get_depth(self):
        return self._depth

    def get_speed(self):
        return self._speed

    def get_side_speed(self):
        return self._side_speed
        
    def get_speed_round(self):
        return self._round0

    def get_auto_stabilization(self):
        return self._auto_stabilization

    def set_yaw(self, value):
        self._yaw = value

    def set_depth(self, value):
        self._depth = value

    def set_speed(self, value):
        self._speed = value

    def set_side_speed(self, value):
        self._side_speed = value
        
    def set_speed_round(self, value):
        self._round0 = value
        
    def get_stabilization_counter(self):
        return self._stabilization_counter

    def reset_stabilization_counter(self):
        self._stabilization_counter = 0

    def add_stabilization_counter(self):
        self._stabilization_counter += 1

    def check_stabilization(self, timeout = 3):
        if self._stabilization_counter > timeout:
            context.reset_stabilization_counter()
            return True
        else:
            self.add_stabilization_counter()
            return False

    def set_auto_stabilization(self, state):
        self._auto_stabilization = state

    def push_mission(self, mission):
        self._missions.append(mission)

    def push_mission_list(self, missions):
        for mission in missions:
            self.push_mission(mission)

    def pop_mission(self):
        if len(self._missions) != 0:
            return self._missions.pop(0)
        return {}

    def get_missions_length(self):
        return len(self._missions)

    def process(self):
        timestamp = int(round(time.time() * 1000))
        if timestamp - self._timestamp > 16:
            if self._auto_stabilization:
                keep_yaw(self._yaw, self._speed, self._round0)
                keep_depth(self._depth)
                mur.set_motor_power(4, self._side_speed)
            self._timestamp = timestamp
        else:
            time.sleep(0.05)

# объект, где будет хранится текущее состояние
context = AUVContext()

# расчёт угла относительно прямоугольника
# для определение отклонения от полоски,
# чтобы затем скорректировать курс
def find_rectangle_contour_angle(contour):
    rectangle = cv.minAreaRect(contour)
    box = cv.boxPoints(rectangle)
    box = np.int0(box)
    edge_first = np.int0((box[1][0] - box[0][0], box[1][1] - box[0][1]))
    edge_second = np.int0((box[2][0] - box[1][0], box[2][1] - box[1][1]))

    edge = edge_first
    if cv.norm(edge_second) > cv.norm(edge_first):
        edge = edge_second

    angle = -((180.0 / math.pi * math.acos(edge[0] / (cv.norm((1, 0)) * cv.norm(edge)))) - 90)
    return angle





def auto_stabilization_on():
    context.set_auto_stabilization(True)
    return True

def auto_stabilization_off():
    context.set_auto_stabilization(False)
    return True
    
def wait_timeout(seconds=3):
    timeout = time.time() + seconds
    while True:
        if time.time() > timeout:
            break
        context.process()
        time.sleep(0.05)
    return True

def wait_short():
    wait_timeout(2)
    return True

def wait_short_3():
    wait_timeout(0.5)
    return True
    

def wait():
    wait_timeout(4.25)
    return True

def wait_long():
    wait_timeout(10)
    return True

# остановка движителей
def stop():
    context.set_side_speed(0)
    context.set_speed(0)
    for motor in range(5):
        mur.set_motor_power(motor, 0)
    return True


def detect_shape(contours):

    biggest_shape = None
    biggest_shape_pos = (0, 0)
    biggest_area = 0
    contour = None

    if contours:
        for contour_1 in contours:
            tag_area = cv.contourArea(contour_1)

            if (tag_area > 200) and (tag_area > biggest_area):
                biggest_area = tag_area
                contour = contour_1
        if not contour is None:       
            (circle_x, circle_y), circle_radius = cv.minEnclosingCircle(contour)
            circle_area = circle_radius ** 2 * math.pi
            
            biggest_shape = 'circle'
            
            biggest_shape_pos = (circle_x, circle_y)
        
    if not biggest_shape is None:
        return biggest_shape, biggest_shape_pos
    else:
        return False, False

def detect_area_shape(contours):

    biggest_shape = None
    biggest_shape_pos = (0, 0)
    biggest_area = 0

    if contours:
        for contour_1 in contours:
            tag_area = cv.contourArea(contour_1)

            if (tag_area > 100) and (tag_area > biggest_area):
                biggest_area = tag_area
                contour = contour_1
        
        area = cv.contourArea(contour)
        
        
    if not biggest_shape is None:
        return area
    else:
        return 0
        
        

# цикл выполнения подзадач

def do_loop_task(tasks):
    for task in tasks:
        print('- ', task.__name__, '[{}]'.format(context.time))
        while not task():
            context.process()
            context.time += 1


context.shape_counter = {
    'shape': None,
    'counter': 0,
}

# поиск фигур
def Detect_korzina(korzina_mask):
    shape, pos = detect_shape(korzina_mask)


    if shape == 'circle':
        return True

    return False
    

# поиск на изображении контура по цвету
def find_contours(image, color_low, color_high, approx = cv.CHAIN_APPROX_SIMPLE):
    hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv_image, color_low, color_high)
    #print("working")
    #cv.imshow('result', mask)
    #cv.waitKey(3)
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, approx)
    area = 0
    if contours:
        for contour in contours:
            area += cv.contourArea(contour)
    return contours, area


def find_coordinates(korzina_mask):

    image_hsv = cv.cvtColor(mur.get_image_bottom(), cv.COLOR_BGR2HSV)
    lower_green = colors[MY_COLOR_GREEN][0]
    upper_green = colors[MY_COLOR_GREEN][1]
    lower_yellow = colors[MY_COLOR_YELLOW][0]
    upper_yellow = colors[MY_COLOR_YELLOW][1]
    image_bin_green = cv.inRange(image_hsv, lower_green, upper_green)
    image_bin_yellow = cv.inRange(image_hsv, lower_yellow, upper_yellow)
    image_bin = image_bin_yellow + image_bin_green
    
    contours = korzina_mask

    if contours:
        for cnt in contours:
            area = cv.contourArea(cnt)
            if abs(area) < 20:
                continue
            ((x1, y1), (w, h), _) = cv.minAreaRect(cnt)
            (_, _), radius = cv.minEnclosingCircle(cnt)
            rectangle_area = w * h
            circle_area = radius ** 2 * math.pi
            aspect_ratio = w / h
            cv.drawContours(image_bin, cnt, 0, (255,0,0),2) # рисуем прямоугольник
            cv.circle(image_bin, (int(x1), int(y1)), 5, (255,0,0), 2) # рисуем маленький кружок в центре прямоугольника
            #cv.imshow("Image", image_bin)
            #cv.waitKey(500)
#            if final._last_area >= area+3:
            if 0.9 <= aspect_ratio <= 1.1:
                moments = cv.moments(cnt)
                try:
                    x = int(moments["m10"] / moments["m00"])
                    y = int(moments["m01"] / moments["m00"])
                except ZeroDivisionError:
                    return False, -1, 0, 0, len(contours)
                if rectangle_area > circle_area:
                    return True, x, y, len(contours)
                else:
                    return True, x, y, len(contours)
                    
    return False, 0, 0, 0

def stab_on_circle_not_bomb(korzina_mask):
    found, x, y, _ = find_coordinates(korzina_mask)
    if found:
        x_center = x - (320 / 2)
        y_center = y + 20 - (240 / 2)
        length = math.sqrt(x_center ** 2 + y_center ** 2)
        if length < 10.0:
            print("разворот")
            return True
        output_forward = stab_on_circle.regulator_forward.process(y_center)
        output_side = stab_on_circle.regulator_side.process(x_center)
        output_forward = clamp(output_forward, -50, 50)
        output_side = clamp(output_side, -50, 50)
        context.set_speed(-output_forward)
        context.set_side_speed(-output_side)

    return False

def stab_on_circle(korzina_mask):
    found, x, y, _ = find_coordinates(korzina_mask)
    if found:
        x_center = x - (320 / 2)
        y_center = y + 20 - (240 / 2)
        try:
            length = math.sqrt(x_center ** 2 + y_center ** 2)
            cur_speed = context.get_speed()
            if length < 10.0 and (-5 <= cur_speed <= 5):
                print("стабилизировался")
                mur.drop()
                return True
            output_forward = stab_on_circle.regulator_forward.process(y_center)
            output_side = stab_on_circle.regulator_side.process(x_center)
            output_forward = clamp(output_forward, -50, 50)
            output_side = clamp(output_side, -50, 50)
            context.set_speed(-output_forward)
            context.set_side_speed(-output_side)
        except AttributeError:
            stab_on_circle.regulator_forward = PDRegulator()
            stab_on_circle.regulator_forward.set_p_gain(0.2)  # было 0.8
            stab_on_circle.regulator_forward.set_d_gain(0.5)

            stab_on_circle.regulator_side = PDRegulator()
            stab_on_circle.regulator_side.set_p_gain(0.2)  # было 0.8
            stab_on_circle.regulator_side.set_d_gain(0.5)

    return False
    

def side_correct_movement(contour):
    
    ((x, _), (_, _), _) = cv.minAreaRect(contour)
    x_center = (cam_w / 2)
    x_res = (x_center - x)
    #context.set_side_speed(x_res) * 0.8
    try:
        output_side = side_correct_movement.regulator_side.process(x_res)
        output_side = clamp(output_side, -50, 50)
        #print(output_side)
        context.set_side_speed(output_side)
        
    except AttributeError:
        
        side_correct_movement.regulator_side = PDRegulator()
        side_correct_movement.regulator_side.set_p_gain(0.5)
        side_correct_movement.regulator_side.set_d_gain(0.1)
            
    return True 
    


      
def bomb_korzina():
    global flag
    image = mur.get_image_bottom()
    green_mask, green_area = find_contours(image, colors[MY_COLOR_GREEN][0], colors[MY_COLOR_GREEN][1], cv.CHAIN_APPROX_SIMPLE)
    yellow_mask, yellow_area  = find_contours(image, colors[MY_COLOR_YELLOW][0], colors[MY_COLOR_YELLOW][1], cv.CHAIN_APPROX_SIMPLE)
    korzina_mask = green_mask + yellow_mask
    if not Detect_korzina(korzina_mask):
        flag = False
        truba_mask, _ = find_contours(image, colors[MY_COLOR_TRUBA][0], colors[MY_COLOR_TRUBA][1], cv.CHAIN_APPROX_SIMPLE)
        go_by_truba()    
        return False
    if flag == False:
        context.set_speed(-7)
        context.set_side_speed(0)
        flag = True
    if stab_on_circle(korzina_mask):
        global counter_of_korz
        if green_area > yellow_area:
            counter_of_korz +=1
            print('green')
        elif green_area < yellow_area:
            counter_of_korz +=2
            print('yellow')
        return True
    return False
    
def go_to_return():
    global flag
    image = mur.get_image_bottom()
    green_mask, green_area = find_contours(image, colors[MY_COLOR_GREEN][0], colors[MY_COLOR_GREEN][1], cv.CHAIN_APPROX_SIMPLE)
    yellow_mask, yellow_area  = find_contours(image, colors[MY_COLOR_YELLOW][0], colors[MY_COLOR_YELLOW][1], cv.CHAIN_APPROX_SIMPLE)
    korzina_mask = green_mask + yellow_mask
    if not Detect_korzina(korzina_mask):
        flag = False
        truba_mask, _ = find_contours(image, colors[MY_COLOR_TRUBA][0], colors[MY_COLOR_TRUBA][1], cv.CHAIN_APPROX_SIMPLE)
        go_by_truba()    
        return False
    if flag == False:
        context.set_speed(-7)
        flag = True
    if stab_on_circle_not_bomb(korzina_mask):
        context.set_yaw(context.get_yaw() + 180)
        return True
    return False


def go_by_truba():
    image = mur.get_image_bottom()
    contours, _ = find_contours(image, colors[MY_COLOR_TRUBA][0], colors[MY_COLOR_TRUBA][1], cv.CHAIN_APPROX_SIMPLE)
    
    for contour in contours:
        tag_area = cv.contourArea(contour)
    
        if (tag_area < 100): # исключаем маленькие области
            pass
        else:        
            truba_angle = find_rectangle_contour_angle(contour)
            side_correct_movement(contour)
            if not math.isnan(truba_angle):
                context.set_yaw(mur.get_yaw() + truba_angle)

    return True

def go_by_truba_multi():
    set_spd()
    for i in range(6):
        image = mur.get_image_bottom()
        contours, _ = find_contours(image, colors[MY_COLOR_TRUBA][0], colors[MY_COLOR_TRUBA][1], cv.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            tag_area = cv.contourArea(contour)
        
            if (tag_area < 100): # исключаем маленькие области
                pass
            else:        
                truba_angle = find_rectangle_contour_angle(contour)
                side_correct_movement(contour)
                if not math.isnan(truba_angle):
                    context.set_yaw(mur.get_yaw() + truba_angle)
        wait_short_3()
    return True
    
    
def escape():
    
    context.set_speed(0)
    time.sleep(0.05)
    
    if counter_of_korz % 2 == 0:  # всплытие по часовой стрелке
        context.set_speed_round(100)
    else:
        context.set_speed_round(-100)
        
    context.set_depth(0)
    
    
    return True
  
def set_spd():
    context.set_speed(25)
    return True
    
def set_data():
    context.set_depth(3)
    context.set_yaw(0.0)
    return True
    
def chech_deth():
    if round(mur.get_depth(), 1) == 3:
        return True
    return False
    
    
# основной код программы
if __name__ == "__main__":

    set_data()


    # определим подзадачи, которые требуется решить
    initial_position = (
        chech_deth,
        set_spd,
    )
    
    main_task = (
        
        bomb_korzina,
        go_by_truba_multi,
        
        bomb_korzina,
        go_by_truba_multi,
        
        bomb_korzina,
        go_by_truba_multi,
        
        bomb_korzina,
        go_by_truba_multi,
        
        bomb_korzina,
        go_by_truba_multi,
        
        
        go_to_return,
        wait,
        go_by_truba_multi,
        go_to_return,
        wait,
    )
    
    finish = (
        stop,
        escape,
        wait_long,
        stop
    )

    # основная миссия состоит из ранее описанных подзадач

    missions = (
        *initial_position,
        *main_task,
        *finish,
    )

    context.push_mission_list(missions)

    print("start")
    
    # основной цикл программы, где выполняются все определенные задачи
    while (True):
        mission = context.pop_mission()
        print('starting', mission.__name__, '[{}]'.format(context.time))
        while not mission():
            context.process()
            context.time += 1

        if context.get_missions_length() == 0:
            break

    print("done!")

    context.set_speed(0)
    context.set_depth(0)

    time.sleep(3)
    
    
    
# 
#Изменения:
#1) мульти проход после корзины
#2) увеличил глубину для обнаружения 2х корзин рядом
#3) добавил доход до 6 корзины и оратно
#4) Сделал PD регулятор для бокового стабилизатора трубы
#5) Исправил проблему с поворотом на тёмой корзине 
#6) Удалил ненужный код и принты
#   
   
   
   
   
   
   
   
