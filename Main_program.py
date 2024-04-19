import pymurapi as api
import cv2 as cv
import time
import numpy as np
import math

mur = api.mur_init()

# предопределенные диапазоны цветов в формате HSV

'''colors = {
    'orange':   np.array([[15,  50,  50], [30, 255, 255]]),
    'dark_red': np.array([[175, 200,  95], [255, 255, 255]]),
    'black': np.array([[0, 0, 0], [255, 255, 255]]),
    'green': np.array([[65, 70, 51], [121, 255, 255]]),
    'purple': np.array([[0, 70, 51], [140, 255, 255]]),
    'yellow': np.array([[0, 70, 51], [58, 255, 255]])
}'''

colors = {
    'orange':   ((15,  50,  50), ( 30, 255, 255)),
    'dark_red': ((175, 200,  95), (255, 255, 255)),
    'black': ((0, 0, 0), (255, 255, 25)),
    'green': ((65, 70, 51), (121, 255, 255)),
    'purple': ((0, 70, 51), (140, 255, 255)),
    'yellow': ((0, 70, 51), (58, 255, 255))
}

MY_COLOR_TRUBA = 'purple'
MY_COLOR_GREEN = 'green'
MY_COLOR_YELLOW = 'yellow'

# разрешение камер
cam_w = 320
cam_h = 240

count_of_korzinas_alarms = 0

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
def length_from_center(x, y):
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
            mur.set_motor_power(0, clamp(-output + speed, -100, 100))
            mur.set_motor_power(1, clamp(output + speed, -100, 100))
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

# стабилизировать курс и глубину
def stabilize():
    yaw = mur.get_yaw()
    depth = mur.get_depth()

    if abs(yaw - context.get_yaw()) < 1 and abs(depth - context.get_depth()) < 0.4:
        # продолжать корректировать курс, пока не будут
        # достигнуты приемлемые значения
        if context.check_stabilization(timeout=5):
            return True
    else:
        context.reset_stabilization_counter()
    return False

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

# расчёт угла относительно стрелки,
# чтобы затем скорректировать курс
def detect_arrow_angle(image, contour):
    (arrow_center_x, arrow_center_y), radius = cv.minEnclosingCircle(contour)
    moments = cv.moments(contour)

    to_draw = image.copy()
    
    cv.circle(to_draw, (int(arrow_center_x), int(arrow_center_y)), 1, (255, 0, 255), 2)
    cv.circle(to_draw, (int(arrow_center_x), int(arrow_center_y)), int(radius), (255, 0, 255), 2)

    try:
        arrow_direction_x = moments['m10'] / moments['m00']
        arrow_direction_y = moments['m01'] / moments['m00']

        arrow_angle = (angle_between((arrow_direction_x, arrow_direction_y), (arrow_center_x, arrow_center_y)))
        context.set_yaw(mur.get_yaw() + arrow_angle)

        target_x = arrow_center_x - (cam_w / 2)
        target_y = arrow_center_y - (cam_h / 2)
        length = math.sqrt(target_x ** 2 + target_y ** 2)

        return arrow_angle, target_x, target_y, length
    except:
        return False, False, False, False


# стабилизироваться по точке изображения,
# здесь координата y - глубина  _bottom
def stabilize_x_y_bottom(x, y):
    y_center = y - (cam_h / 2)
    x_center = x - (cam_w / 2)

    try:
        length = math.sqrt(x_center ** 2 + y_center ** 2)
        if length < 3.5:
            if context.check_stabilization(timeout=2):
                #mur.drop()                                                             ---------добавить дроп
                return True
        else:
            context.reset_stabilization_counter()

        output_side = stabilize_x_y.side_regulator.process(x_center)
        output_depth = stabilize_x_y.depth_regulator.process(y_center)

        output_side = clamp(int(output_side), -50, 50)
        output_depth = clamp(output_depth, -1, 1)

        context.set_side_speed(-output_side)
        context.set_depth(mur.get_depth() + output_depth)                              #----------   изменить на движки по  горизонтали (вперед, назад)

    except AttributeError:
        stabilize_x_y.side_regulator = PDRegulator()
        stabilize_x_y.side_regulator.set_p_gain(0.5)
        stabilize_x_y.side_regulator.set_d_gain(0.1)

        stabilize_x_y.depth_regulator = PDRegulator()
        stabilize_x_y.depth_regulator.set_p_gain(0.01)
        stabilize_x_y.depth_regulator.set_d_gain(0.01)
    return False
    
        



def stabilize_x_y(x, y):
    y_center = y - (cam_h / 2)
    x_center = x - (cam_w / 2)

    try:
        length = math.sqrt(x_center ** 2 + y_center ** 2)
        if length < 3.5:
            if context.check_stabilization(timeout=2):
                return True
        else:
            context.reset_stabilization_counter()

        output_side = stabilize_x_y.side_regulator.process(x_center)
        output_depth = stabilize_x_y.depth_regulator.process(y_center)

        output_side = clamp(int(output_side), -50, 50)
        output_depth = clamp(output_depth, -1, 1)

        context.set_side_speed(-output_side)
        context.set_depth(mur.get_depth() + output_depth)

    except AttributeError:
        stabilize_x_y.side_regulator = PDRegulator()
        stabilize_x_y.side_regulator.set_p_gain(0.5)
        stabilize_x_y.side_regulator.set_d_gain(0.1)

        stabilize_x_y.depth_regulator = PDRegulator()
        stabilize_x_y.depth_regulator.set_p_gain(0.01)
        stabilize_x_y.depth_regulator.set_d_gain(0.01)
    return False

# стабилизация глубины
def stabilize_y(y):
    y_center = y - (cam_h / 2)
    try:
        output_depth = stabilize_y.depth_regulator.process(y_center)
        output_depth = clamp(context.get_depth() + output_depth, 0, 3)
        context.set_depth(output_depth)
    except AttributeError:
        stabilize_y.depth_regulator = PDRegulator()
        stabilize_y.depth_regulator.set_p_gain(0.00002)
        stabilize_y.depth_regulator.set_d_gain(0.00002)

# функции для управления автоматической стабилизацией.
# это полезно при полностью ручном управлении моторами

def auto_stabilization_on():
    context.set_auto_stabilization(True)
    return True

def auto_stabilization_off():
    context.set_auto_stabilization(False)
    return True

# двигаться вперёд
def go_forward():
    if (context.get_speed() != 30):
        context.set_speed(30)
        time.sleep(1)
        return False
    else:
        return True

# двигаться назад
def go_back():
    if (context.get_speed() != -30):
        context.set_speed(-30)
        return False
    else:
        return True

# двигаться лагом (боком)
def go_side():
    context.set_side_speed(-50)
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

# всплыть на поверхность
def surface():
    context.set_depth(0)
    mur.set_motor_power(2, 50)
    mur.set_motor_power(3, 50)
    time.sleep(5)
    return True


# повернуться к стене
def turn_to_wall():
    if (context.get_yaw() != -90):
        context.set_yaw(-90)
        return False
    else:
        return True
        

# функции для стабилизации по определенной фигуре

def stabilize_on_shape(target_shape):
    while True:
        shape, pos = detect_shape(mur.get_image_front(),colors[MY_COLOR])
        if shape == target_shape:
            if not stabilize_x_y(pos[0], pos[1]):
                context.process()
                context.time += 1
            else:
                return True

def stabilize_on_circle():
    return stabilize_on_shape('circle')

def stabilize_on_triangle():
    return stabilize_on_shape('triangle')

def stabilize_on_square():
    return stabilize_on_shape('square')



# цикл выполнения подзадач

def do_loop_task(tasks):
    for task in tasks:
        print('- ', task.__name__, '[{}]'.format(context.time))
        while not task():
            context.process()
            context.time += 1

'''
для более точного определения объектов, форма будет определяться не по одному кадру, а по присутствию фигуры на нескольких кадрах подряд. данный словарь хранит последнюю фигуру, которая была определена, а также количество кадров, в течении которых она наблюдалась.
'''

context.shape_counter = {
    'shape': None,
    'counter': 0,
}

# поиск фигур
def Roma_look_for_picture():
    shape, pos = detect_shape(mur.get_image_front(),colors['dark_red'])

    if shape and abs(cam_w - pos[0]) < 120:
        stabilize_y(pos[1])

    if shape and abs(cam_w/2 - pos[0]) < 50:
        if context.shape_counter['shape'] == shape:
            context.shape_counter['counter'] += 1
        else:
            context.shape_counter['counter'] = 0

        context.shape_counter['shape'] = shape

    if context.shape_counter['counter'] >= 15:
        print('found', shape)
        context.shape_counter['counter'] = 0
        if shape == 'circle':
            Roma_stabilize_on_circle(color)
            print('circle')
            return True
        elif shape == 'triangle':
            do_loop_task(touch_picture_task)
        elif shape == 'square':
            print("square")

    return False

def Roma_look_for_picture_yellow():
    shape, pos = detect_shape(mur.get_image_bottom(),colors['yellow'])

    if shape and abs(cam_w - pos[0]) < 120:
        stabilize_y(pos[1])

    if shape and abs(cam_w/2 - pos[0]) < 50:
        if context.shape_counter['shape'] == shape:
            context.shape_counter['counter'] += 1
        else:
            context.shape_counter['counter'] = 0

        context.shape_counter['shape'] = shape

    if context.shape_counter['counter'] >= 15:
        print('found', shape)
        context.shape_counter['counter'] = 0
        if shape == 'circle':
            Roma_stabilize_on_circle(color)
            print('circle')
            return True
      

    return False
    
def Roma_look_for_picture_green():
    shape, pos = detect_shape(mur.get_image_bottom(),colors['green'])

    if shape and abs(cam_w - pos[0]) < 120:
        stabilize_y(pos[1])

    if shape and abs(cam_w/2 - pos[0]) < 50:
        if context.shape_counter['shape'] == shape:
            context.shape_counter['counter'] += 1
        else:
            context.shape_counter['counter'] = 0

        context.shape_counter['shape'] = shape

    if context.shape_counter['counter'] >= 15:
        print('found', shape)
        context.shape_counter['counter'] = 0
        if shape == 'circle':
            Roma_stabilize_on_circle(color)
            print('circle')
            return True
            

    return False
############################################


# поиск на изображении контура по цвету
def find_contours(image, color_low, color_high, approx = cv.CHAIN_APPROX_SIMPLE):
    hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv_image, color_low, color_high)
    #print("working")
    cv.imshow('result', mask)
    cv.waitKey(3000)
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, approx)
    return contours

 


def Roma_stabilize_on_circle():
    image = mur.get_image_front()
    contours = find_contours(image, colors[MY_COLOR][0], colors[MY_COLOR][1], cv.CHAIN_APPROX_SIMPLE)
    biggest_shape = None
    biggest_shape_pos = (0, 0)
    biggest_area = 0
    if contours:
        for contour in contours:
            area = cv.contourArea(contour)
            length = cv.arcLength(contour, True)

            if (area < 250):
                continue
            
            (circle_x, circle_y), circle_radius = cv.minEnclosingCircle(contour)
            circle_area = circle_radius ** 2 * math.pi
            circle_length = circle_radius * 2 * math.pi

            #print("area=",area," circle_area=",circle_area," length=",length," circle_length=",circle_length)
            if abs(area/circle_area - 1) < 0.05 and abs(length/circle_length - 1) < 0.05:
                #print("Found Circle !")
                return stabilize_x_y_bottom(circle_x, circle_y)
                
    return False

def Roma_stabilize_on_circle_yellow():
    image = mur.get_image_bottom()
    contours = find_contours(image, colors[MY_COLOR_YELLOW][0], colors[MY_COLOR_YELLOW][1], cv.CHAIN_APPROX_SIMPLE)
    biggest_shape = None
    biggest_shape_pos = (0, 0)
    biggest_area = 0
    if contours:
        for contour in contours:
            area = cv.contourArea(contour)
            length = cv.arcLength(contour, True)

            if (area < 250):
                continue
            
            (circle_x, circle_y), circle_radius = cv.minEnclosingCircle(contour)
            circle_area = circle_radius ** 2 * math.pi
            circle_length = circle_radius * 2 * math.pi

            #print("area=",area," circle_area=",circle_area," length=",length," circle_length=",circle_length)
            if abs(area/circle_area - 1) < 0.05 and abs(length/circle_length - 1) < 0.05:
                #print("Found Circle !")
                return stabilize_x_y_bottom(circle_x, circle_y)
                
    return False
    
def Roma_stabilize_on_circle_green():
    image = mur.get_image_bottom()
    contours = find_contours(image, colors[MY_COLOR_GREEN][0], colors[MY_COLOR_GREEN][1], cv.CHAIN_APPROX_SIMPLE)
    biggest_shape = None
    biggest_shape_pos = (0, 0)
    biggest_area = 0
    if contours:
        for contour in contours:
            area = cv.contourArea(contour)
            length = cv.arcLength(contour, True)

            if (area < 250):
                continue
            
            (circle_x, circle_y), circle_radius = cv.minEnclosingCircle(contour)
            circle_area = circle_radius ** 2 * math.pi
            circle_length = circle_radius * 2 * math.pi

            #print("area=",area," circle_area=",circle_area," length=",length," circle_length=",circle_length)
            if abs(area/circle_area - 1) < 0.05 and abs(length/circle_length - 1) < 0.05:
                #print("Found Circle !")
                return stabilize_x_y(circle_x, circle_y)
    if mur.get_depth() < 2.5:
        d = mur.get_depth() + 0.1
        context.set_depth(d)
    return False


def Roma_go_deep():
    if mur.get_depth() < 2.5:
        d = mur.get_depth() + 0.3
        context.set_depth(d)
        return Roma_stabilize_on_circle()
        #Roma_look_for_picture()
    else:
        return True
   

def bomb_korzina():
    while not Roma_look_for_picture_yellow() or not Roma_look_for_picture_green():
        go_by_truba()
        
    if Roma_look_for_picture_green():
        ount_of_korzinas_alarms += 1
        Roma_stabilize_on_circle__green()
    else: 
        ount_of_korzinas_alarms += 2
        Roma_stabilize_on_circle__yellow()
    return True


def go_by_truba():
    image = mur.get_image_bottom()
    contours = find_contours(image, colors[MY_COLOR_TRUBA][0], colors[MY_COLOR_TRUBA][1], cv.CHAIN_APPROX_SIMPLE)
    if(contours):
        a = find_rectangle_contour_angle(contours)
        print(a)
        context.set_yaw(a)
        context.set_speed(10)
    else:
        context.set_speed(10)
    return True
    
def escape():
    
    context.set_speed(0)
    time.sleep(0.05)
    
    if count_of_korzinas_alarms % 2 == 0:  # всплытие по часовой стрелке
        context.set_speed_round(100)
        
    else:
        context.set_speed_round(-100)
        
    context.set_depth(0)
    
    
    return True
    
# основной код программы
if __name__ == "__main__":

    context.set_depth(3.0)
    context.set_yaw(0.0)
         
    ######cv.namedWindow("result")

    # определим подзадачи, которые требуется решить
    initial_position = (
        wait,
        go_by_truba,
        wait,
        wait,
        )
    iinitial_position = (
        wait,
        bomb_korzina,
        go_by_truba,
        wait,
        bomb_korzina,
        go_by_truba,
        wait,
        bomb_korzina,
        go_by_truba,
        wait,
        bomb_korzina,
        go_by_truba,
        wait,
        bomb_korzina,
        wait,
       #     surface_20cm,
       # turn_to_wall,
#        wait,
#        stop,
#        stabilize,
#        wait_short,
#        stop,
#        wait,
        #Roma_look_for_picture,
        #Roma_go_deep,
        #Roma_stabilize_on_circle,
        #wait,
        #go_side,
        
       
    )

    main_task = (

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
    