import numpy as np

class Car():
    def __init__(self,car_locate,car_lane,car_type):
        self.car_locate = car_locate               # 车辆位置(坐标以车头为锚点)
        self.car_lane = car_lane                   # 车辆所在车道号
        self.car_acc = 0                           # 车辆加速度，初始置0
        self.car_speed = 0                         # 车速，初始置0
        self.car_lc = -1                           # 车辆换道标号，记录车辆每次即将变向的车道号，初始置-1
        self.car_type = car_type                   # 车辆类型，人工驾驶车置1，智能网联车置2
        self.outofrange = 0

    def set_car_locate(self,car_locate):
        self.car_locate = car_locate

    def set_car_lane(self,car_lane):
        self.car_lane = car_lane
    
    def set_car_lc(self,car_lc):
        self.car_lc = car_lc
    
    def set_car_speed(self,car_speed):
        self.car_speed = car_speed

    def set_car_acc(self,car_acc):
        self.car_acc = car_acc