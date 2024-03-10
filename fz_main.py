from statistics import mean
from car import Car
from road import Road
import random
import numpy as np
import pandas as pd
import math
import copy
import matplotlib.pyplot as plt
import matplotlib
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
import json

class Automata(Car,Road):
    def __init__(self,lane_type,lane_length,lane_num,max_speed,P_cav,car_num):
        Road.__init__(self,lane_type,lane_length,lane_num)
        self.car_length = 15
        self.max_speed = max_speed
        self.P_cav = P_cav
        self.car_num = car_num
        self.timer = 0
        self.cav_num = int(self.car_num*self.P_cav)
        self.mv_num = self.car_num - self.cav_num
        self.carbox = []
        for i in range(self.lane_num):
            self.carbox.append([])
        self.changlaneswitch = True
        self.flux = np.zeros(4)
        self.lane_speed = np.zeros(4)
        self.type_speed = np.zeros(2)
        self.whole_speed = 0
        self.changein = np.zeros(4)
        self.changeout = np.zeros(4)
    
    def get_cars_locate(self):
        output = []
        for lane in self.carbox:
            output.append(np.array([car.car_locate for car in lane]))
        return output

    def get_cars_v(self):
        output = []
        for lane in self.carbox:
            output.append(np.array([car.car_speed for car in lane]))
        return output

    def get_mean_speed(self):
        carsvbox = self.get_cars_v()
        output = np.array([])
        counter = 0
        adder = 0.0
        for lane in carsvbox:
            if lane.size != 0:
                value = lane.mean()
                counter += 1
                output = np.append(output, value)
                adder += value
            else:
                output = np.append(output, -1)
        if counter != 0:
            whole = adder/counter
        else:
            whole = None
        return output, whole
    
    def __timer(self):
        self.timer += 1

    def change_switch(self,flag):
        self.changlaneswitch = flag
  
    #车辆初始化
    def car_init_distribute(self):
        if self.P_cav >= 0.6:
            potential_loc = []
            ret_loc = []
            #cav初始化
            for y in range(self.lane_num):
                if self.lane_type[y] == 2 or self.lane_type[y] == 0:
                    for x in range(int(self.lane_length/self.car_length)):               
                        potential_loc.append([x*self.car_length, y])  
            cav_location = random.sample(potential_loc, self.cav_num)

            #mv初始化
            potential_loc=[]
            for y in range(self.lane_num):
                # 判断类型，mv只能生成在普通车道和mv专用道
                if self.lane_type[y] == 1 or self.lane_type[y] == 0:
                    for x in range(int(self.lane_length/self.car_length)):               
                        potential_loc.append([x*self.car_length, y])

            for loc in potential_loc:
                if loc not in cav_location:
                    ret_loc.append(loc)
            mv_location = random.sample(ret_loc, self.mv_num)

        else:
            potential_loc = []
            ret_loc = []
            #mv初始化
            for y in range(self.lane_num):
                if self.lane_type[y] == 1 or self.lane_type[y] == 0:
                    for x in range(int(self.lane_length/self.car_length)):               
                        potential_loc.append([x*self.car_length, y])  
            mv_location = random.sample(potential_loc, self.mv_num)

            #cav初始化
            potential_loc=[]
            for y in range(self.lane_num):
                # 判断类型，mv只能生成在普通车道和mv专用道
                if self.lane_type[y] == 2 or self.lane_type[y] == 0:
                    for x in range(int(self.lane_length/self.car_length)):               
                        potential_loc.append([x*self.car_length, y])

            for loc in potential_loc:
                if loc not in mv_location:
                    ret_loc.append(loc)
            cav_location = random.sample(ret_loc, self.cav_num)

        #创建carbox对象，顺序存储车辆
        for y in range(self.lane_num):
            for x in range(int(self.lane_length/self.car_length)):
                if [x*self.car_length, y] in cav_location:
                    self.carbox[y].append(Car(car_locate=x*self.car_length,car_lane=y,car_type=2))
                elif [x*self.car_length, y] in mv_location:
                    self.carbox[y].append(Car(car_locate=x*self.car_length,car_lane=y,car_type=1))
        return self.carbox

    def update_speed(self,opcar,frontCar,front2Car, i):
        def mv_update_speed(opcar,frontCar,front2Car,i):
            vmax = self.max_speed
            pa, pb, pc= 0.85, 0.55, 0.1
            vc, alpha = 30, 10
            T = 1.8
            amv, bmax = 1, 3
            gmv_mv, gmv_cav = 1.8, 2.4
            brand = 1
            if i != 1 and i != 0:
                d = opcar.car_locate - frontCar.car_locate - self.car_length
                dl = frontCar.car_locate - front2Car.car_locate - self.car_length
            elif i == 1:
                d = opcar.car_locate - frontCar.car_locate - self.car_length
                dl = frontCar.car_locate - front2Car.car_locate + self.lane_length - self.car_length
            elif i == 0:
                d = opcar.car_locate - frontCar.car_locate + self.lane_length - self.car_length
                dl = frontCar.car_locate - front2Car.car_locate - self.car_length
            
            v = opcar.car_speed
            vl = frontCar.car_speed
            vanti = min(dl,vl+amv,vmax)
            if d<0:
                print(d,opcar.car_locate,frontCar.car_locate,"d<0",opcar.car_speed,frontCar.car_speed)
            try:
                vsafe = int(-bmax+math.sqrt(bmax*bmax+vl*vl+2*bmax*d))
            except ValueError:
                print(d,opcar.car_locate,frontCar.car_locate,"mvdistanceerror",opcar.car_speed,frontCar.car_speed)
            g = gmv_mv if frontCar.car_type == 1 else gmv_cav
            danti = min((d+vanti+self.car_length)/(1+g),d+vanti)
            v = int(min(v+amv,vmax,vsafe,danti))
            if v <= danti / T:  # MV随机慢化
                p = np.array([pc, 1 - pc])
            else:
                p = np.array([pc + pa / (1 + math.e ** (alpha * (vc - v))),
                        1- pc - pa / (1 + math.e ** (alpha * (vc - v)))])
            v = np.random.choice([max(v - brand, 0), v], p=p.ravel())
            return v

        def cav_update_speed(opcar,frontCar,front2Car,i):
            vmax = self.max_speed
            k1,k2,kp,kd = 0.14,0.90,0.45,0.25
            ta,tc = 1.1,0.6
            t = 0.1
            amax, bmax = 3,3
            gcav_cav,gcav_mv = 0,0.9
            if i != 0 and i != 1:
                d = opcar.car_locate - frontCar.car_locate - self.car_length
                dl = frontCar.car_locate - front2Car.car_locate - self.car_length
            elif i == 1:
                d = opcar.car_locate - frontCar.car_locate - self.car_length
                dl = frontCar.car_locate - front2Car.car_locate + self.lane_length - self.car_length
            elif i == 0:
                d = opcar.car_locate - frontCar.car_locate + self.lane_length - self.car_length
                dl = frontCar.car_locate - front2Car.car_locate - self.car_length
            v = opcar.car_speed
            vl = frontCar.car_speed
            al = frontCar.car_acc
            if d<0:
                print(d,opcar.car_locate,frontCar.car_locate,"d<0",opcar.car_speed,frontCar.car_speed,i,frontCar.outofrange)
            try:            
                vsafe = int(-bmax*t+math.sqrt(bmax*bmax*t*t+vl*vl+2*bmax*d))
            except ValueError:
                print(d,opcar.car_locate,frontCar.car_locate,"cavdistanceerror",opcar.car_speed,frontCar.car_speed,i,frontCar.outofrange)
            if frontCar.car_type == 1:
                g = gcav_mv
                acav = int(max(-bmax,min(
                    amax, k1 * (d - ta * opcar.car_speed) + k2 * (vl - opcar.car_speed))))
                vanti = min(dl, vl+al, vmax)
            else:
                g = gcav_cav
                acav = int(max(-bmax, min(amax, kp * (d - tc * opcar.car_speed) + kd * (
                    vl - opcar.car_speed - tc * opcar.car_acc))))
                vanti = min(dl, vl+al, vmax)
            danti = min((d+vanti+self.car_length)/(1+g),d+vanti)
            v = int(max(min(opcar.car_speed + acav, vmax, danti, vsafe),0))
            return v
        
        if opcar.car_type == 1:
            v = mv_update_speed(opcar,frontCar,front2Car,i)
        elif opcar.car_type == 2:
            v = cav_update_speed(opcar,frontCar,front2Car,i)
        return v

    def __change_lane(self):
        def find_d(opcar,frontCar,i):
            if i != 0:
                d = opcar.car_locate - frontCar.car_locate
            else:
                d = opcar.car_locate - frontCar.car_locate + self.lane_length
            return d

        def find_d_other(opcar,tar_lane):
            loc = opcar.car_locate
            tar_carlocate = self.get_cars_locate()[tar_lane]
            tar_carfront = tar_carlocate[tar_carlocate < loc]
            tar_carback = tar_carlocate[tar_carlocate >= loc]
            if len(tar_carfront) != 0:
                d_other = opcar.car_locate - tar_carfront[len(tar_carfront)-1]- self.car_length
                v_other = carbox[tar_lane][len(tar_carfront)-1].car_speed
            else:
                d_other = opcar.car_locate - tar_carback[len(tar_carback)-1] - self.car_length + self.lane_length
                v_other = carbox[tar_lane][len(carbox[tar_lane])-1].car_speed
            if len(tar_carback) != 0:
                d_other_back = tar_carback[0] - opcar.car_locate - self.car_length
                v_other_back = carbox[tar_lane][len(tar_carfront)].car_speed
            else:
                d_other_back = tar_carfront[0] - opcar.car_locate - self.car_length + self.lane_length
                v_other_back = carbox[tar_lane][0].car_speed
            return d_other,d_other_back,v_other,v_other_back
        
        def find_vl(frontCar):
            return frontCar.car_speed
        
        def find_num(opcar,lane):
            carsLocate = self.get_cars_locate()[lane]
            carsdistance = opcar.car_locate - carsLocate
            carsspeed = self.get_cars_locate()[lane]
            if opcar.car_locate - 300 >= 0:
                incrflag = (carsdistance > 0) & (carsdistance <= 300)
                incrdis = carsdistance[incrflag]
                incrspeed = carsspeed[incrflag]
                num = incrdis.size
                meanspeed = incrspeed.mean()
            else:
                incrflag = (carsLocate < opcar.car_locate) | (0 < opcar.car_locate + self.lane_length - carsLocate) & (opcar.car_locate + self.lane_length - carsLocate <= 300)
                incrdis = carsdistance[incrflag]
                incrspeed = carsspeed[incrflag]
                num = incrdis.size
                meanspeed = incrspeed.mean()
            return num,meanspeed
                
        def find_num_other(opcar,tar_lane):
            try:
                carsLocate = self.get_cars_locate()[tar_lane]
            except IndexError:
                print(opcar.car_lane,tar_lane)
            carsdistance = opcar.car_locate - carsLocate
            carsspeed = self.get_cars_locate()[tar_lane]
            if opcar.car_locate - 300 >= 0:
                incrflag = (carsdistance > 0) & (carsdistance <= 300)
                incrdis = carsdistance[incrflag]
                incrspeed = carsspeed[incrflag]
                num = incrdis.size
                meanspeed = incrspeed.mean()
            else:
                incrflag = (carsLocate < opcar.car_locate) | (0 < opcar.car_locate + self.lane_length - carsLocate) & (opcar.car_locate + self.lane_length - carsLocate <= 300)
                incrdis = carsdistance[incrflag]
                incrspeed = carsspeed[incrflag]
                num = incrdis.size
                meanspeed = incrspeed.mean()
            return num,meanspeed
        
        def mv_check_change(opcar,d,d_other,d_other_back,v_other_back):
            if d < min(opcar.car_speed + 1, self.max_speed) and d_other > d and d_other_back >= v_other_back - opcar.car_speed:
                return True
            else:
                return False

        def mv_change_lane(opcar,frontCar,cur_lane,i):
            d = find_d(opcar,frontCar,i)
            if cur_lane == 0:
                if self.lane_type[1] == 0 or self.lane_type[1] == 1:       
                    d_other, d_other_back, v_other, v_other_back= find_d_other(opcar, tar_lane = 1)
                    if mv_check_change(opcar,d,d_other,d_other_back,v_other_back):
                        opcar.car_lc = np.random.choice([1,-1],p=[0.2,0.8])
            elif cur_lane == self.lane_num - 1:
                if self.lane_type[self.lane_num - 2] == 0 or self.lane_type[self.lane_num - 2] == 1:
                    d_other, d_other_back, v_other, v_other_back = find_d_other(opcar, tar_lane = self.lane_num - 2)
                    if mv_check_change(opcar,d,d_other,d_other_back,v_other_back):
                        opcar.car_lc = np.random.choice([self.lane_num - 2,-1],p=[0.2,0.8])
            else:
                if (self.lane_type[cur_lane-1] == 0 or self.lane_type[cur_lane-1] == 1) and (self.lane_type[cur_lane+1] == 0 or self.lane_type[cur_lane+1] == 1):
                    d_other, d_other_back, v_other, v_other_back = find_d_other(opcar, tar_lane = cur_lane-1)
                    d_other2, d_other_back2, v_other2, v_other_back2 = find_d_other(opcar, tar_lane = cur_lane+1)
                    if mv_check_change(opcar,d,d_other,d_other_back,v_other_back) and mv_check_change(opcar,d,d_other,d_other_back,v_other_back):
                        if d_other > d_other2:
                            opcar.car_lc = np.random.choice([cur_lane-1,-1],p=[0.2,0.8])
                        elif d_other < d_other2:
                            opcar.car_lc = np.random.choice([cur_lane+1,-1],p=[0.2,0.8])
                        elif d_other == d_other2:
                            opcar.car_lc = np.random.choice([cur_lane+1,cur_lane-1,-1],p=[0.1,0.1,0.8])
                    elif mv_check_change(opcar,d,d_other,d_other_back,v_other_back):
                        opcar.car_lc = np.random.choice([cur_lane-1,-1],p=[0.2,0.8])
                    elif mv_check_change(opcar,d,d_other2,d_other_back2,v_other_back2):
                        opcar.car_lc = np.random.choice([cur_lane+1,-1],p=[0.2,0.8])

        def cav_check_change(opcar,d,vl,d_other,v_other,d_other_back,v_other_back,num,num_other,vmean_l,vmean_other_l):
            if d + vl < min(opcar.car_speed + 1, self.max_speed) and d_other + v_other > d + vl and d_other_back >= v_other_back - opcar.car_speed and (num < num_other < 10 or vmean_other_l > vmean_l):
                return True
            else:
                return False
        
        def cav_change_lane(opcar,frontCar,cur_lane,i):
            d = find_d(opcar,frontCar,i)
            vl = find_vl(frontCar)
            num,vmean_l = find_num(opcar,cur_lane)
            if cur_lane == 0:
                if self.lane_type[1] == 0 or self.lane_type[1] == 1:       
                    d_other, d_other_back, v_other, v_other_back= find_d_other(opcar, tar_lane = 1)
                    num_other,vmean_other_l = find_num_other(opcar,1)
                    if cav_check_change(opcar,d,vl,d_other,v_other,d_other_back,v_other_back,num,num_other,vmean_l,vmean_other_l):
                        opcar.car_lc = 1
            elif cur_lane == self.lane_num - 1:
                if self.lane_type[self.lane_num - 2] == 0 or self.lane_type[self.lane_num - 2] == 1:
                    d_other, d_other_back, v_other, v_other_back= find_d_other(opcar, tar_lane = self.lane_num - 2)
                    num_other,vmean_other_l = find_num_other(opcar,self.lane_num-2)
                    if cav_check_change(opcar,d,vl,d_other,v_other,d_other_back,v_other_back,num,num_other,vmean_l,vmean_other_l):
                        opcar.car_lc = self.lane_num - 1
            else:
                if (self.lane_type[cur_lane-1] == 0 or self.lane_type[cur_lane-1] == 1) and (self.lane_type[cur_lane+1] == 0 or self.lane_type[cur_lane+1] == 1):
                    d_other, d_other_back, v_other, v_other_back = find_d_other(opcar, tar_lane = cur_lane-1)
                    d_other2, d_other_back2, v_other2, v_other_back2 = find_d_other(opcar, tar_lane = cur_lane+1)
                    num_other,vmean_other_l = find_num_other(opcar,cur_lane-1)
                    num_other2,vmean_other_l2 = find_num_other(opcar,cur_lane+1)
                    if cav_check_change(opcar,d,vl,d_other,v_other,d_other_back,v_other_back,num,num_other,vmean_l,vmean_other_l) and cav_check_change(opcar,d,vl,d_other2,v_other2,d_other_back2,v_other_back2,num,num_other2,vmean_l,vmean_other_l2):
                        if num_other > num_other2:
                            opcar.car_lc = cur_lane-1
                        elif num_other < num_other2:
                            opcar.car_lc = cur_lane+1
                        elif num_other == num_other2:
                            opcar.car_lc = np.random.choice([cur_lane+1,cur_lane-1],p=[0.5,0.5])
                    elif cav_check_change(opcar,d,vl,d_other,v_other,d_other_back,v_other_back,num,num_other,vmean_l,vmean_other_l):
                        opcar.car_lc = cur_lane-1
                    elif mv_check_change(opcar,d,d_other2,d_other_back2,v_other_back2):
                        opcar.car_lc = cur_lane+1

        def no_collision(opcar,tar_lane):
            carsLocate = self.get_cars_locate()[tar_lane]
            carsflag = abs(opcar.car_locate - carsLocate)>15
            if carsflag.all():
                return True
            else:
                return False

        
        if not self.changlaneswitch:
            return None
        for lane in range(self.lane_num):
            for i in range(len(self.carbox[lane])):
                opcar = self.carbox[lane][i]
                frontCar = self.carbox[lane][i-1]
                if opcar.car_type == 1:
                    mv_change_lane(opcar,frontCar,lane,i)
                elif opcar.car_type == 2:
                    cav_change_lane(opcar,frontCar,lane,i)
        for lane in range(self.lane_num):
            i = 0
            while i < len(self.carbox[lane]):
                opcar = self.carbox[lane][i]
                if opcar.car_lc != -1: 
                    if no_collision(opcar,opcar.car_lc):
                        self.changeout[opcar.car_lane] += 1
                        self.changein[opcar.car_lc] += 1
                        Tempcar = copy.deepcopy(opcar)
                        Tempcar.car_lc = -1
                        lc_carlocate = self.get_cars_locate()[opcar.car_lc]
                        carfront = lc_carlocate[lc_carlocate < opcar.car_locate]
                        self.carbox[opcar.car_lc].insert(len(carfront),Tempcar)
                        self.carbox[lane].pop(i)
                        i -= 1
                    else:
                        opcar.car_lc = -1
                i += 1

    def update_status(self):
        self.__change_lane()
        self.__timer()

        #速度并行更新，加速度实时更新
        for lane in range(self.lane_num):
            TempSpeedSaver = []
            TempLocateSaver = []
            if len(self.carbox[lane]) <= 2:
                for i in range(0, len(self.carbox[lane])):
                    opcar = self.carbox[lane][i]
                    opcar.car_speed = min(opcar.car_speed + 3,self.max_speed)
            if len(self.carbox[lane]) >= 2:
                for i in range(0, len(self.carbox[lane])):
                    opcar = self.carbox[lane][i]
                    opcar.car_lane = lane
                    frontCar = self.carbox[lane][i-1]
                    front2Car = self.carbox[lane][i-2]
                    speed = self.update_speed(opcar, frontCar, front2Car, i)
                    self.carbox[lane][i].car_acc = speed - self.carbox[lane][i].car_speed
                    TempSpeedSaver.append(speed)
                    TempLocateSaver.append(opcar.car_locate - speed)
            
            #处理循环边界问题
            Tempcar = []           #Tempcar列表，用于存储超出循环边界的车辆
            checkdis1 = TempLocateSaver[1] - TempLocateSaver[0]
            checkdis2 = TempLocateSaver[0] - TempLocateSaver[len(self.carbox[lane])-1] +self.lane_length
            if checkdis1 >= 15 and checkdis2 >= 15:
                checkflag = True
            else:
                checkflag = False
                print('timer:',self.timer,'circulate boundary problem solved')
            if checkflag:
                for i in range(len(TempSpeedSaver)):
                    opcar = self.carbox[lane][i]
                    opcar.car_speed = TempSpeedSaver[i]
                    if opcar.car_locate >= 0:
                        opcar.car_locate = TempLocateSaver[i]
                        opcar.outofrange = 0
                    else:           
                        opcar.car_locate = TempLocateSaver[i] + self.lane_length
                        opcar.outofrange = 1
                        Tempcar.append(copy.deepcopy(opcar))
                        if self.timer >= 1000:
                            self.flux[lane] += 1
                if Tempcar:
                    for car in Tempcar:
                        carbox[lane].pop(0)
                        carbox[lane].append(car)
            else:
                i = 0
                while not checkflag:
                    opcar = self.carbox[lane][i]
                    opcar.car_lane = lane
                    frontCar = self.carbox[lane][i-1]
                    front2Car = self.carbox[lane][i-2]
                    speed = self.update_speed(opcar, frontCar, front2Car, i)
                    self.carbox[lane][i].car_acc = speed - self.carbox[lane][i].car_speed
                    TempSpeedSaver[i] = speed
                    TempLocateSaver[i] = opcar.car_locate - speed
                    checkdis1 = TempLocateSaver[1] - TempLocateSaver[0]
                    checkdis2 = TempLocateSaver[0] - TempLocateSaver[len(self.carbox[lane])-1] +self.lane_length
                    if checkdis1 >= 15 and checkdis2 >= 15:
                        checkflag = True
                    i = i + 1
        if self.timer >= 1000:
            output,whole = self.get_mean_speed()
            for i in range(self.lane_num):
                self.lane_speed[i] += output[i]
            self.whole_speed += whole

    def init_draw(self):
        N = 30
        cmap = matplotlib.cm.get_cmap('viridis')
        cmap_values = np.linspace(0., 1., N)
        colors = cmap(cmap_values)
        self.colors_rgb = ['#{0:02x}{1:02x}{2:02x}'.format(int(255*a), int(255*b), int(255*c)) for a, b, c, _ in colors]
        

    def draw(self):       
        carslocate = self.get_cars_locate()[0]
        carstime = np.zeros(len(carslocate))
        carsspeed = self.get_cars_v()[0]
        speed_color=[]
        for i in range(len(carsspeed)):
            if carsspeed[i] >= 30:
                speed_color.append(self.colors_rgb[29])
            else:
                speed_color.append(self.colors_rgb[carsspeed[i]])
        for i in range(len(carslocate)):
            carstime[i] = self.timer
        plt.scatter(carstime, carslocate, c = speed_color, s = 0.1)

    def after_draw(self):
        #设置坐标轴范围
        plt.xlim((1000, 2000))
        plt.ylim((0, 6000))
        #设置坐标轴名称
        plt.xlabel('time')
        plt.ylabel('space')
    
        norm = matplotlib.colors.Normalize(vmin=0, vmax=30)
        cmap = matplotlib.cm.get_cmap('viridis')
        ticks = np.arange(0, 35, 5)
        cbaxes = inset_axes(plt.gca(), width="3%", height="80%", loc=2)
        cbar = matplotlib.colorbar.ColorbarBase(cbaxes, cmap=cmap, norm=norm, ticks=ticks)
        cbar.set_label('speed')
        cbar.ax.set_yticklabels(ticks, fontsize=12)


for car_num in range(1280,1441,80):
    for p in range(15,80,15):
        for j in range(1):
            fz = Automata("2000",6000,4,60,p/100,car_num)
            #fz.init_draw()
            carbox = fz.car_init_distribute()
            for i in range(4600):
                fz.update_status()
                #if i>=1000:
                    #fz.draw()
            #fz.after_draw()
            #plt.savefig('0000-'+str(p/10)+'-'+str(car_num)+'-'+str(j)+'.jpg')
            mean_lane_speed = fz.lane_speed/3600
            mean_whole_speed = np.array([fz.whole_speed/3600]*4)
            data = pd.DataFrame({
                'flux':fz.flux,
                'mean_speed':mean_lane_speed,
                'changein':fz.changein,
                'changeout':fz.changeout,
                'whole_mean_speed':mean_whole_speed
            })
            data.to_csv('2000-'+str(p/100)+'-'+str(car_num)+'-'+str(j)+'.csv')
