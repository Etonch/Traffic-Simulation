class Road():
    def __init__(self,lane_type,lane_length = 6000,lane_num = 4):
        self.lane_length = lane_length
        self.lane_num = lane_num
        self.lane_type = [int(lane_type[i]) for i in range(self.lane_num)]

    def set_lane_num(self,lane_num):
        self.lane_num = lane_num   

    def set_lane_type(self,lane_type):
        self.lane_type = [int(lane_type[i]) for i in range(self.lane_num)]


if __name__=='__main__':
    r = Road('2001')
    print(r.lane_type)