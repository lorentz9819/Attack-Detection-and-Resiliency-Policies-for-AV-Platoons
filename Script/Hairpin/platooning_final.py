import carla
import math
import numpy as np
from cloud import SafeCloud


def sign(number):
  if number>=0: return 1
  else: return -1

class PlatoonMember:
    def __init__(self, vehicle: carla.Vehicle):
        self.vehicle = vehicle
        tr = self.vehicle.get_transform()
        con = self.vehicle.get_control()
        vel = self.vehicle.get_velocity()
        self.x = tr.location.x
        self.y = tr.location.y
        self.z = tr.location.z
        self.yaw = tr.rotation.yaw
        self.speed = 3.6*math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        self.throttle = con.throttle
        self.steer = con.steer
        self.brake = con.brake
        self.cloud: SafeCloud = None
        self.waypoints = []

    def update_position(self):
        tr = self.vehicle.get_transform()
        con = self.vehicle.get_control()
        vel = self.vehicle.get_velocity()
        self.x = tr.location.x
        self.y = tr.location.y
        self.z = tr.location.z
        self.yaw = tr.rotation.yaw
        self.speed = 3.6*math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        self.throttle = con.throttle
        self.steer = con.steer
        self.brake = con.brake

    def get_pos(self):
        return self.x, self.y

    def get_speed(self):
        return self.speed

class Follower(PlatoonMember):
    def __init__(self, vehicle:carla.Vehicle):
        super().__init__(vehicle)
        self.speedGoal = 0.0
        self.last_t = 0.0
        self.big_dist = False
        self.leader_dist = 0.0
        self.safe_dist = 12.0
        self.override_brake = False
        self.leader: PlatoonMember = None


    def set_leading_vehicle(self, lead:PlatoonMember):
        if not self.leader:
            self.leader = lead

    def update_position(self):
        super().update_position()
        x = self.x
        y = self.y
        tx,ty = self.leader.get_pos()
        self.leader_dist = ((x-tx)**2 + (y-ty)**2)**(1/2)
        self.safe_dist = max(12.0, self.speed/2)
        if self.leader_dist > self.safe_dist: self.big_dist = True
        else: self.big_dist = False

    def connect_to_cloud(self, sc: SafeCloud):
        if not self.cloud:
            self.cloud = sc
        sc.add_members(self)

    def check_lidar(self,points):
        detection=False
        blinded=False
        danger_dist = self.speed/570
        for p in points:
            if p.point.x<max(3,danger_dist):
                if p.point.x < 0.5:
                    blinded = True
                    detection = False
                    print("Points = ", p.point.x)
                else:
                    detection=True
                    print("LiDAR triggered")
        self.override_brake=detection
        self.blinded=blinded

    def IMU_callback(self, sensor_data):
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)

    def add_waypoint(self, wp):
        self.waypoints.append(wp)

    def set_speed_goal(self, s):
        self.speedGoal = s

    def define_throttle(self, sg, ss):

        #--Rule 1-Speed bounds (Leader speed is faked)--
        if self.speed > 5:
            if self.speedGoal > 35:
                print("Abnormal behaviour: Faked speed")
                self.speedGoal = self.speed
                sg = ss
            elif self.speedGoal < 1:
                if not self.override_brake:
                    #Cio' significa che se il lidar e' blindato o se non rileva ostacoli, la regola non permette al veicolo di arrestarsi
                    print("Fake signal, the data are corrupted, override")
                    #print("SpeedGoal = ", self.speedGoal)
                    #print("Speed = ", self.speed)
                    self.speedGoal = self.speed
                    sg = ss

        delta = sg - ss
        t = b = 0.0
        if self.speedGoal>0.04:
            if (delta>0):
                boost = 0.4 if self.big_dist else 0.0
                t = (delta/10 + boost) if (delta/10 + boost) <= 1.0 else 1.0
            else:
                t = self.last_t
                if not self.big_dist:
                    t = 0.0
                    b = -delta/10 if -delta/10 <= 1.0 else 1.0
        else:
            if self.big_dist:
                t = 0.2
            else:
                b = 0.8
        self.last_t=t
        return t, b

    def define_steer(self, wp, x, y, ya):
        if self.speed<=0.03: return 0.0

        rl = wp
        fx = x
        fy = y
        rel_yaw = yaw = ya
        s=0.0
        delta = delta_x = delta_y = 1000
        rel_x = rel_y = 0
        for row in reversed(rl):
            lx = row[0]
            ly = row[1]
            lyaw = row[2]
            gyro = row[4]
            accx = row[5]
            accy = row[6]
            accz = row[7]

            #--Rule 2-Fake turn data (gyroscope and/or yaw data are faked)
            if (gyro > 20 or gyro < -40 or self.gyroscope[-1] > 20 or self.gyroscope[-1] < -40):
                print("Excessive turning angle")
                print("Self Gyroscope = ", self.gyroscope[-1])
                lyaw = yaw

            #--Rule 3a-Fake turn signal (the hairpin is to the left, during the turn the signal indicates to turn opposite (right))
            if yaw < 110 and self.gyroscope[-1] < -5 and (self.compass < 190 and self.compass > 0): #hairpin to the left, direction east
                if lyaw > yaw:
                    print("Error, wrong turn 1")
                    lyaw = yaw
            elif (yaw < -70 or yaw < 180) and self.gyroscope[-1] < -5 and (self.compass < 360 and self.compass > 170): #hairpin to the left, direction west
                if lyaw > yaw:
                    print("Error, wrong turn 2")
                    lyaw = yaw
            elif yaw < 20 and self.gyroscope[-1] < -5: #hairpin to the left, direction north
                if lyaw > yaw:
                    print("Error, wrong turn 3")
                    lyaw = yaw
            elif (yaw < -160 or yaw < 180) and self.gyroscope[-1] < -5: #hairpin to the left, direction south
                if lyaw > yaw:
                    print("Error, wrong turn 4")
                    lyaw = yaw

            #--Rule 3b-Fake turn signal (the hairpin is to the right, during the turn the signal indicates to turn opposite (left))
            if (yaw > 70 or yaw > -180) and self.gyroscope[-1] > 5 and (self.compass < 190 and self.compass > 0): #hairpin to the right, direction east
                if lyaw < yaw:
                    print("Error, wrong turn 5")
                    lyaw = yaw
            elif yaw > -110 and self.gyroscope[-1] > 5 and (self.compass < 360 and self.compass > 170): #hairpin to the right, direction west
                if lyaw < yaw:
                    print("Error, wrong turn 6")
                    lyaw = yaw
            elif yaw > -20 and self.gyroscope[-1] > 5: #hairpin to the right, direction north
                if lyaw < yaw:
                    print("Error, wrong turn 7")
                    lyaw = yaw
            elif (yaw > 160 or yaw > -180) and self.gyroscope[-1] > 5: #hairpin to the right, direction south
                if lyaw < yaw:
                    print("Error, wrong turn 8")
                    lyaw = yaw

            #--Rule 4-No steer input resilience
            if (gyro > 11 or gyro < -11) and (self.gyroscope[-1] < 0.01 and self.gyroscope[-1] > -0.01):
                print("The platoon leader is turning")
                print("Gyro = ", gyro)
                print("self.gyroscope = ", self.gyroscope[-1])
                lyaw = 180 * np.arctan(accz/np.sqrt(accx*accx + accz*accz))/np.pi


            #--Rule 5-Fake turn while going straight
            #if self.speed > 0.03:
            #    if ((lyaw > 95 and lyaw < 175) or (lyaw < 85 and lyaw > 5)) or ((lyaw < -5 and lyaw > -85) or (lyaw > 5 and lyaw < 85)) or ((lyaw < -95 and lyaw > -175) or (lyaw > -85 and lyaw < -5)) or ((lyaw < 175 and lyaw > 95) or (lyaw > -175 and lyaw < -95)):
            #        if gyro < 4 and gyro > -4 and self.gyroscope[-1] < 4 and self.gyroscope[-1] > -4:
            #            print("Error: The platoon leader is going straight")
                        #print("giroscopio = ", gyro)
                        #print("lyaw = ", lyaw)
                        #print("---------------------")
            #            lyaw = yaw

            toll = (0.1**2+0.1**2)**(1/2)
            diff = ((fx-lx)**2 + (fy-ly)**2)**(1/2)
            if diff <= toll:
                s=row[3]
                if lyaw<-90 and yaw>90:
                    corr=(lyaw-yaw+360)/60
                elif lyaw>90 and yaw<-90:
                    corr=(lyaw-yaw-360)/60
                else:
                    corr=(lyaw-yaw)/60
                if corr > 1.0: corr = 1.0
                elif corr < -1.0: corr = -1.0
                s+=corr
                if s>1.0:
                    s=1.0
                elif s<-1.0:
                    s=-1.0
                return s

            else:

                if diff < delta:
                    rel_y = ly
                    rel_x = lx
                    delta = diff
                    rel_yaw = lyaw
                    if abs(lx-fx) < delta_x:
                        delta_x = abs(lx-fx)
                    if abs(ly-fy) < delta_y:
                        delta_y = abs(ly-fy)
        lyaw = rel_yaw
        corr = (lyaw-yaw)/90

        if abs(lyaw)<=5: #going east
            s=(rel_y-fy)/10
            if s>1.0: s=1.0
            elif s<-1.0: s=-1.0
            if corr>1.0: corr = 1.0
            elif corr<-1.0: corr = -1.0
            s+=corr
        elif abs(lyaw)>=175: #going ovest
            if lyaw>90 and yaw<-90:
                corr-=4
                s=(fy-rel_y)/10
                if s>1.0: s=1.0
                elif s<-1.0: s=-1.0
                if corr>1.0: corr = 1.0
                elif corr<-1.0: corr = -1.0
                s+=corr
            elif lyaw<-90 and yaw>90:
                corr+=4
                s=(fy-rel_y)/10
                if s>1.0: s=1.0
                elif s<-1.0: s=-1.0
                if corr>1.0: corr = 1.0
                elif corr<-1.0: corr = -1.0
                s+=corr
            else:
                s=(fy-rel_y)/10
                if s>1.0: s=1.0
                elif s<-1.0: s=-1.0
                if corr>1.0: corr = 1.0
                elif corr<-1.0: corr = -1.0
                s+=corr
        elif lyaw<95 and lyaw>85: #going south
            s=(fx-rel_x)/10
            if s>1.0: s=1.0
            elif s<-1.0: s=-1.0
            if corr>1.0: corr = 1.0
            elif corr<-1.0: corr = -1.0
            s+=corr
        elif lyaw>-95 and lyaw<-85: #going north
            s=(rel_x-fx)/10
            if s>1.0: s=1.0
            elif s<-1.0: s=-1.0
            if corr>1.0: corr = 1.0
            elif corr<-1.0: corr = -1.0
            s+=corr

        else:
            if yaw>-180 and yaw<=-90:
                if sign(rel_x)==sign(fx): fx=-fx
                xs = (rel_x-fx)/20
                if xs > 1.0: xs = 1.0
                elif xs < -1.0: xs = -1.0

                if sign(rel_y)==sign(fy): rel_y=-rel_y
                ys = (fy-rel_y)/20
                if ys > 1.0: ys = 1.0
                elif ys < -1.0: ys = -1.0

            elif yaw>-90 and yaw<=0:
                if sign(rel_x)==sign(fx): fx=-fx
                xs = (rel_x-fx)/20
                if xs > 1.0: xs = 1.0
                elif xs < -1.0: xs = -1.0

                if sign(rel_y)==sign(fy): fy=-fy
                ys = (rel_y-fy)/20
                if ys > 1.0: ys = 1.0
                elif ys < -1.0: ys = -1.0

            elif yaw>0 and yaw<=90:
                if sign(rel_x)==sign(fx): rel_x=-rel_x
                xs = (fx-rel_x)/20
                if xs > 1.0: xs = 1.0
                elif xs < -1.0: xs = -1.0

                if sign(rel_y)==sign(fy): fy=-fy
                ys = (rel_y-fy)/20
                if ys > 1.0: ys = 1.0
                elif ys < -1.0: ys = -1.0

            elif yaw>90 and yaw<=180:
                if sign(rel_x)==sign(fx): rel_x=-rel_x
                xs = (fx-rel_x)/20
                if xs > 1.0: xs = 1.0
                elif xs < -1.0: xs = -1.0

                if sign(rel_y)==sign(fy): rel_y=-rel_y
                ys = (fy-rel_y)/20
                if ys > 1.0: ys = 1.0
                elif ys < -1.0: ys = -1.0

            s = (xs+ys)
            if s>1.0: s=1.0
            elif s<-1.0: s=-1.0

            if lyaw>90 and yaw<-90:
                corr=(lyaw-yaw-360)/60
                if corr>1.0: corr = 1.0
                elif corr<-1.0: corr = -1.0
                s+=corr

            elif lyaw<-90 and yaw>90:
                corr=(lyaw-yaw+360)/60
                if corr>1.0: corr = 1.0
                elif corr<-1.0: corr = -1.0
                s+=corr

            else:
                corr=(lyaw-yaw)/60
                if corr>1.0: corr = 1.0
                elif corr<-1.0: corr = -1.0
                s+=corr

        if s>1.0: s=1.0
        elif s<-1.0: s=-1.0
        return s

    def move(self):
        self.update_position()
        self.cloud.retrieve_check_data(self)
        self.control()

    def control(self):
        t = s = b = 0.0
        if self.override_brake: control = carla.VehicleControl(throttle=0, steer=0, brake=1.0)
        else:
            wp = self.waypoints
            x = self.x
            y = self.y
            ya = self.yaw
            s = self.define_steer(wp, x, y, ya)
            sg = self.speedGoal
            ss = self.speed
            t, b = self.define_throttle(sg, ss)
            control = carla.VehicleControl(throttle=t, steer=s, brake=b)
            cond, ct, cs, cb = self.cloud.check_action(self, t,s,b)
            #if not cond:
            #    print("ANOMALY DETECTED",t,ct,s,cs,b,cb)
            #    control = carla.VehicleControl(throttle=ct, steer=cs, brake=cb)
        self.vehicle.apply_control(control)

class Leader(PlatoonMember):
    def __init__(self, vehicle: carla.Vehicle):
        super().__init__(vehicle)
        self.followers = []

    def connect_to_cloud(self, sc: SafeCloud):
        if not self.cloud:
            self.cloud = sc
        sc.set_leader(self)

    def addFollower(self, f: Follower):
        self.followers.append(f)

    def IMU_callback(self, sensor_data):
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)

    def move(self):
        self.update_position()
        self.update_follower()

    def update_follower(self):
        #print("x = ", self.x)
        #print("y = ", self.y)
        #print("Yaw = ", self.yaw)
        #print("-------------")
        self.waypoints.append([self.x, self.y, self.yaw, self.steer, self.gyroscope[-1], self.accelerometer[0], self.accelerometer[1], self.accelerometer[2]])
        for follower in self.followers:
            if self.speed>0.03:
                follower.add_waypoint([self.x, self.y, 90, self.steer, self.gyroscope[-1], self.accelerometer[0], self.accelerometer[1], self.accelerometer[2]])
                follower.set_speed_goal(self.speed)
            """if self.speed>0.03 and self.speed<=20:
                follower.add_waypoint([self.x, self.y, self.yaw, self.steer, self.gyroscope[-1], self.accelerometer[0], self.accelerometer[1], self.accelerometer[2]])
                follower.set_speed_goal(self.speed)
            if self.speed>20:
                follower.add_waypoint([self.x, self.y, self.yaw, self.steer, self.gyroscope[-1], self.accelerometer[0], self.accelerometer[1], self.accelerometer[2]])
                follower.set_speed_goal(0)"""
            """if self.speed>0.03:
                if self.x > 4 and self.x < 6:
                    follower.add_waypoint([self.x, self.y, 150, self.steer, self.gyroscope[-1], self.accelerometer[0], self.accelerometer[1], self.accelerometer[2]])
                    follower.set_speed_goal(self.speed)
                else:
                    follower.add_waypoint([self.x, self.y, self.yaw, self.steer, self.gyroscope[-1], self.accelerometer[0], self.accelerometer[1], self.accelerometer[2]])
                    follower.set_speed_goal(self.speed)"""
