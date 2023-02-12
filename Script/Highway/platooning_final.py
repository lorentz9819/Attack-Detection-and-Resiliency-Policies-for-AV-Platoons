import carla
import math
import numpy as np
import random
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
        #x = random.randint(-100,100)
        #y = random.randint(-100,100)
        tx,ty = self.leader.get_pos()
        self.leader_dist = ((x-tx)**2 + (y-ty)**2)**(1/2)#+10 aggiunto per foolare il check sulla distanza e far accelerare follower
        self.safe_dist = max(12.0, self.speed/2)
        #self.safe_dist = 12.0
        if self.leader_dist > self.safe_dist: self.big_dist = True
        else: self.big_dist = False

    def connect_to_cloud(self, sc: SafeCloud):
        if not self.cloud:
            self.cloud = sc
        sc.add_members(self)

    def check_lidar(self,points):
        detection=False
        blinded=False
        #danger_dist = self.speed/5
        danger_dist = self.speed/10
        for p in points:
            if p.point.x<max(3,danger_dist):
                if p.point.x < 0.5:
                    blinded = True
                    detection = False
                else:
                    detection=True
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
        if self.speed > 15:
            if self.speedGoal >= 90:
                    print("Abnormal behaviour")
                    self.speedGoal = self.speed
                    sg = ss

        #--Rule 2-Abnormal stop--
        if self.speed > 15:
            if self.speedGoal <= 10:
                if not self.override_brake:
                    #Cio' significa che se il lidar e' blindato o se non rileva ostacoli, la regola non permette al veicolo di arrestarsi
                    print("Fake signal, the LiDAR is blinded or the data is corrupted, override")
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

            #--Rule 3-Avoid sudden swerve (Leader coordinates are faked to make the car turn)--
            if  (yaw < 95 and yaw > 85) and (lyaw<80 or lyaw>100) and (gyro < 5 and gyro > -5):
                print("Fake swerve")
                lyaw=90
            elif (yaw < 5 and yaw > -5) and (lyaw<-10 or lyaw>10) and (gyro < 5 and gyro > -5):
                print("Fake swerve")
                lyaw=0
            elif (yaw < -85 and yaw > -95) and (lyaw<-100 or lyaw>-80) and (gyro < 5 and gyro > -5):
                print("Fake swerve")
                lyaw=-90
            elif (yaw < -175 and yaw > 175) and (lyaw<170 or lyaw>-170) and (gyro < 5 and gyro > -5):
                print("Fake swerve")
                lyaw=180

            toll = (0.1**2+0.1**2)**(1/2)
            diff = ((fx-lx)**2 + (fy-ly)**2)**(1/2)
            #print ("Toll = ", toll)
            #print ("Diff = ", diff)
            #print ("Leader x", lx)
            #print ("Leader y", ly)
            #print ("Follower x", fx)
            #print ("Follower y", fy)
            #print ("Leader Yaw", lyaw)
            #print ("Follower Yaw", yaw)
            #print ("----------------")
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

        if abs(lyaw)<=5:
            s=(rel_y-fy)/10
            if s>1.0: s=1.0
            elif s<-1.0: s=-1.0
            if corr>1.0: corr = 1.0
            elif corr<-1.0: corr = -1.0
            s+=corr
        elif abs(lyaw)>=175:
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
        elif lyaw<95 and lyaw>85:
            s=(fx-rel_x)/10
            if s>1.0: s=1.0
            elif s<-1.0: s=-1.0
            if corr>1.0: corr = 1.0
            elif corr<-1.0: corr = -1.0
            s+=corr
        elif lyaw>-95 and lyaw<-85:
            print ("----------------")
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
        if self.override_brake: control = carla.VehicleControl(throttle=0, brake=1.0)
        else:
            wp = self.waypoints
            x = self.x
            y = self.y
            #y = self.y + 10 #+10 per falsare y
            #ya = random.randint(-180,180)
            ya = self.yaw
            #print ("Valore x = ", x)
            #print ("Valore y = ", y)
            #print ("Valore yaw = ", ya)
            s = self.define_steer(wp, x, y, ya)
            sg = self.speedGoal
            ss = self.speed
            t, b = self.define_throttle(sg, ss)
            control = carla.VehicleControl(throttle=t, steer=s, brake=b)
            cond, ct, cs, cb = self.cloud.check_action(self, t,s,b)
            #if not cond: #messo a commento per togliere self check su valori (vedi classe cloud)
                #print("ANOMALY DETECTED",t,ct,s,cs,b,cb)
                #control = carla.VehicleControl(throttle=ct, steer=cs, brake=cb)
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

    def move(self):
        self.update_position()
        self.update_follower()

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

    def update_follower(self):
        self.waypoints.append([self.x, self.y, self.yaw, self.steer, self.gyroscope[-1]])
        for follower in self.followers:
            #if self.speed>0.03:
            #    follower.add_waypoint([self.x, self.y, self.yaw, self.steer, self.gyroscope[-1]])
            #    follower.set_speed_goal(self.speed)
            """if self.speed>0.03 and self.speed<30.0:
                #print("NORMAL")
                follower.add_waypoint([self.x, self.y, self.yaw, self.steer, self.gyroscope[-1]])
                follower.set_speed_goal(self.speed)
            elif self.speed>=30.0 and self.speed<50.0:
                #print("ATTACK steer")
                follower.add_waypoint([self.x, self.y, 70, self.steer, self.gyroscope[-1]])
                follower.set_speed_goal(self.speed)
            elif self.speed>=50.0 and self.speed<70:
                #print("ATTACK speed")
                follower.add_waypoint([self.x, self.y, self.yaw, self.steer, self.gyroscope[-1]])
                follower.set_speed_goal(1)
            elif self.speed>=70.0:
                #print("NORMAL")
                follower.add_waypoint([self.x, self.y, self.yaw, self.steer, self.gyroscope[-1]])
                follower.set_speed_goal(self.speed)"""
            if self.speed>0.03 and self.speed<30.0:
                follower.add_waypoint([self.x, self.y, self.yaw, self.steer, self.gyroscope[-1]])
                follower.set_speed_goal(self.speed)
            if self.speed>=30.0:
                follower.add_waypoint([self.x, self.y, 70, self.steer, self.gyroscope[-1]])
                follower.set_speed_goal(self.speed)
