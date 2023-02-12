
class SafeCloud:

    def __init__(self):
        self.members =[]
        self.leader = None
        self.check_args=[]

    def set_leader(self, l):
        if not self.leader:
            self.leader=l
            print("Leader set.")

    def add_members(self, m):
        if len(self.members)>0:
            l = self.members[-1]
        elif self.leader!=None:
            l = self.leader
        else:
            print("No leader available")
            return
        m.set_leading_vehicle(l)
        self.members.append(m)
        print("Connected followers:",len(self.members))
        self.leader.addFollower(m)

    def retrieve_check_data(self,vehicle):
        for elem in self.members:
            if vehicle==elem:
                wp = elem.waypoints
                x = elem.vehicle.get_transform().location.x
                y = elem.vehicle.get_transform().location.y
                ya = elem.vehicle.get_transform().rotation.yaw
                sg = self.leader.speed
                ss = elem.speed
                self.check_args=[wp,x,y,ya,sg,ss]

    def check_action(self,member,t,s,b):
        cloud_t = cloud_s = cloud_b = 0.0
        for elem in self.members:
            if member==elem:
                wp = self.check_args[0]
                x = self.check_args[1]
                y = self.check_args[2]
                ya = self.check_args[3]
                cloud_s = elem.define_steer(wp, x, y, ya)
                sg = self.check_args[4]
                ss = self.check_args[5]
                cloud_t, cloud_b = elem.define_throttle(sg, ss, wp)
                break
        cond = abs(cloud_b-b)<0.1 and abs(cloud_s-s)<0.1 and abs(cloud_t-t)<0.1
        return cond, cloud_t, cloud_s, cloud_b
