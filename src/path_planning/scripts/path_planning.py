#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
import numpy as np
import sys

#pub = rospy.Publisher('test', Float64, queue_size=10)

###### HELPER FUNCTIONS ######

##############################

###### SYSTEM CLASSES ######

class Integrator:
    def __init__(self):
        self.state = 0
        self.time = 0

    def __call__(self, input, new_time):
        self.state = input *(new_time - self.time) + self.state
        self.time = new_time

        return self.state

class Differentiator:
    def __init__(self):
        self.state = 0
        self.time = 0

    def __call__(self, input, new_time):
        ret = (input - self.state) / (new_time - self.time)
        self.time = new_time
        self.state = input
        return ret

class Smoother:
    def __init__(self, order):
        self.pos = 0
        self.order = order
        self.values = [0] * order

    def __call__(self, input):
        self.values[self.pos % self.order] = input
        self.pos += 1
        return np.mean(self.values)

class MinFilter:
    def __init__(self, order):
        self.pos = 0
        self.order = order
        self.values = [0] * order

    def __call__(self, input):
        self.values[self.pos % self.order] = input
        self.pos += 1
        return np.min(self.values)

############################

class Robot:

    def __init__(self,targetX, targetY):
        self.target = np.array([targetX, targetY])
        self.N_target = (len(sys.argv)-1)/2
        self.target_counter = 1
        self.state = {'pos':np.zeros(2)+ 0.01, 'theta':0.0}
        self.timestamp = 0
        self.updated_sensors = {
            'right': False,
            'fright': False,
            'front': False,
            'fleft': False,
            'left': False,
            'imu': False
        }
        self.sonar = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0
        }

        self.sonar_filter = {
            'right': MinFilter(3),
            'fright': MinFilter(3),
            'front': MinFilter(3),
            'fleft': MinFilter(3),
            'left': MinFilter(3)
        }

        self.imu = {
            'accX':0,
            'accY':0,
            'accZ':0,
            'oriX':0,
            'oriY':0,
            'oriZ':0
        }

        self.Krep = 2.0
        self.r0 = 0.5
        self.Kp = -1

    def obst_avoid(self):
        msg = Twist()
        theta = self.state['theta']
        p = self.state['pos']
        t = self.target - p
        ori = np.array([np.cos(theta),np.sin(theta)])
        e_theta = np.arccos(np.dot(ori,t)/(np.linalg.norm(ori)*np.linalg.norm(t)))
        #print np.linalg.norm(t)
        if np.linalg.norm(t) < 0.1: #target reached
            self.N_target -= 1
            self.target_counter += 2
            if self.N_target > 0:
                self.target = np.array([float(sys.argv[self.target_counter]), float(sys.argv[self.target_counter+1])])
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            #sys.exit()
        else:
            msg.linear.x = 0.3
            ang = 0;
            ob_front = self.sonar['front'] < self.r0
            ob_fleft = self.sonar['fleft'] < self.r0
            ob_fright = self.sonar['fright'] < self.r0
            ob_left = self.sonar['left'] < self.r0
            ob_right = self.sonar['right'] < self.r0

            if ob_fleft:
                ang = ang - self.Krep*(1/self.sonar['fleft'] - 1/self.r0)
                #print 'fleft: ' + str(ang)
            if ob_fright:
                ang = ang + self.Krep*(1/self.sonar['fright'] - 1/self.r0)
                #print 'fright: ' + str(self.Krep*(1/self.sonar['fright'] - 1/self.r0))
            if ob_front:
                sign = -1 if (ob_left or ob_fleft) else 1
                ang = ang + sign*self.Krep*(1/self.sonar['front'] - 1/self.r0)
            if ob_right:
                ang = ang + 0.2
            if ob_left:
                ang = ang - 0.2



            ang = ang + self.Kp*np.sign(np.cross(t,ori))*e_theta
            print e_theta*np.sign(np.cross(t,ori)),' ',ang

            if ang > 5:
                ang = 5
            elif ang<-5:
                ang = -5
            msg.angular.z = ang
        return msg

    def localizer(self, msg):
        Dt = 0.1
        theta = np.arctan2(self.imu['oriY'], self.imu['oriX'])
        self.state['theta'] =theta
        Dtheta = msg.angular.z * Dt
        fi = Dtheta/2
        if np.abs(msg.angular.z) > 0.01:
            Ds = 2*np.sin(msg.angular.z*Dt/2)*msg.linear.x/msg.angular.z
        else:
            Ds = Dt * msg.linear.x
        Dp = np.array([Ds*np.sin(theta - fi), Ds*np.cos(theta - fi)])

        #overwrite evrything
        Ds = Dt * msg.linear.x
        Dp = np.array([Ds*np.cos(theta), Ds*np.sin(theta)])

        self.state['pos'] += Dp
        p = Pose()
        p.position.x, p.position.y = self.state['pos'][0], self.state['pos'][1]
        p.position.z = theta
        p.orientation.x = self.target[0]
        p.orientation.y = self.target[1]
        pose_pub.publish(p)

    def calculate_action(self):
        #L.publish([('log_error',e), ('log_Pout',Pout), ('log_Dout',Dout), ('log_out', out), ('log_theta', theta), ('log_mu', mu)])
        msg = self.obst_avoid()
        self.localizer(msg) #change state
        return msg


    def update_values(self,which,value):
        #rospy.loginfo("Updating %s with: %.2f", which, value)
        self.updated_sensors[which] = True
        if which == 'imu':
            self.imu['accX'] = value.linear_acceleration.x
            self.imu['accY'] = value.linear_acceleration.y
            self.imu['accZ'] = value.linear_acceleration.z
            self.imu['oriX'] = value.orientation.x
            self.imu['oriY'] = value.orientation.y
            self.imu['oriZ'] = value.orientation.z
        else:
            self.sonar[which] = value
        if(sum(self.updated_sensors.values()) == 6):
            self.timestamp = rospy.get_time()
            #rospy.loginfo("All sensors read")
            for k in self.updated_sensors.keys():
                self.updated_sensors[k] = False
            msg = self.calculate_action()
            velocity_pub.publish(msg)

class Logger:
    def __init__(self,name_list):
        self.pub = dict()
        for n in name_list:
            self.pub[n] = rospy.Publisher(n, Float64, queue_size=10)

    def publish(self, plot_list):
        for t in plot_list:
            self.pub[t[0]].publish(t[1])

###### CALLBACK FUNCTIONS ######

def sonarFrontCallback(msg):
    R.update_values('front', msg.range)

def sonarFrontLeftCallback(msg):
    R.update_values('fleft', msg.range)

def sonarFrontRightCallback(msg):
    R.update_values('fright', msg.range)

def sonarLeftCallback(msg):
    R.update_values('left', msg.range)

def sonarRightCallback(msg):
    R.update_values('right', msg.range)


def imuCallback(msg):
    R.update_values('imu',msg)


################################

def path_planner():
    # Starts a new node
    rospy.init_node('path_planning_node', anonymous=True, log_level=rospy.DEBUG)
    rospy.Subscriber("sonar_front", Range, sonarFrontCallback)
    rospy.Subscriber("sonar_front_left", Range, sonarFrontLeftCallback)
    rospy.Subscriber("sonar_front_right", Range, sonarFrontRightCallback)
    rospy.Subscriber("sonar_left", Range, sonarLeftCallback)
    rospy.Subscriber("imu_data", Imu, imuCallback)
    rospy.Subscriber("sonar_right", Range, sonarRightCallback)
    global velocity_pub
    velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    global pose_pub
    pose_pub = rospy.Publisher("/pose", Pose, queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print 'give target X Y'
        sys.exit()
    R = Robot(targetX=float(sys.argv[1]), targetY=float(sys.argv[2]))
    #L = Logger(['log_posx','log_posy','])
    try:
        path_planner()
    except rospy.ROSInterruptException: pass
