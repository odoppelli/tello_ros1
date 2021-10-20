#!/usr/bin/env python
from enum import Enum
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import rospy
from sensor_msgs.msg import Image
import std_msgs.msg
from std_msgs.msg import Empty, UInt8
import geometry_msgs.msg
import imutils
import tf2_ros
import time
from datetime import datetime


#   NODE NAME   
rospy.init_node('FLYING-DUTCHMAN')

#   PUBLISHERS 
verarbeiter = rospy.Publisher('/tello/image_verarbeitet', Image, queue_size=1)
pub_takeoff = rospy.Publisher('/tello/takeoff', Empty,  queue_size=1, latch=False)
pub_land = rospy.Publisher('/tello/land', Empty,  queue_size=1, latch=False)
set_setpoint_x = rospy.Publisher('/x_dir/setpoint', std_msgs.msg.Float64, queue_size=1)
set_setpoint_y = rospy.Publisher('/y_dir/setpoint', std_msgs.msg.Float64, queue_size=1)
set_setpoint_z = rospy.Publisher('/z_dir/setpoint', std_msgs.msg.Float64, queue_size=1)
pid_x_stop = rospy.Publisher('/x_dir/pid_enable',std_msgs.msg.Bool, queue_size=1)
pid_y_stop = rospy.Publisher('/y_dir/pid_enable',std_msgs.msg.Bool, queue_size=1)
pid_z_stop = rospy.Publisher('/z_dir/pid_enable',std_msgs.msg.Bool, queue_size=1)



#   DEKLARATIONEN & HANDLER
# bildverarbeitung
bridge = CvBridge()
image = None
final_image = None
HSV_BOUNDS1 = [np.array([173,100,100]),np.array([180,255,255])]
HSV_BOUNDS2 = [np.array([0,100,100]),np.array([10,255,225])]
KNOWN_WIDTH = 1
FOCAL_LENGTH = 1

geschaetzte_messungen_counter = 0
image_points = None
MODEL_POINTS = np.array([
            (0.0,0.0,0.0),
            (0.05,0,0.68),
            (0.05,1.45,0.68),
            (0.0,1.45,0.0)
            ], dtype='double')
DIST_COEFFS = np.array([-0.019450, 0.060040, -0.001809, 0.004933, 0.000000])
CAMERA_MATRIX = np.array([ [922.98818,    0.     ,  506.64966],
                            [0.     ,  922.19482,  358.04117],
                            [0.     ,    0.     ,    1.     ]])

#   --- KLASSEN --- 
#   POSITION
class Position(object):
    def __init__(self):
        self.pos_msg = geometry_msgs.msg.Pose()
        self.pos_msg.position.x = None
        self.pos_msg.position.y = None
        self.pos_msg.position.z = None
        self.pos_msg.orientation.x = None
        self.pos_msg.orientation.y = None
        self.pos_msg.orientation.z = None
        self.pos_msg.orientation.w = None
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.static_transformStamped = geometry_msgs.msg.TransformStamped()
        
    def set_tf_position(self, parent_id, child_id):
        #only do after set_position()
        self.static_transformStamped.header.stamp = rospy.Time.now()
        self.static_transformStamped.frame_id = parent_id
        self.static_transformStamped.child_frame_id = child_id

        self.static_transformStamped.transform.translation.x = self.pos_msg.position.x
        self.static_transformStamped.transform.translation.y = self.pos_msg.position.y
        self.static_transformStamped.transform.translation.z = self.pos_msg.position.z
        self.static_transformStamped.transform.rotation.x = self.pos_msg.orientation.x
        self.static_transformStamped.transform.rotation.y = self.pos_msg.orientation.y
        self.static_transformStamped.transform.rotation.z = self.pos_msg.orientation.z
        self.static_transformStamped.transform.rotation.w = self.pos_msg.orientation.w

        self.broadcaster.sendTransform(self.static_transformStamped)
          
    def set_position(self, pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w):
        self.pos_msg.position.x = pos_x
        self.pos_msg.position.y = pos_y
        self.pos_msg.position.z = pos_z
        self.pos_msg.orientation.x = ori_x
        self.pos_msg.orientation.y = ori_y
        self.pos_msg.orientation.z = ori_z
        self.pos_msg.orientation.w = ori_w
    
    def print_position(self):
        print('x',self.pos_msg.position.x)
        print('y',self.pos_msg.position.y)
        print('z',self.pos_msg.position.z)
    

class Nullpunkt(Position):
    def __init__(self):
        super(Nullpunkt, self).__init__()
        self.zero_is_set = False
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.static_transformStamped = geometry_msgs.msg.TransformStamped()
        #self.zero_is_set = rospy.Publisher('/tello/zero_set', std_msgs.Bool, queue_size=1)
        while not self.zero_is_set:
            self.set_zero()

    def set_zero(self):
        try:
            transformObject = self.tfBuffer.lookup_transform('world', 'tello_base_link', rospy.Time())
            self.static_transformStamped.header.stamp = rospy.Time.now()
            self.static_transformStamped.header.frame_id = 'world'
            self.static_transformStamped.child_frame_id = 'my_zero'
            # POS Tello
            p_x = transformObject.transform.translation.x
            p_y = transformObject.transform.translation.y
            p_z = transformObject.transform.translation.z
            o_x = transformObject.transform.rotation.x
            o_y = transformObject.transform.rotation.y
            o_z = transformObject.transform.rotation.z
            o_w = transformObject.transform.rotation.w
            # Nullpunkt
            self.static_transformStamped.transform.translation.x = p_x
            self.static_transformStamped.transform.translation.y = p_y
            self.static_transformStamped.transform.translation.z = p_z
            self.static_transformStamped.transform.rotation.x = o_x
            self.static_transformStamped.transform.rotation.y = o_y
            self.static_transformStamped.transform.rotation.z = o_z
            self.static_transformStamped.transform.rotation.w = o_w
            self.broadcaster.sendTransform(self.static_transformStamped)
            self.set_position(p_x, p_y, p_z, o_x, o_y, o_z, o_w)
            self.zero_is_set = True
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            return False
        
        


#   MESSUNGEN + GEDAECHTNIS
class Messung:
    def __init__(self, box_x, box_y, height, width, image):
        self.box_x = box_x
        self.box_y = box_y
        self.height = height
        self.width = width
        self.area = self.width * self.height
        self.image = image


class Gedaechtnis:
    def __init__(self):
        self.kurzzeit = []
        self.langzeit = []
        self.kurzzeit_counter = 0
        self.langzeit_counter = 0
        self.chosen_messung = None

    def neue_Messung(self,msg):
        verarbeitet = None
        verarbeitet = verarbeiten(msg)
        if not(verarbeitet is None):
            if verarbeitet.area >= 5000:
                self.kurzzeit.append(verarbeitet)
                self.kurzzeit_counter += 1
                if self.kurzzeit_counter >= 10:
                    self.verwalten()

    def get_langzeitcounter(self):
        return self.langzeit_counter

    def verwalten(self):
        greatest_Area = None
        tmp_area = 0
        #print('kurzzeit:', self.kurzzeit)
        for m in self.kurzzeit:
            if m.area >= tmp_area:
                tmp_area = m.area
                greatest_Area = m
        self.langzeit.append(greatest_Area)
        self.langzeit_counter += 1
        self.kurzzeit_counter = 0
        #self.kurzzeit.clear()
        for item in self.kurzzeit:
            self.kurzzeit.remove(item)
    
    def auswaehlen(self):
        greatest_Area = None
        tmp_Area = 0
        for messung in self.langzeit:
            #print('langzeit messung', messung)
            if messung.area >= tmp_Area:
                greatest_Area = messung
                tmp_Area = messung.area
        self.chosen_messung = greatest_Area
        return self.chosen_messung


class Setpoint:
    def __init__(self, x,y,z):
        self.x = x
        self.y = y
        self.z = z


class Route:
    def __init__(self, nullpunkt_position):
        self.detection_setpoints = []
        self.start_pos = nullpunkt_position
        self.setpoints = []
        self.zero = Position()
        self.current_setpoint = None
        self.zero.set_position(0,0,0,0,0,0,0)
    
    def compute_detection_setpoints(self):
        for i in range(-1,2,2):
            new_pos = Position()
            new_y = i * 0.2
            new_pos.set_position(0,new_y,0,0,0,0,0)
            self.detection_setpoints.append(new_pos)

    def set_gatesetpoints(self, distanz_ergebnis):
        # 1 meter davor und dahinter & ein drittel der gesamthoehe und mitte tor
        tmp = distanz_ergebnis.get('translation_vector')
        t = []
        for value in tmp:
            t.append(round(value[0],3))
        print ('t', t)
        for i in range(-1,2,2):
            new_pos = Position()
            new_pos.set_position(t[2]+i,-1*(t[1]+0.75),-1*(t[2]+0.225),0,0,0,0)
            self.setpoints.append(new_pos)

    def get_new_setpoint(self):
        if len(self.setpoints) == 0:
            return 0
        self.current_setpoint = self.setpoints.pop(0)
        return self.current_setpoint

    def get_current_setpoint(self):
        return self.current_setpoint
    

def set_setpoint(position):
    global set_setpoint_x
    global set_setpoint_y
    global set_setpoint_z
    rospy.loginfo('fly to x:' + str(position.pos_msg.position.x))
    set_setpoint_x.publish(position.pos_msg.position.x)
    rospy.loginfo('fly to y:' + str(position.pos_msg.position.y))
    set_setpoint_y.publish(position.pos_msg.position.y)
    rospy.loginfo('fly to z:' + str(position.pos_msg.position.z))
    set_setpoint_z.publish(position.pos_msg.position.z) 

def distanz_schaetzen(messung, model_points, camera_matrix, dist_coeffs):
    chosen_messung = messung
    print('chosen_messung', chosen_messung)
    #image points rausfiltern
    global image_points
    global geschaetzte_messungen_counter
    point_1 = [chosen_messung.box_x, chosen_messung.box_y]
    point_2 = [chosen_messung.box_x, chosen_messung.box_y + chosen_messung.height]
    point_3 = [chosen_messung.box_x + chosen_messung.width, chosen_messung.box_y + chosen_messung.height]
    point_4 = [chosen_messung.box_x + chosen_messung.width, chosen_messung.box_y]
    image_points=np.array([
                    point_1,
                    point_2,
                    point_3,
                    point_4  
                ], dtype='double')
    ergebnis = {}
    (succ, rot_vect, trans_vect) = cv.solvePnP(
                model_points,
                image_points,
                camera_matrix,
                dist_coeffs)
    ergebnis['success'] = succ
    ergebnis['rotation_vector'] = rot_vect
    ergebnis['translation_vector'] = trans_vect

    now = datetime.now()
    y_m_d_h_m = str(now.year) + str(now.month) + str(now.day) + '-' + str(now.hour) + str(now.minute)
    name = y_m_d_h_m + str(geschaetzte_messungen_counter)+'.png'
    path = '/home/tello/tello_ws/src/tello_driver/pics/'+name
    text = ''
    pos = 100
    for value in trans_vect:
        text =str(round(value[0],3))
        cv.putText(messung.image, text,(50,messung.image.shape[0]-pos),
                cv.FONT_HERSHEY_SIMPLEX,1,(245, 242, 66),2)
        pos -= 35
    cv.imwrite(path, messung.image)

    #cv.imshow(str(geschaetzte_messungen_counter),messung.image)
    geschaetzte_messungen_counter += 1
    return ergebnis


#   BILDVERARBEITUNG
def verarbeiten(msg):
    global image
    global HSV_BOUNDS1
    global HSV_BOUNDS2
    global pos_msg
    # zu CV Bild konvertieren
    image = bridge.imgmsg_to_cv2(msg, "bgr8")
    # smoothing image
    gauss = cv.GaussianBlur(image, (7,7), 0)
    # hsv color filter [H: 0-179 , S: 0-255, V: 0-255]
    hsv = cv.cvtColor(gauss, cv.COLOR_BGR2HSV)
    mask1, mask2 = cv.inRange(hsv,HSV_BOUNDS1[0], HSV_BOUNDS1[1]), cv.inRange(hsv,HSV_BOUNDS2[0], HSV_BOUNDS2[1])
    hsv_mask = mask1 + mask2
    hsv_filtered = cv.bitwise_and(gauss, gauss, mask=hsv_mask)
    # contour detection
    canny = cv.Canny(hsv_filtered, 50, 150)
    # bounding rectangle
    cnts = cv.findContours(canny, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    max_cnt = max(cnts, key = cv.contourArea)
    # bounding a box (normal)
    x,y,w,h = cv.boundingRect(max_cnt)
    drawing = image.copy()
    cv.rectangle(drawing,(x,y),(x+w, y + h),(245, 242, 66),2)
    rect_area = w * h
    #publish image
    #drawing = bridge.cv2_to_imgmsg(drawing)
    #verarbeiter.publish(drawing)

    return Messung(x, y, h, w, drawing)




#   MAIN
rate = rospy.Rate(30)
gedaechtnis = None
ship_position = None
pid_helper = None
route = None
zustand = 0
gate_ergebnis = None
if __name__ == "__main__":
    while not rospy.is_shutdown():
        if zustand == 0:
            # start sequence
            rospy.loginfo('Zustand: ' + str(zustand))
            
            gedaechtnis = Gedaechtnis()
            for _ in range(0,5):
                pub_takeoff.publish()
            rospy.loginfo('took off')
            time.sleep(8)
            # pid debugging
            #pid_x_stop.publish(False)
            #pid_y_stop.publish(False)
            #pid_z_stop.publish(False)
            nullpunkt = Nullpunkt()
            rospy.loginfo('Nullpunkt gesetzt')
            route = Route(nullpunkt)
            route.compute_detection_setpoints()
            rospy.loginfo('Kalibierungsroute gesetzt')
            set_setpoint(route.zero)
            time.sleep(5)
            zustand += 1
            
        if zustand == 1:
            # detection sequence
            rospy.loginfo('Zustand: ' + str(zustand))
            # naechste position abfliegen
            
            # DICKE BAUSTELLE
            
            to_position = route.detection_setpoints.pop(0)
            
            #flyto to_position
            #set_setpoint(to_position)
            #time.sleep(3)
            
            #taking some beautiful pictures
            
            image_sub = None
            while not (gedaechtnis.get_langzeitcounter() >= 5):
                image_sub =rospy.Subscriber('/tello/image_raw', Image, gedaechtnis.neue_Messung)
            
            ''''''
            if gedaechtnis.get_langzeitcounter() >=5:
                image_sub.unregister()
                zustand += 1
                set_setpoint(route.zero)
                #zuruek zum nullpunkt fliegen
            
            if len(route.detection_setpoints) == 0:
                #image_sub.unregister()
                zustand += 1
                set_setpoint(route.zero)
                time.sleep(3)
                #zuruek zum nullpunkt fliegen

        if zustand == 2:
            # planning sequence
            rospy.loginfo('Zustand: ' + str(zustand))
            
            gate_ergebnis = distanz_schaetzen(gedaechtnis.auswaehlen(),MODEL_POINTS,CAMERA_MATRIX,DIST_COEFFS)
            print(gate_ergebnis.get('translation_vector'))
                #if cv.waitKey(0):
                   # break
            
            route.set_gatesetpoints(gate_ergebnis)
            
            zustand += 1
        if zustand == 3:
            # flying sequence
            rospy.loginfo('Zustand: ' + str(zustand))
            
            to_position = route.setpoints.pop(0)
            set_setpoint(to_position)
            time.sleep(10)
            
            if len(route.setpoints) == 0:
                zustand += 1
            
            
        if zustand == 4:
            # idle sequence and land
            print('zustand', zustand)
            pub_land.publish()
            zustand +=1
        if zustand == 5:
            # stopping and landing sequence
            rospy.loginfo('Zustand: ' + str(zustand))
            # wie terminiere ich das programm lol
            
        rate.sleep()

