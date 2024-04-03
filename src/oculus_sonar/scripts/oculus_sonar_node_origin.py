#!/usr/bin/env python
import socket
import struct
# import rospy
import numpy as np
import rospy
# from sensor_msgs.msg import Image
# from std_msgs.msg import Float64MultiArray
# from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server
from oculus_sonar.cfg import OculusConfig
import cv2

from std_msgs.msg import Header
from sensor_msgs.msg import Image

import threading
import subprocess
import cv2
import time

class OculusSonar:
    """
    Class to handle Sonar shit.
    """
    # Constant ports for Oculus comms
    TCP_PORT = 52100

    def __init__(self):
        """
        Not done yet.
        """
        self.tcp_sock = socket.socket(socket.AF_INET,
                                      socket.SOCK_STREAM)
        self.tcp_ip = rospy.get_param("~tcp_ip","192.168.115.42")
        #self.tcp_ip = '192.168.1.112'

        # Dynamic reconfigure server
        #self.config = None
        self.config = Server(OculusConfig, self.dynamic_cb)
        self.config_change = False

        self.fire_message = self.build_simplefire_msg(self.config)

        # Sonar image publisher
        self.count = 0
        self.image_pub = rospy.Publisher('/sonar_image_raw', Image, queue_size=1)
        self.azimuth = 0
        self.ranges = 0

        # self.center_ip = '192.168.1.110'
        # self.center_port = 8082
        # self.sonar_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.sonar_client.connect((self.center_ip, self.center_port))

        # self.capture_img = False


    def connect_tcp(self):
        """
        If no message is sent to the sonar in more than
        one second, it'll drop the tcp connection.
        """
        self.tcp_sock.connect((self.tcp_ip, self.TCP_PORT))

    def send_tcp_msg(self, msg):
        """
        Send tcp message to the connected port.
        """
        self.tcp_sock.sendto(msg, (self.tcp_ip, self.TCP_PORT))

    def recv_tcp_msg(self):
        """
        Duh.
        """
        data = self.tcp_sock.recv(1500)
        return data

    def build_simplefire_msg(self, config,
                             masterMode=1, pingRate=0,
                             gamma=150, gain=0, range_m=10,
                             vos=0, salinity=0):
        """
        Fetch the first part from the UDP config message and fill
        in the rest.
        """
        # If dynamic reconfig not running
        # SimpleFire Message
        oculusId = b'\x53\x4f'
        srcDeviceId = b'\x00\x00'
        dstDeviceId = b'\x00\x00'
        msgId = b'\x15\x00'
        msgVersion = b'\x00\x00'
        payloadSize = b'\x00\x00\x00\x00'
        spare2 = b'\x00\x00'
        fire_message = oculusId + srcDeviceId + dstDeviceId + msgId + msgVersion + payloadSize + spare2

        if not self.config_change:
            # Constant header for the SimpleFireRequest
            fire_message += struct.pack('B', masterMode)
            fire_message += struct.pack('B', pingRate)
            networkSpeed = b'\xff'
            fire_message += networkSpeed
            fire_message += struct.pack('B', gamma)
            flags = b'\x19'
            fire_message += flags
            fire_message += struct.pack('d', range_m)
            fire_message += struct.pack('d', gain)
            fire_message += struct.pack('d', vos)
            fire_message += struct.pack('d', salinity)
        else:
        # Constant header for SimpleFireRequest
            fire_message += struct.pack('B', config['masterMode'])
            fire_message += struct.pack('B', config['pingRate'])
            networkSpeed = b'\xff'
            fire_message += networkSpeed
            fire_message += struct.pack('B', config['gamma'])
            flags = b'\x19'
            fire_message += flags
            fire_message += struct.pack('d', config['range_m'])
            fire_message += struct.pack('d', config['gain'])
            fire_message += struct.pack('d', config['vOfSound'])
            fire_message += struct.pack('d', config['salinity'])

        self.config_change = False

        return fire_message

    def dynamic_cb(self, config, level):
        """
        Callback for the dynamic reconfigure server.
        """
        self.config = config
        self.config_change = True
        return config


    def process_n_publish(self, data):
        """
        Process the recieved sonar message and
        publish the sonar image on a ROS Image
        message.
        """
        self.count += 1
        # Get no.bearings and no.ranges
        dim = struct.unpack('HH', data[106:110])
        # Starting offset for sonar image
        img_offset = struct.unpack('I', data[110:114])[0]
        # Build greyscale image from echo intensity data
        img = np.frombuffer(data[img_offset:], dtype='uint8')
        img_temp_buffer = img.copy()
        for i in range(dim[0]):
            img_temp_buffer[i*dim[1] : (i+1)*dim[1]] = img[(dim[0] - i - 1)*dim[1] : (dim[0] - i)*dim[1]]
        img_size = struct.unpack('I', data[114:118])[0]
        msg_len = struct.unpack('I',data[10:14])[0]
        
        if dim[0]*dim[1] != img_size:
            rospy.logwarn("Message dims {0} don't match ping result info {1}. Dropping frame.".format(dim, len(img)))
        else:            
            #print(dim)
            master_mode = data[16]
            if master_mode == 1:
                self.azimuth = 130
            else:
                self.azimuth = 80
            self.ranges = struct.unpack('d', data[21:29])[0]

            # ros publish sonar image
            image_temp = Image()
            header = Header(stamp=rospy.Time.now())
            header.frame_id = "["+ str(float(self.azimuth)) + " " + str(float(self.ranges)) + "]"
            image_temp.height = dim[0]
            image_temp.width = dim[1]
            image_temp.encoding = 'mono8'
            image_temp.data = np.array(img_temp_buffer).tobytes()
            image_temp.header = header
            image_temp.step = dim[1]
            self.image_pub.publish(image_temp)

            
            # # capture current image and sent in tcp
            # if self.capture_img:
            #     cap_img_buffer = b'\xff\xcc\xff\xcc'
            #     cap_img_buffer += struct.pack('<f', float(self.ranges))
            #     cap_img_buffer += struct.pack('<f', float(self.azimuth))
            #     cap_img_buffer += struct.pack('<i', dim[0])
            #     cap_img_buffer += struct.pack('<i', dim[1])

            #     try:
            #         cap_img = img_temp_buffer.reshape(dim[0], dim[1])
            #         encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 10]
            #         result, encode_img = cv2.imencode('.jpg', cap_img, encode_param)
            #         cap_img_len = len(encode_img)
            #         cap_img_buffer += struct.pack('<i', cap_img_len)
            #         cap_img_buffer += bytearray(encode_img)
            #         cap_img_buffer += b'\xff\xc0\xff\xc0'
            #         self.sonar_client.sendall(cap_img_buffer)
            #         self.capture_img = False

            #     except Exception as e:
            #         print("capture sonar image error: ",e)
            
                      
    # # receive tcp cmd msg from center server, set the sonar parameter
    # def recv_tcp_cmd(self):
    #     #self.sonar_client.connect((self.center_ip, self.center_port))
    #     while True:
    #         sonar_msg = self.sonar_client.recv(1024)
    #         # set the low/high frequency channel, 0 for low frequency channel(750KHz), 1 for high frequency channel(1200kHz)
    #         if sonar_msg[0:4] == b'\xee\xbb\xee\x01':
    #             master_mode = struct.unpack('i', sonar_msg[4:8])[0]
    #             master_mode_cmd = 'rosrun dynamic_reconfigure dynparam set sonar_imager masterMode ' + str(master_mode)
    #             sonar_subprocess = subprocess.Popen(master_mode_cmd, shell=True, executable="/bin/bash")

    #         # set the gamma correction, 1~200
    #         elif sonar_msg[0:4] == b'\xee\xbb\xee\x02':
    #             gamma = struct.unpack('i', sonar_msg[4:8])[0]
    #             gamma_cmd = 'rosrun dynamic_reconfigure dynparam set sonar_imager gamma ' + str(gamma)
    #             sonar_subprocess = subprocess.Popen(gamma_cmd, shell=True, executable="/bin/bash")

    #         # set the sonar_range, 1~120
    #         elif sonar_msg[0:4] == b'\xee\xbb\xee\x03':
    #             sonar_range = struct.unpack('i', sonar_msg[4:8])[0]
    #             sonar_range_cmd = 'rosrun dynamic_reconfigure dynparam set sonar_imager range_m ' + str(sonar_range)
    #             sonar_subprocess = subprocess.Popen(sonar_range_cmd, shell=True, executable="/bin/bash")

    #         # set the gain, 0~100
    #         elif sonar_msg[0:4] == b'\xee\xbb\xee\x04':
    #             sonar_gain = struct.unpack('f', sonar_msg[4:8])[0]
    #             sonar_gain_cmd = 'rosrun dynamic_reconfigure dynparam set sonar_imager gain ' + str(sonar_gain)
    #             sonar_subprocess = subprocess.Popen(sonar_gain_cmd, shell=True, executable="/bin/bash")

    #         # set the velocity of sound speed, 0~2000, 0 is suggestied if unclear
    #         elif sonar_msg[0:4] == b'\xee\xbb\xee\x05':
    #             sonar_v_sound = struct.unpack('f', sonar_msg[4:8])[0]
    #             sonar_v_sound_cmd = 'rosrun dynamic_reconfigure dynparam set sonar_imager vOfSound ' + str(sonar_v_sound)
    #             sonar_subprocess = subprocess.Popen(sonar_v_sound_cmd, shell=True, executable="/bin/bash")

    #         # set the salinity in ppm, 0~50, 0 is suggestied if unclear
    #         elif sonar_msg[0:4] == b'\xee\xbb\xee\x06':
    #             sonar_salinity = struct.unpack('f', sonar_msg[4:8])[0]
    #             sonar_salinity_cmd = 'rosrun dynamic_reconfigure dynparam set sonar_imager salinity ' + str(sonar_salinity)
    #             sonar_subprocess = subprocess.Popen(sonar_salinity_cmd, shell=True, executable="/bin/bash")

    #         # capture the current img
    #         elif sonar_msg[0:4] == b'\xee\xbb\xee\xcc':
    #             self.capture_img = True
                        
    #         # set the ping rate mode: 0(10Hz), 1(15Hz), 2(40Hz), 3(5Hz), 4(2Hz), 5(Disable)
    #         #elif sonar_msg[0:4] == b'\xee\xbb\xee\x07':
    #         #    sonar_ping_rate = struct.unpack('i', sonar_msg[4:8])[0]
    #         #    sonar_ping_rate_cmd = 'rosrun dynamic_reconfigure dynparam set sonar_imager pingRate ' + str(sonar_ping_rate)
    #         #    sonar_subprocess = subprocess.Popen(sonar_ping_rate_cmd, shell=True, executable="/bin/bash")

    #         else:
    #             print("sonar parameters set cmd error...")

    # def keep_tcp_alive(self):
    #     try:
    #         alive_msg = b'\xef\xef\xfe\xfe'
    #         self.sonar_client.send(alive_msg)
    #         time.sleep(60)
    #     except Exception as e:
    #         print("sonar alive error: ",e)
        

def main():
    """
    Main method for the ROS node.
    """
    rospy.init_node("oculus_sonar", anonymous = False)
    sonar = OculusSonar()
    fire_message = sonar.build_simplefire_msg(sonar.config)
    sonar.connect_tcp()

    # t1 = threading.Thread(target=sonar.recv_tcp_cmd, daemon=True)
    # t2 = threading.Thread(target=sonar.keep_tcp_alive, daemon=True)
    # t1.start()
    # t2.start()

    while not rospy.is_shutdown():
        if sonar.config_change:
            fire_message = sonar.build_simplefire_msg(sonar.config)
        
        sonar.send_tcp_msg(fire_message)
        data = sonar.recv_tcp_msg()

        if data == None:
            continue
        if len(data) < 100:
            continue

        msg_len = struct.unpack('I',data[10:14])[0]
        while len(data) < msg_len:
            data += sonar.recv_tcp_msg()

        sonar.process_n_publish(data)
        data = ''

if __name__ == "__main__":
    main()

