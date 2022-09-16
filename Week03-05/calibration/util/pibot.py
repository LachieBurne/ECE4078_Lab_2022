# access each wheel and the camera onboard of PenguinPi

import numpy as np
import requests
import cv2 


class PenguinPi:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.wheel_vel = [0, 0]

    ##########################################
    # Change the robot velocity here
    # tick = forward speed
    # turning_tick = turning speed
    ########################################## 
    def set_velocity(self, command, tick=20, turning_tick=5, time=0): 
        l_vel = command[0]*tick - command[1]*turning_tick
        r_vel = command[0]*tick + command[1]*turning_tick
        self.wheel_vel = [l_vel, r_vel]
        if time == 0:
            requests.get(
                f"http://{self.ip}:{self.port}/robot/set/velocity?value="+str(l_vel)+","+str(r_vel))
        else:
            assert (time > 0), "Time must be positive."
            assert (time < 30), "Time must be less than network timeout (20s)."
            requests.get(
                "http://"+self.ip+":"+str(self.port)+"/robot/set/velocity?value="+str(l_vel)+","+str(r_vel)
                            +"&time="+str(time))
        return l_vel, r_vel
        
    def get_image(self):
        try:
            r = requests.get(f"http://{self.ip}:{self.port}/camera/get")
            img = cv2.imdecode(np.frombuffer(r.content,np.uint8), cv2.IMREAD_COLOR)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        except (requests.exceptions.ConnectTimeout, requests.exceptions.ConnectionError, requests.exceptions.ReadTimeout) as e:
            print("Image retrieval timed out.")
            img = np.zeros((240,320,3), dtype=np.uint8)
        return img
