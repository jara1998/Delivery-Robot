import serial
import time
import string
import pynmea2
import math
#import urllib.request
from six.moves import urllib
import json


class navigation_sys:
    def __init__(self, finished, step_num, desti, DIST_THRESHHOLD=0.00009):
        self.finished = finished
        self.step_num = step_num
        self.DIST_THRESHHOLD = DIST_THRESHHOLD # about 5.5 meters
        self.latitude = -1
        self.longitude = -1
        self.gps_fetch(First=True)
        self.steps = []
        self.desti = desti
    
    def direction_request(self, origin, destination):
        endpoint = "https://maps.googleapis.com/maps/api/directions/json?"
        api_key = "AIzaSyCEl81jShwlw-93u2ujLqk0qnSv09osY7E"
        #origin = origin.replace(' ', '+')
        destination = destination.replace(' ', '+')
        nav_request = 'origin={}&destination={}&mode=walking&key={}'.format(origin, destination, api_key)
        request = endpoint + nav_request
        #print(request)
        response = urllib.request.urlopen(request).read()
        direction = json.loads(response)
        routes = direction['routes']
        legs = routes[0]['legs']
        steps_final = legs[0]['steps']
        for i in range(len(steps_final)):
            print("step ", i, ": ", steps_final[i])
        
        return steps_final

    def gps_fetch(self, First=False):
        port="/dev/serial0"
        ser=serial.Serial(port, baudrate=9600, timeout=0.5)
        dataout = pynmea2.NMEAStreamReader()
        newdata=ser.readline().decode("utf-8")
        while newdata[0:6] != "$GPGLL":
            newdata=ser.readline().decode("utf-8")
        newmsg=pynmea2.parse(newdata)
        #print("newdata = ", newdata)
        if (newmsg.latitude == 0.0 or newmsg.longitude == 0.0):
            print(newmsg.latitude, newmsg.longitude)
            print("trying to connect to the satellite, please wait")
            # wait until get gps signal
            if (First):
                self.gps_fetch(First=True)
            # pass, no update
        else:
            # update lati & longi
            self.latitude = newmsg.latitude
            self.longitude = newmsg.longitude
        
        #return self.latitude, self.longitude
        #gps = "Latitude=" + str(lat) + "and Longitude=" + str(lng)
        #print(gps)
    
    def update_step(self):
        #self.latitude, self.longitude = self.gps_fetch()
        ## distance to next step
        
        self.gps_fetch()
        if self.finished:
            destination = self.desti
            # latitude,longitude = gps_fetch()
            origin = str(self.latitude) + "," + str(self.longitude)
            self.steps = self.direction_request(origin, destination)
            self.finished = False
            self.step_num = 0
            return 0 # start mode
            
        elif abs(self.steps[self.step_num]['end_location']['lat'] - self.latitude)**2 + abs(self.steps[self.step_num]['end_location']['lng'] - self.longitude)**2 < self.DIST_THRESHHOLD**2:
            if (self.step_num == len(self.steps) - 1):
                self.finished = True
                print("Arrived!")
                input("Press any button to continue")
                return 4 # hold/arrive mode
            else:
                print(self.steps[self.step_num + 1]['maneuver'])                                     
                self.step_num = self.step_num + 1
                if (self.steps[self.step_num]['maneuver'] == 'turn-left'):
                    return 2 # 2 for turning left, 3 for turning right
                elif (self.steps[self.step_num]['maneuver'] == 'turn-right'):
                    return 3
                else:
                    print("returns 99 from Direction API, unknown waypoint maneuver ")
                    return 99
        else:
            #print("keep going")
            print("distance to next step: ", math.sqrt(abs(self.steps[self.step_num]['end_location']['lat'] - self.latitude)**2 + abs(self.steps[self.step_num]['end_location']['lng'] - self.longitude)**2))
            print("threshold = ", self.DIST_THRESHHOLD)
        
            return 1 # go straight
        #time.sleep(1)


if __name__ == "__main__":
    nav = navigation_sys(True, 0, "uw comotion")
    while True:
        nav.update_step()
        time.sleep(1)
        #print("distance to next step: ", abs(nav.steps[nav.step_num]['end_location']['lat'] - nav.latitude)**2 + abs(nav.steps[nav.step_num]['end_location']['lng'] - nav.longitude)**2)
        #print("threshold: ", nav.DIST_THRESHHOLD**2)
