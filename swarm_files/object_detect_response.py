####################################################################
#
# Summary: 
#    Basic object-detection-response behavior for swarm
#
# Author:  
#    Ross Arnold, USMA, 25 Jan 2019
#
# Concept:
#    Searches for a particular type of object using on-board camera
#    When object is detected, communicate this data to the swarm and
#    perform some kind of swarm behavior
#
# Steps:
#   1. Launch script that takes a photo with on-board camera, 
#      then analyzes photo with darknet YOLOv3 (tiny weights)
#   2. Parse contents of file written by darknet software
#   3. If correct object listed in file, respond accordingly
#   4. If object not found, repeat
#
####################################################################

# IF YOU RECEIVE AN ERROR: 
#   "ImportError: No module named autopilot_bridge.msg"
# THEN:  
#   Go to the folder: ACS/acs_ros_ws/
#   Type: source devel/setup.bash
# Now it should work again BUT ONLY IN THAT TERMINAL
# This is because the autopilot_bridge software (mavlink software 
# written by NPS) needs to be sourced,
# it also may need to be built first using catkin_make.

import numpy as np
import math
import autopilot_bridge.msg as apbrg
import ap_msgs.msg as apmsg
from autopilot_bridge.msg import LLA
import ap_lib.gps_utils as gps
import ap_lib.ap_enumerations as enums
import ap_lib.sasc_scrimmage as ss
import rivercourt_enumerations as usma_enums
import timeit

# new imports for object detection stuff
import os.path
import subprocess
# end new imports

#SAFE_ALT = 100
SAFE_ALT = 10

class ObjectDetResponse(ss.Tactic):

    object_found = False
    darknet_path = "/home/rrc/Ross/darknet_ab/"

    def init_variables(self, params):
        self._id = int(params['id'])
        self._target_id = -1
        self._wp = np.array([0, 0, 0])
        self._max_range = enums.MAX_RANGE_DEF
        self._fov_width = enums.HFOV_DEF
        self._fov_height = enums.VFOV_DEF
        self._own_pose = apbrg.Geodometry()
        self._blues = dict()
        #self._reds = dict()
        self._shot = set()
        self._safe_waypoint = np.array([0, 0, 0])
        self._last_ap_wp = np.array([0, 0, 0])
        self._action = ["None", 0]
        self._vehicle_type = int(params["vehicle_type"])
        self._name = "ObjectDetResponse"
        #self._location = int(params['location'])
        #self._desired_lat = float(usma_enums.WP_LOC[self._location][0])
        #self._desired_lon = float(usma_enums.WP_LOC[self._location][1])

        # Initialize Variables for Waypoint Assignment
        self._subswarm_id = 0
        self._subswarm_ids_list = [] # This is actually a list of all the vehicle IDs in your subswarm
        self._first_tick = True
        self._subswarm_num_to_assign = []
        self._subswarm_wp_assignment = dict()
        self._wp_assigned = []
        for i in range(0, len(usma_enums.WP_LOC)):
            self._wp_assigned.append(False)  

        # Initialize Variables for Sequencing between Waypoints
        self._wp_id_list = []   # List of WPs to be assigned to this UAS
        for i in range(0, len(usma_enums.WP_LOC)):
            self._wp_id_list.append(i)  # Place holder for other logic
        self._wp_id_list_id = 0     # Index within assigned list of WPs
        self._loc = self._wp_id_list[self._wp_id_list_id]
        self._desired_lat = float(usma_enums.WP_LOC[self._loc][0])
        self._desired_lon = float(usma_enums.WP_LOC[self._loc][1])
        self._desired_alt = SAFE_ALT #self._last_ap_wp[2]
        self._original_alt = SAFE_ALT
        self._time_at_wp = 0
        self._time_start = 0
        self._at_wp = False
        
        print "Variables assigned"
    
    def determine_subswarm_id(self):
        # Determine your own subswarm ID
        #      key   value
        # Why is there a blue_id in the for statement???
        for blue_id, blue in self._blues.iteritems():
            if blue.vehicle_id == self._id:
                self._subswarm_id = blue.subswarm_id
                break
        print "self id: ", self._id
        print "subswarm_id: ", self._subswarm_id
        
    def build_vehicle_list(self):
        # Build a list of all vehicle IDs within your subswarm
        blue_in_subswarm = dict()
        i = 0
        # Why is there a blue_id in the for statement???
        for blue_id, blue in self._blues.iteritems():
            if blue.subswarm_id == self._subswarm_id:
                self._subswarm_ids_list.append(blue.vehicle_id)
                self._subswarm_num_to_assign.append(0) 
                blue_in_subswarm[i] = blue
                i = i+1        
        print "subswarm_ids_list: ", self._subswarm_ids_list
        return blue_in_subswarm
        
    def move_to_waypoint(self):
        # Figure out where to go after object was detected
        #elf._loc = self._wp_id_list[0]
        self._desired_lat = 0 #float(usma_enums.WP_LOC[self._loc][0])
        self._desired_lon = 0 #float(usma_enums.WP_LOC[self._loc][1]) 
        #print "Going to wp: ", self._loc
        
    # init function is called as the constructor
    def init(self, params):
        self.init_variables(params)
        
    # Run our detection scripts
    def run_detection(self):
        # First create the file that tells our python script that we're running
        # the detection software
        print(os.path.dirname(os.path.abspath( __file__ )))
        print("Running detection software...")
        f = open(self.darknet_path + "detecting.temp","w+")
        f.write("Detecting...\n")
        f.close()
        # Now call the script that actually does the detection (assuming it's there)
        # subprocess.call(['/home/rrc/Ross/darknet_ab/detect.sh'])
        subprocess.Popen(["sh", self.darknet_path + "detect.sh"])
        print("Detection software complete")

    # If we have a detection data file and it has what we want, then return
    # true, otherwise return false
    def parse_detection_data(self):
        if os.path.isfile(self.darknet_path + "detection_data.txt") == True:
            f = open(self.darknet_path + "detection_data.txt","r")
            while True:
                text = f.readline()
                if not text: 
                    break
                if "bottle" in text:
                    print "********************************************************\n"
                    print "********************************************************\n"
                    print "**************** TARGET OBJECT DETECTED ****************\n"
                    print "********************************************************\n"
                    print "********************************************************\n"
                    # Set some overall boolean to true and then we don't run this anymore
                    self.object_found = True
                    f.close()
                    return True
            f.close()
        return False
    
    # This is what we do before we detect the target object
    def step_autonomy_not_detected(self, t, dt):
        self._loc = self._wp_id_list[14]
        self._desired_lat = float(usma_enums.WP_LOC[self._loc][0])
        self._desired_lon = float(usma_enums.WP_LOC[self._loc][1])
        self._at_wp = False
        
        # Go to desired latitude, longitude, and altitude
        self._wp = np.array([self._desired_lat, self._desired_lon, 
                             self._desired_alt])

        # First, check if the image analysis software is still running
        # If the file does exist, we just wait for it to finish 
        if os.path.isfile(self.darknet_path + "detecting.temp") == True:
            return True

        if self.parse_detection_data() == True:
            return True

        # Run our detection again if we need to
        self.run_detection()

        return True

    # This is what we do after the target object has been detected
    def step_autonomy_detected(self, t, dt):
        self._loc = self._wp_id_list[24]
        self._desired_lat = float(usma_enums.WP_LOC[self._loc][0])
        self._desired_lon = float(usma_enums.WP_LOC[self._loc][1])
        self._at_wp = False
        
        # Go to desired latitude, longitude, and altitude
        self._wp = np.array([self._desired_lat, self._desired_lon, 
                             self._desired_alt])
        return True

    # step_autonomy is basically the "main loop" function called continuously 
    def step_autonomy(self, t, dt):

        # It does not work to put this stuff under the init function.  
        # Probably all the UAVs need to init first before some of this will work.
        if self._first_tick == True:
            self._first_tick = False

            # Set initial altitude settings
            self._desired_alt = self._last_ap_wp[2]
            self._original_alt = self._last_ap_wp[2]

            self.determine_subswarm_id()
            blue_in_subswarm = self.build_vehicle_list()

            # Remove our detection files if we had some legacy there.try:
            try: os.remove(self.darknet_path + "detecting.temp")
            except OSError: pass
            try: os.remove(self.darknet_path + "detection_data.txt")
            except OSError: pass
            try: os.remove(self.darknet_path + "webcamshot.jpg")
            except OSError: pass

        if self.object_found == True:
            return self.step_autonomy_detected(t, dt)
        
        return self.step_autonomy_not_detected(t, dt)
