#!/usr/bin/env python

import json

import rospy, roslib
from std_msgs.msg import String

from digilab_control.support import Support
from digilab_control.sensors import Sensors
from digilab_control.cModelGripper import CModelGripper

class RobotSystem(Support, Sensors, CModelGripper):
    def __init__(self):
        # Init
        rospy.init_node('RobotSystem')
        self.rate = rospy.Rate(10)
        Support.__init__(self)
        Sensors.__init__(self)
        CModelGripper.__init__(self)

        # Robot publisher
        self.urScriptPub = rospy.Publisher("/ur_driver/URScript", String, queue_size=1)

        # get poses from json
        #self.robotPoses = {}
        #self.filePath = roslib.packages.get_pkg_dir('pick_place')+"/src/robot_poses.json"
        #with open(self.filePath) as configFile:
            #self.robotPoses = json.load(configFile)

        # some wait so that everything is setup
        rospy.sleep(1)
		
		# collecting other types of pick poses
        self.robotPoses = {}
        self.filePath = roslib.packages.get_pkg_dir('pick_place')+"/src/robot_pick_poses.json"
        with open(self.filePath) as configFile:
            self.robotPoses = json.load(configFile)

        # some wait so that everything is setup
        rospy.sleep(1)
        print self.robotPoses
        # Start sequence
        rospy.loginfo(rospy.get_caller_id() + " started...")
        if self.sequence():
            print "Done"
        else:
            print "Problem. Stopped!"

    # sequence --------------------------------------------
    def sequence(self):
        self.resetGripper()
        self.activateGripper()
        
        #if not self.moveToPose("pose3", 7):
            #return False
        
        #rospy.sleep(2)
        #self.positionGripper(100)
        #if not self.moveToPose("pose4"),7:
            #return False
            
        #pick and place----------    
        #if not self.moveToPose( "pose8" ,7):
            #return False
        #self.positionGripper(100)
        #rospy.sleep(2)
        #if not self.moveToPose( "pose9",5):
            #return False
        #self.closeGripper()
        #rospy.sleep(2)
        #self.closeGripper()
        #if not self.moveToPose( "pose10" ):
            #return False
        #rospy.sleep(2)
        #if not self.moveToPose( "pose11" ,7):
            #return False
        #rospy.sleep(2)
        #if not self.moveToPose( "pose12" ,7):
            #return False
        #self.positionGripper(100)
        #rospy.sleep(2)
        #if not self.moveToPose( "pose13" ):
            #return False
        #rospy.sleep(2)
        
        
        
        #self.closeGripper()
        #rospy.sleep(2)
        #self.positionGripper(100)
        #if not self.moveToPose("pose4"):
            #return False
        self.positionGripper(100)    
        if not self.Pick_Place( "pick1", "PICK" ,7):
            return False
        
        rospy.sleep(2)
        if not self.Pick_Place( "place1", "PLACE" ,7):
            return False
        rospy.sleep(2)
        self.positionGripper(100)
        if not self.Pick_Place( "pick2", "PICK" ,7):
            return False
        rospy.sleep(2)
        if not self.Pick_Place( "place2", "PLACE" ,7):
            return False
        rospy.sleep(2)
        #if not self.moveToPose( "pose9",5):
            #return False
        #self.closeGripper()
        #rospy.sleep(2)
        #self.closeGripper()
        #if not self.moveToPose( "pose10" ):
            #return False
        #rospy.sleep(2)
        #if not self.moveToPose( "pose11" ,7):
            #return False
        #rospy.sleep(2)
        #if not self.moveToPose( "pose12" ,7):
            #return False
        #self.positionGripper(100)
        #rospy.sleep(2)
        #if not self.moveToPose( "pose13" ):
            #return False
        #rospy.sleep(2)
        
        #self.closeGripper()
        #rospy.sleep(2)
        #self.positionGripper(100)
        #if not self.moveToPose("pose4"):
            #return False
        return True

    # move to pose------------------------------------------
    def moveToPose(self, poseName, pathTime = 5):
        print "pose:", poseName
        pose = self.robotPoses.get(poseName)
        print pose
        if pose:
            # create command
			command = self.urSrciptToString(
                move="movej",
                jointPose=pose,
                a=2.0,
                v=2.0,
                t=pathTime,
                r=0)
			print command    
            # publish command to robot
			self.urScriptPub.publish(command)
            
            # wait pathTime
			rospy.sleep(pathTime+0.1)
            
            # robot at pose?
			if self.comparePose(pose):
				return True
			else:
				print "Robot is not at pose"
				return False
        print poseName, "is not in", self.filePath
        return False

	#Do pick  action
    def Pick_Place(self, Name, pickOrPlace, pathTime = 5):
		print "Name:", Name
		pose = self.robotPoses.get(Name)
		#print pose
		if pose:
		  
		  for i in [1,2,3]:
			poseName="pose"+str(i)
			#print poseName
			pos=pose.get(poseName)
			#print pos
            # create command
			command = self.urSrciptToString(
                move="movej",
                jointPose=pos,
                a=1.0,
                v=1.0,
                t=pathTime,
                r=0)
			print command    
            # publish command to robot
			self.urScriptPub.publish(command)
            
            # wait pathTime
			rospy.sleep(pathTime+0.1)
            
            # robot at pose?
			if self.comparePose(pos):
				if i == 3:
					return True
				else: 
					if i == 2:
						if pickOrPlace is "PICK":
						  self.closeGripper()
						else:
						  self.positionGripper(100)
					pass
			else:
				print "Robot is not at pose"
				return False
				
		print Name, "is not in", self.filePath
		return False
		
	
		

# Here is the main entry point
if __name__ == '__main__':
    try:
        RobotSystem()
    except rospy.ROSInterruptException:
        pass
