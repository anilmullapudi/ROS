#!/usr/bin/env python
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import math
import random
import rospy
import traceback
import tf
from std_msgs.msg import String
from pr2_LookAround_pkg.msg import face_emu_command
from face_detector.msg import FaceDetectorActionResult
from people_msgs.msg import PositionMeasurement
from geometry_msgs.msg import PointStamped


# creating topic 'face_info' to publish face emulator data
pub = rospy.Publisher('/face_detector_action/result', FaceDetectorActionResult, queue_size=10)
rospy.init_node('face_emulator', anonymous=True)
r = rospy.Rate(10) # 10hz
faces = FaceDetectorActionResult();
singleFace = PositionMeasurement();
faces.result.face_positions.append(singleFace);

# creating a topic to publish markers data into rviz
topic = 'visualization_marker_array'
rviz_pub = rospy.Publisher(topic, MarkerArray, queue_size=10)
markerArray = MarkerArray()

# stop flag to stop emulation
stop = False;

# transformations
tf2 = tf.TransformListener();				
# Transform target point to camera reference frame
CAMERA_FRAME = "/wide_stereo_optical_frame";
MAP_FRAME = "/map";

def markFace(x,y,z, seqCount):
	if(seqCount > 1): #replace 2 with MARKERS_MAX constant.
		markerArray.markers.pop(0);
		
	#print "marking face %0.2f %o.2f %0.2f"%(x, y, z)
	marker = Marker()
	marker.header.frame_id = CAMERA_FRAME
	marker.type = marker.SPHERE
	marker.action = marker.ADD
	marker.scale.x = 0.2
	marker.scale.y = 0.2
	marker.scale.z = 0.2
	marker.color.a = 1.0
	marker.color.r = 1.0
	marker.color.g = 1.0
	marker.color.b = 0.0
	marker.pose.orientation.w = 1.0
	marker.pose.position.x = x
	marker.pose.position.y = y
	marker.pose.position.z = z
	#marker.id = seqCount+mod;
	#marker.lifetime = rospy.Duration(0.2)
	markerArray.markers.append(marker)
	rviz_pub.publish(markerArray)
	rospy.sleep(1.0)

# Create a callback function for the subscriber.
def callback(data):
	#Simply print out values in our custom message.
	#~ rospy.loginfo(" Face Mode: %d", data.mode)
	#~ rospy.loginfo(" Origin_X : %d", data.x)
	#~ rospy.loginfo(" Origin_Y : %d", data.y)
	#~ rospy.loginfo(" Origin_Z : %d", data.z)
	#~ rospy.loginfo(" velocity : %d", data.velocity)
	
	
	
	#Convert the map fram co-ordinates into camera frame
	try:
		
		#while not rospy.is_shutdown():
		#~ print "data.mode: ", data.mode;
		global stop;
		if(data.mode == 73):
			print "****************exit Command received*******************"
			
			stop = True;
			#break;
		if(data.mode == 67):
			print "****************Command received to emulate Circle*******************"
			#~ global stop;
			stop = False;
			generateCirclePoints(data.x, data.y, data.z)
		elif(data.mode == 85): 
			print "****************Command received to emulate UpandDown*******************"
			#~ global stop;
			stop = False;
			generateUpandDownPoints(data.x, data.y, data.z)
		elif(data.mode == 83):
			print "****************Command received to emulate Left and Right*******************"
			#~ global stop;
			stop = False;
			generateLeftRightPoints(data.x, data.y, data.z)
			#r.sleep()
		print "exited from while loop"
	except Exception as e:
		print "exception is******************************************************\n ";
		print e.message;
		print "exception is******************************************************\n ";
		traceback.print_exc();
	
	


# This method is to generate the circle co-ordinates if the input command is 'C'
def generateCirclePoints(origin_x, origin_y, origin_z):
		print "Entering into generateCirclePoints() method"
		while not rospy.is_shutdown():
			#numberOfPoints: number of points requried to generate for a circle
			#radius: radias of the circle motions reqried for face tracker emulation
			del markerArray.markers[:]
			
			numberOfPoints = 24
			radius = 1;
			PI =  math.pi;
			PI_VAL_DEGREES = 180;
			circle_angles = []
			eachAngleValue = 360/(numberOfPoints-1);
			circle_angles.append(0);
			sequence = 1;
			
			for i in range(1, numberOfPoints):
				circle_angles.append(circle_angles[i-1]+ (PI/(PI_VAL_DEGREES/eachAngleValue)));

			for angle in circle_angles:
				faces.header.stamp = rospy.Time.now();
				faces.header.seq = sequence;
				faces.result.face_positions[0].header.stamp = rospy.Time.now();
				faces.result.face_positions[0].header.seq=sequence;
				sequence = sequence+1;
				#faces.result.face_positions[0].header = rospy.Time.now();
				
				x = radius * math.cos(angle) + origin_x;
				y = radius * math.sin(angle) + origin_y;
				z = origin_z;
				
				camera_target = getTransformedPoint(x,y,z);
				faces.result.face_positions[0].pos.x = camera_target.point.x
				faces.result.face_positions[0].pos.y = camera_target.point.y
				faces.result.face_positions[0].pos.z = camera_target.point.z
				faces.result.face_positions[0].reliability = 1.0
				markFace(camera_target.point.x,camera_target.point.y,camera_target.point.z, sequence-1);
				
				#faces.command = command;
				#~ rospy.loginfo(faces)
				pub.publish(faces) 
				global stop;
				#~ print "***********published face successfully********"
				#~ print stop;
				if(stop == True):
					return;
				r.sleep()
			r.sleep()
		print "Leaving from generateCirclePoints() method"

def getTransformedPoint(x,y,z):
	point = PointStamped()
	point.point.x = x;
	point.point.y = y;
	point.point.z = z;
	point.header.frame_id = MAP_FRAME;
	point.header.stamp = rospy.Time();
	#~ print "before"
	tf2.waitForTransform(CAMERA_FRAME, point.header.frame_id, rospy.Time(), rospy.Duration(7.0));
	camera_target = tf2.transformPoint(CAMERA_FRAME, point)
	#print camera_target
	#~ print "successfully transformed x:%f y:%f z:%f"%(camera_target.point.x, camera_target.point.y, camera_target.point.z);
	return camera_target;

		
def generateUpandDownPoints(origin_x, origin_y, origin_z):
	while not rospy.is_shutdown():
		del markerArray.markers[:]
		offset = 0.2;
		noOfPoints = 20;
		positive_Counter = origin_z;
		negative_Counter = origin_z;
		sequence = 1;
		count = 0;
		#generating points for looking up
		for i in range(0, noOfPoints):
			if(count == 5 or count == 15):
				offset=offset*-1;
			faces.header.stamp = rospy.Time.now();
			faces.header.seq = sequence;
			faces.result.face_positions[0].header.stamp = rospy.Time.now();
			faces.result.face_positions[0].header.seq=sequence;
			sequence = sequence+1;
			positive_Counter = positive_Counter + offset;
			camera_target = getTransformedPoint(origin_x,origin_y,positive_Counter);
			faces.result.face_positions[0].pos.x = camera_target.point.x
			faces.result.face_positions[0].pos.y = camera_target.point.y
			faces.result.face_positions[0].pos.z = camera_target.point.z
			#~ faces.result.face_positions[0].pos.x = -1*camera_target.point.y
			#~ faces.result.face_positions[0].pos.y = -1*camera_target.point.z
			#~ faces.result.face_positions[0].pos.z = camera_target.point.x
			
			#~ faces.result.face_positions[0].pos.x = origin_x;
			#~ faces.result.face_positions[0].pos.y = positive_Counter;
			#~ faces.result.face_positions[0].pos.z = origin_z;
			faces.result.face_positions[0].reliability = 1.0
			markFace(faces.result.face_positions[0].pos.x ,faces.result.face_positions[0].pos.y ,faces.result.face_positions[0].pos.z, sequence-1);
			#rospy.loginfo(faces);
			pub.publish(faces);
			count = count+1;
			if(stop == True):
				return;
		r.sleep()
			
	
	#~ #generating points for looking down
	#~ for i in range(0, noOfPoints):
			#~ faces.header.stamp = rospy.Time.now();
			#~ faces.header.seq = sequence;
			#~ faces.result.face_positions[0].header.stamp = rospy.Time.now();
			#~ faces.result.face_positions[0].header.seq=sequence;
			#~ sequence = sequence+1;
			#~ faces.result.face_positions[0].pos.x = origin_x;
			#~ negative_Counter = negative_Counter - .20;
			#~ faces.result.face_positions[0].pos.y = negative_Counter;
			#~ faces.result.face_positions[0].pos.z = origin_z;
			#~ faces.result.face_positions[0].reliability = 1.0
			#~ markFace(origin_x,negative_Counter,origin_z, sequence-1);
			#~ rospy.loginfo(faces);
			#~ pub.publish(faces);


			
def generateLeftRightPoints(origin_x, origin_y, origin_z):
	while not rospy.is_shutdown():
		offset = 0.2;
		noOfPoints = 20;
		positive_Counter = origin_y;
		negative_Counter = origin_y;
		sequence = 1;
		count = 0;
		#generating points for looking up
		for i in range(0, noOfPoints):
			if(count == 5 or count == 15):
				offset=offset*-1;
			faces.header.stamp = rospy.Time.now();
			faces.header.seq = sequence;
			faces.result.face_positions[0].header.stamp = rospy.Time.now();
			faces.result.face_positions[0].header.seq=sequence;
			sequence = sequence+1;
			positive_Counter = positive_Counter + offset;
			
			camera_target = getTransformedPoint(origin_x,positive_Counter,origin_z);
			faces.result.face_positions[0].pos.x = camera_target.point.x
			faces.result.face_positions[0].pos.y = camera_target.point.y
			faces.result.face_positions[0].pos.z = camera_target.point.z
			#~ faces.result.face_positions[0].pos.x = -1*camera_target.point.y
			#~ faces.result.face_positions[0].pos.y = -1*camera_target.point.z
			#~ faces.result.face_positions[0].pos.z = camera_target.point.x
			
			#~ faces.result.face_positions[0].pos.y = origin_y;
			#~ faces.result.face_positions[0].pos.x = positive_Counter;
			#~ faces.result.face_positions[0].pos.z = origin_z;
			faces.result.face_positions[0].reliability = 1.0
			markFace(camera_target.point.x,camera_target.point.y,camera_target.point.z, sequence-1);
			#~ rospy.loginfo(faces);
			pub.publish(faces);
			count = count+1;
			if(stop == True):
				return;
			r.sleep();
		r.sleep();
	
	#~ #generating points for looking down
	#~ for i in range(0, noOfPoints):
			#~ faces.header.stamp = rospy.Time.now();
			#~ faces.header.seq = sequence;
			#~ faces.result.face_positions[0].header.stamp = rospy.Time.now();
			#~ faces.result.face_positions[0].header.seq=sequence;
			#~ sequence = sequence+1;
			#~ faces.result.face_positions[0].pos.y = origin_y;
			#~ negative_Counter = negative_Counter - offset;
			#~ faces.result.face_positions[0].pos.x = negative_Counter;
			#~ faces.result.face_positions[0].pos.z = origin_z;
			#~ faces.result.face_positions[0].reliability = 1.0
			#~ rospy.loginfo(faces);
			#~ markFace(negative_Counter,origin_y,origin_z, sequence-1);
			#~ pub.publish(faces);

	


def emulator():
	#subcribe face emulator command
	print "********Emulation Started************";
	rospy.Subscriber("face_emu_commnad", face_emu_command, callback);
	rospy.spin()

if __name__ == '__main__':
	try:
		emulator()
	except rospy.ROSInterruptException: pass
