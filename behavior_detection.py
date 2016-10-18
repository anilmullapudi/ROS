#!/usr/bin/env python
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import math
import random
import rospy
import tf
from std_msgs.msg import String
from std_msgs.msg import Float64
from pr2_LookAround_pkg.msg import user_behavior_command
from face_detector.msg import FaceDetectorActionResult
from people_msgs.msg import PositionMeasurement
from geometry_msgs.msg import PointStamped
from pr2_LookAround_pkg.msg import BehaviorResult;
from sensor_msgs.msg import PointCloud
from ar_track_alvar_msgs.msg import AlvarMarkers


# initilize the node
rospy.init_node("head_bav", anonymous=True)
behaviorPub = rospy.Publisher('/behaviors', BehaviorResult, queue_size=1)
ecludian_dist_pub = rospy.Publisher('/distance_face_marker', Float64, queue_size=1);
dist_bed_face_pub = rospy.Publisher('/distance_bed_face', Float64, queue_size=1);
messagePub = rospy.Publisher("/Messager/SendMsg", String, queue_size=1)
filteredFacePub = rospy.Publisher("/filteredFaces", PointCloud, queue_size=1)

windowSize = 0;
windowCounter = 0;
falsePositiveCounter=0;
falsePositevTagCounter=0;
blackFaceCounter=0;
#requried variables for face detection
xQueue=[]
yQueue=[]
zQueue=[]
faceCoordinate = [xQueue, yQueue, zQueue]
dist_frm_ground=[];
#Marker position and orientation initialization
foundMarker=False;
pos_x=0;
pos_y=0;
pos_z=0;
orient_x=0;
orient_y=0;
orient_z=0;
orient_w=0;
marker_time=0;
	


#Initialize bed origin and add a marker for bed in rviz
bedX=rospy.get_param("~bed_x");
bedY=rospy.get_param("~bed_y");
bedZ=rospy.get_param("~bed_z");
SIT_ID = rospy.get_param("~sitting_id");
SIT_DESC = rospy.get_param("~sitting_desc");
STAND_ID = rospy.get_param("~standing_id");
STAND_DESC = rospy.get_param("~standing_desc");
ONBED_ID = rospy.get_param("~on_bed_id");
ONBED_DESC = rospy.get_param("~on_bed_desc");
OFFBED_ID = rospy.get_param("~off_bed_id");
OFFBED_DESC = rospy.get_param("~off_bed_desc");

CAMERA_FRAME = "/wide_stereo_l_stereo_camera_optical_frame";
#~ CAMERA_FRAME = "/camera_rgb_optical_frame";
BED_FRAME="/bed_frame";						
MAP_FRAME="/map";
MARKER_FRAME="/ar_marker_10";

#behavior threshold values for sitting
MIN_DIST_ST = rospy.get_param("~min_dist_sit");
MAX_DIST_ST = rospy.get_param("~max_dist_sit");

#behavior thresold values for standing
MIN_DIST_STAND = rospy.get_param("~min_dist_stand");

#behavior thresold values for sleepting
MIN_DIST_SLEEP = rospy.get_param("~min_dist_sleep");
ECLUDIAN_DIST = rospy.get_param("~ecludian_dist");

#Filter parameters
TIME_DIFFER = rospy.get_param("~time_diff");
MIN_THRESH = rospy.get_param("~min_dist_face_to_marker");
MAX_THRESH = rospy.get_param("~max_dis_face_to_marker");
BLANK_FACE_COUNT = rospy.get_param("~blacnk_face_count");


prev_bev_id=0;
prev_bev_desc=""

# transformations
br = tf.TransformBroadcaster()
tf2 = tf.TransformListener();	
activationFlag=0;

# creating a topic to publish markers data into rviz
topic = 'visualization_marker_array_bed'
rviz_pub = rospy.Publisher(topic, MarkerArray, queue_size=10)
markerArray = MarkerArray()



def updateMarkerPositions(data):
	#~ print "received positions"
	global foundMarker;
	global pos_x;
	global pos_y;
	global pos_z;
	global orient_x;
	global orient_y;
	global orient_z;
	global orient_w;
	global marker_time;
	
	if(len(data.markers) >=1):
		foundMarker = True;
		pos_x = data.markers[0].pose.pose.position.x;
		pos_y = data.markers[0].pose.pose.position.y;
		pos_z = data.markers[0].pose.pose.position.z;
		orient_x = data.markers[0].pose.pose.orientation.x;
		orient_y = data.markers[0].pose.pose.orientation.y;
		orient_z = data.markers[0].pose.pose.orientation.z;
		orient_w = data.markers[0].pose.pose.orientation.w;
		marker_time = data.markers[0].header.stamp;
	#~ print "position x: ", pos_x;
	#~ print "orientation x: ", orient_x; 


def markFace(x,y,z, seqCount):
			
	#~ print "marking face %0.2f %o.2f %0.2f"%(x, y, z)
	marker = Marker()
	marker.header.frame_id = BED_FRAME
	marker.type = marker.SPHERE
	marker.action = marker.ADD
	marker.scale.x = 0.2
	marker.scale.y = 0.2
	marker.scale.z = 0.2
	marker.color.a = 1.0
	marker.color.r = 0.0
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
	#~ rospy.sleep(1.0)


def publishBehaviors(bevId, bevDesc):
	global prev_bev_id;
	global prev_bev_desc;
	behavior_result = BehaviorResult();
	behavior_result.header.stamp = rospy.Time.now();
	behavior_result.behav_Id = bevId;
	behavior_result.behavior_Desc = bevDesc;
	behavior_result.prev_bev_Id = prev_bev_id;
	behavior_result.prev_bev_desc = prev_bev_desc;
	
	if(bevId != prev_bev_id):
		print "*******sending msg to mobile********"
		messagePub.publish(bevDesc);
	
	prev_bev_id=bevId;
	prev_bev_desc=bevDesc;
	behaviorPub.publish(behavior_result);
	
	

def getTransformedPoint(x,y,z, src_fram, des_frame):
	point = PointStamped()
	point.point.x = x;
	point.point.y = y;
	point.point.z = z;
	point.header.frame_id = src_fram;
	t=rospy.Time();
	point.header.stamp = t;
	
	tf2.waitForTransform(src_fram, des_frame, t, rospy.Duration(12.0));
	desired_frame = tf2.transformPoint(des_frame, point)
	#~ print "successfully transformed";
	return desired_frame;


def receiveCommand(data):
	global activationFlag
	global windowSize
	activationFlag = data.activationFlag;
	windowSize = data.windowSize
		
#detect behaviors, call back for face topic positions
def detectBehavr_with_markers(data):
	global windowCounter;
	global faceCoordinate;
	global dist_frm_ground;
	if(activationFlag == 1):
		if(windowCounter == windowSize): #window size is reached its capacity
			#calculate the average of x,y,z in a particualr window
			xAvg = sum(faceCoordinate[0])/windowSize;
			yAvg = sum(faceCoordinate[1])/windowSize;
			zAvg = sum(faceCoordinate[2])/windowSize;
			dist_from_ground = sum(dist_frm_ground)/windowSize;
			markFace(bedX,bedY,bedZ, 6000);
			
			#ecludianDistance bed to face 
			dist = math.sqrt(math.pow((bedX-xAvg),2) + math.pow((bedY-yAvg),2) +  math.pow((bedZ-zAvg),2))
			print ("ecludianDistance from bed to face: %0.2f")%(dist)
			print("distance from the ground: %0.2f")%(dist_from_ground);
			#~ print "ecludianDistance from marker to face_position:",ecludianDistance ;
			dist_bed_face_pub.publish(dist);
			
			#distance from the ground min and max value thereshold checking for detecting sitting behavior
			if(dist_from_ground > MIN_DIST_ST and dist_from_ground <= MAX_DIST_ST):
				publishBehaviors(SIT_ID, SIT_DESC);
			#minimum distance from the ground to check the stand position
			elif(dist_from_ground > MIN_DIST_STAND):
				publishBehaviors(STAND_ID, STAND_DESC);
			#ecludian distance from the face position to bed and minimum ground distance check for checking the on the bed or lying behavior
			elif(dist < ECLUDIAN_DIST and dist_from_ground <= MIN_DIST_SLEEP ):
				publishBehaviors(ONBED_ID, ONBED_DESC);
			else:
				publishBehaviors(OFFBED_ID, OFFBED_DESC);
				
			#reset counters
			windowCounter=windowCounter-1;
			if(len(faceCoordinate[0]) > 0):
				faceCoordinate[0].pop(0);
				faceCoordinate[1].pop(0);
				faceCoordinate[2].pop(0);
				dist_frm_ground.pop(0);
			
		else:
			try:
				
				#*********Receiving data from stereo camera**********
				#~ x = data.result.face_positions[0].pos.x;
				#~ y = data.result.face_positions[0].pos.y;
				#~ z = data.result.face_positions[0].pos.z;
				#~ print "data: ", data;
				
				#*********Receiving data from kinect camera**********
				global falsePositiveCounter;
				global blackFaceCounter;
				global falsePositevTagCounter;
				num_of_faces = len(data.points);
				closet_face_distance=100; #any arbitary large value, face to bed distance
				closet_face_x=0;
				closet_face_y=0;
				closet_face_z=0;
				if(num_of_faces >= 1):
					#balck counter check
					if(blackFaceCounter>BLANK_FACE_COUNT):
						 blackFaceCounter=0;
						 return;
						 
					for i in range(0,num_of_faces):
						x = data.points[i].x;
						y = data.points[i].y;
						z = data.points[i].z;
						face_time = data.header.stamp;
						print ("face position received x:%0.2f, y:%0.2f, z:%0.2f)")%(x,y,z);
												
						if(foundMarker == True):
							#~ print "face_time:", face_time.secs;
							#~ print "marker_time:", marker_time.secs;
							time_diff = abs(face_time.to_sec()-marker_time.to_sec());
							#~ print "time difference between marker and face positions: ", time_diff;
							if(time_diff <= TIME_DIFFER):
								##false positve detection for face positions (convert face position into marker frame)
								faceCoordinates_markar_frm = getTransformedPoint(x,y,z, CAMERA_FRAME, MARKER_FRAME);
								face_x = faceCoordinates_markar_frm.point.x;
								face_y = faceCoordinates_markar_frm.point.y;
								face_z = faceCoordinates_markar_frm.point.z;
								
								#~ print ("face position with respect to marker frame x:%0.2f, y:%0.2f, z:%0.2f)")%(face_x,face_y,face_z);
								#~ print ("marker position in marker frame            x:%0.2f, y:%0.2f, z:%0.2f)")%(pos_x,pos_y,pos_z);
								ecludianDistance = math.sqrt(math.pow(face_x,2) + math.pow(face_y,2) +  math.pow(face_z,2))
								
								ecludian_dist_pub.publish(ecludianDistance);
								
								
								if(ecludianDistance <= MAX_THRESH):
									blackFaceCounter = 0;
									if(ecludianDistance <= MIN_THRESH):
										falsePositevTagCounter += 1;
										print "False positive detected(due to shorter marker distance). Total number of false positives: ", falsePositevTagCounter;
										return;
									
									if(ecludianDistance < closet_face_distance):
										#~ print ("found a closest face to bed at x y z:%f %f %f ")%(x,y,z);
										closet_face_distance = ecludianDistance;
										closet_face_x = x;
										closet_face_y = y;
										closet_face_z = z;
										windowCounter +=1;
								else:
									falsePositiveCounter += 1;
									print "False positive detected(due to long marker distance). Total number of false positives: ", falsePositiveCounter;
									return;
							else:
								#~ falsePositiveCounter += 1; #not found latest marker position values, ignoring face position values
								print "Ignoring face position due to time delay. Total number of false positives: ", falsePositiveCounter;
								return;
						else:
							#~ windowCounter -= 1;
							return; #marker not found, ignoring face position values
							
					 #else black counter
				else:
					blackFaceCounter +=1;
					return;
				
				filteredFacePub.publish(data);
				#Transformations
				faceCoordinates_bedfrm = getTransformedPoint(closet_face_x,closet_face_y,closet_face_z, CAMERA_FRAME, BED_FRAME);
				faceCoordinates_mapfrm = getTransformedPoint(closet_face_x,closet_face_y,closet_face_z, CAMERA_FRAME, MAP_FRAME);
				print("xyz before transformation: %0.2f %0.2f %0.2f")%(x, y, z);
				print("xyz after transformation to bed: %0.2f %0.2f %0.2f")%(faceCoordinates_bedfrm.point.x, faceCoordinates_bedfrm.point.y, faceCoordinates_bedfrm.point.z);
				faceCoordinate[0].append(faceCoordinates_bedfrm.point.x);
				faceCoordinate[1].append(faceCoordinates_bedfrm.point.y);
				faceCoordinate[2].append(faceCoordinates_bedfrm.point.z);
				dist_frm_ground.append(faceCoordinates_mapfrm.point.z);
			except rospy.ROSInterruptException: pass
			
def analyzeBehaviors():
	rospy.Subscriber("/behaviorCommandTopic", user_behavior_command, receiveCommand)
	
	#*********Receiving data from kinect camera**********
	rospy.Subscriber("/face_detector/faces_cloud", PointCloud, detectBehavr_with_markers)
	
	#*********Receiving data from stereo camera**********
	#~ rospy.Subscriber("/face_detector_action/result", FaceDetectorActionResult, detectBehaviors)
	
	#*********Receiving data from marker**********
	rospy.Subscriber("/ar_pose_marker", AlvarMarkers, updateMarkerPositions)
	
	while not rospy.is_shutdown():
		print "behavior detection started."
		rospy.spin();

if __name__ == '__main__':
	try:
		analyzeBehaviors()
	except rospy.ROSInterruptException: pass

	
