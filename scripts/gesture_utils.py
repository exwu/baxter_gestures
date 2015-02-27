
from __future__ import division 
import baxter_interface

import argparse
import struct
import sys

import rospy

import numpy as np
import scipy.optimize

from geometry_msgs.msg import (
		PoseStamped,
		Pose,
		Point,
		Quaternion,
		)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
		SolvePositionIK,
		SolvePositionIKRequest,
		)

limbs = None

defaultlimb = "left"

def init(str_limbs): 
	"""
	Initializes the rosnode as well as the limbs.

	Args: 
			limbs: a list of limbs to initialize		

	"""
	rospy.init_node("gestures") 
	global limbs
	limbs = {limb:baxter_interface.Limb(limb) for limb in str_limbs}
	if len(str_limbs) == 1: 
		global defaultlimb
		defaultlimb = str_limbs[0]


def get_pose(limb=defaultlimb): 
	"""
	Gets the current end effector pose of the specified limb. 
	"""
	return limbs[limb].endpoint_pose()

def get_joint_angles(limb=defaultlimb): 
	"""
	Gets the current end effector pose of the specified limb. 
	"""
	return limbs[limb].joint_angles()

def neutral_pose(): 

	#neutral_pose_left = {'left_w0': -0.9828981887145997, 'left_w1': 0.8828059424194337, 'left_w2': 3.0572237067626955, 'left_e0': -0.5296068664123535, 'left_e1': 1.3502865869934082, 'left_s0': 0.6749515458984375, 'left_s1': -0.16490293450927734}
	neutral_pose_left = {'left_w0': -1.0020729485412598, 'left_w1': 0.8214467109741211, 'left_w2': 3.0553062307800296, 'left_e0': -0.43565054326171876, 'left_e1': 1.8680051023132325, 'left_s0': 1.7031021678039553, 'left_s1': 0.07401457293090821}

	return neutral_pose_left


def dict_to_pose(dict_pose): 
	"""
	Converts the dictionary representation of a pose into the object

	Args: 
			dict_pose: the dict to convert

	"""
	ik_pose = Pose( 
			position=dict_pose['position'], 
			orientation=dict_pose['orientation']
			)
	return ik_pose

def get_ik_sol(pose, limb=defaultlimb): 
	"""
	Find a joint configuration to position the end effector through Baxter's IK service

	Args: 
			pose: end effector pose (dict form)
			limb: the limb to move

	Returns:
			the joint configuration if solution exists, else None
	"""

	ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
	iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
	ikreq = SolvePositionIKRequest()
	hdr = Header(stamp=rospy.Time.now(), frame_id='base')
	pose = PoseStamped(
			header=hdr, 
			pose=dict_to_pose(pose)
			)
	ikreq.pose_stamp.append(pose)
	try: 
		rospy.wait_for_service(ns, 100)
		resp = iksvc(ikreq)
	except (rospy.ServiceException, rospy.ROSException), e:
		rospy.logerr("Service call failed: %s" % (e,))

	resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
			resp.result_type)

	if (resp_seeds[0] != resp.RESULT_INVALID):
		# success
			limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
			return limb_joints
	else: 
		print("failure, no solution")
		return None

def align_to_vector(v): 
	"""
	Returns a quaternion such that the end effector is aligned with vector v

	Args: 
			v : the vector to align to

	Returns: 
			A quaternion 
	"""

	def list_to_quaternion(l): 
		return Quaternion(l[0], l[1], l[2], l[3])

	o = [0, 0, 1]
	xyz = np.cross(o, v)
	w = (np.linalg.norm(o)**2 *  np.linalg.norm(v)**2)**0.5 + np.inner(o, v)
	orientation = [xyz[0], xyz[1], xyz[2], w]
	return list_to_quaternion(normalize(orientation))


def get_orientation_to(src, dest): 
	"""
	Get an aligned on a vector from source to destination

	Args: 
	src: the source
	dest: the destination to get the orientation to

	Returns: 
	A quaternion of the orientation
	"""

	return align_to_vector(normalize([dest.x - src.x, dest.y - src.y, dest.z - src.z]))

def normalize(v): 
	"""
	Normalizes a vector

	"""

	norm = np.linalg.norm(v)
	if norm != 0: 
		v = v/norm
	return v



def point_distance(p1, p2): 
	#Calculates the distance between two point objects

	return ((p1.x-p2.x)**2 + (p1.y-p2.y)**2 + (p1.z-p2.z)**2)**0.5


def find_point_at_distance(p, d, c):
	#Finds the point at distance d from point p that is closest 
	#	 to point c

	def dist(x1, y1, z1, x2, y2, z2): 
		return ((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)**0.5

	def objfunc(x, sign=1.0): 
		return sign*dist(x[0], x[1], x[2], c.x, c.y, c.z)

	cons = ({ 'type' : 'eq', 
		'fun' : lambda x : np.array( 
			[(x[0]-p.x)**2		+ (x[1]-p.y)**2 + (x[2]-p.z)**2 - d**2]), 
		})
	res = scipy.optimize.minimize(objfunc,
			[p.x, p.y, p.z], 
			#jac=derivs,
			method='SLSQP', 
			constraints=cons, 
			options={'disp':False})
	x = res['x']
	return Point(x[0], x[1], x[2])



def baxter_point_at(p, limb=defaultlimb): 
	print "here"
	baxter_point_at_position(p, limb)
	print "tere"
	limbs[limb].move_to_joint_positions(neutral_pose())
	print "dondone"

def baxter_point_at_position(p, limb=defaultlimb): 
	"""
	Tell baxter to point at a given point

	Args: 
			p: The point to point at

	Returns: 
			None
	"""

	curr_pose = get_pose(limb)
	distance = min(0.15, point_distance(curr_pose['position'], p))
	point = find_point_at_distance(p, distance, curr_pose['position'])
	orientation = get_orientation_to(point, p)
	pose = make_pose(point, orientation)
	joints = get_ik_sol(pose)
	if joints: 
		limbs[limb].move_to_joint_positions(joints)
	print "done"


def baxter_point_emphatically_at(point, emphasis=0.0, limb=defaultlimb):
	if emphasis == 0.0: 
		baxter_point_at(point)
		return

	limb_length = 0.25 # Probably the length of baxter's first joint

	# get normal point position
	curr_pose = get_pose(limb)
	distance = min(0.15, point_distance(curr_pose['position'], point))
	point_close = find_point_at_distance(point, distance, curr_pose['position'])
	orientation = get_orientation_to(point_close, point)

	# get emphasis point position
	unit_dir = normalize(point_diff(point_close, point))
	v_back = limb_length * emphasis * -1 * unit_dir
	point_far = point_vec_add(point_close, v_back)

	joints_close = get_ik_sol(make_pose(point_close, orientation))

	joints_far = get_ik_sol(make_pose(point_far, orientation))


	if joints_far == None or joints_close == None: 
		print "No solution found"
		return

	### pick a movement method
	def use_position(): 
		limbs[limb].move_to_joint_positions(joints_close)
		limbs[limb].move_to_joint_positions(joints_far)
		limbs[limb].move_to_joint_positions(joints_close)

	def use_velocity(): 
		rate = rospy.Rate(500) # Hz
		time = 1.0 * emphasis # seconds
		# d = v * t = v / f; what unit are these in anyways? m/s??

		joints_diff = {}
		joints_diff_reverse = {}
		for joint in joints_close.keys(): 
			joints_diff[joint] = (joints_far[joint] - joints_close[joint])
			joints_diff_reverse[joint] = (joints_close[joint] - joints_far[joint])

		def moveBack():
			start = rospy.get_time()
			while rospy.get_time() - start < time: 
				limbs[limb].set_joint_velocities(joints_diff)

		def moveForward(): 
			start = rospy.get_time()
			while rospy.get_time() - start < time: 
				limbs[limb].set_joint_velocities(joints_diff_reverse)


		limbs[limb].move_to_joint_positions(joints_close)
		moveBack()
		moveForward()


	use_velocity()
	#use_position()






def point_diff(src, dst): 
	#Return the vector from point src to point dest
	return [dst.x-src.x, dst.y-src.y, dst.z-src.z]

def point_vec_add(point, vector): 
	return Point(point.x + vector[0], point.y + vector[1], point.z + vector[2])



def make_pose(point, orientation): 
	pose =		{
			'position' : point, 
			'orientation' : orientation
			}

	return pose
