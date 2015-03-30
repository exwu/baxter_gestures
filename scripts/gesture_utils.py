
from __future__ import division 
import baxter_interface

import argparse
import struct
import sys
import time

import rospy
import math

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

defaultlimb = None

def default_limb():
    if default_limb == None:
        return "right"
    else: 
        return defaultlimb


def init(str_limbs): 
    # Initializes the rosnode as well as the limbs specified in str_limbs, a list of strings
    rospy.init_node("gestures") 
    global limbs
    limbs = {limb:baxter_interface.Limb(limb) for limb in str_limbs}
    if len(str_limbs) == 1: 
        global defaultlimb
        defaultlimb = str_limbs[0]


#===========================================================================
# point functions
#===========================================================================

def baxter_point_at(p, limb=defaultlimb, min_distance=0.15): 
    #Tell baxter to point at point p


    if not limb: 
        limb=default_limb()
    curr_pose = get_ee_pose(limb)
    distance = min(min_distance, point_distance(curr_pose['position'], p))
    point = find_point_at_distance(p, distance, curr_pose['position'])
    orientation = get_orientation_to(point, p)
    pose = make_pose(point, orientation)
    joints = get_ik_sol(pose)
    if joints: 
        limbs[limb].move_to_joint_positions(joints)
    print "done"

def baxter_point_at_and_back(p, limb=defaultlimb): 
    if not limb: 
        limb=default_limb()

    limbs[limb].move_to_joint_positions(neutral_pose())
    baxter_point_at(p, limb)
    limbs[limb].move_to_joint_positions(neutral_pose())


def baxter_point_emph_and_back(p, limb=defaultlimb): 
    if not limb: 
        limb=default_limb()

    #limbs[limb].move_to_joint_positions(neutral_pose())
    limbs[limb].set_joint_position_speed(0.4)
    original_pose = limbs[limb].joint_angles()
    print "pointing emphatically"
    baxter_point_emphatically_at(p, limb)
    print "sleeping"
    time.sleep(0.5)
    print "back to original"
    limbs[limb].move_to_joint_positions(original_pose)
    print "done"


def baxter_point_emphatically_at(point, limb=defaultlimb):
    if not limb: 
        limb=default_limb()


    animation_distance = 0.15 # Probably the length of baxter's first joint

    # get normal point position
    curr_pose = get_ee_pose(limb)
    distance = min(0.15, point_distance(curr_pose['position'], point))
    point_close = find_point_at_distance(point, distance, curr_pose['position'])
    orientation = get_orientation_to(point_close, point)

    # get emphasis point position
    unit_dir = normalize(point_diff(point_close, point))
    v_back = animation_distance *  -1 * unit_dir
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

        time = 1.0  # seconds



        def moveBack():
            move_to_velocity(time, joints_far)

        def moveForward(): 
            move_to_velocity(time, joints_close)


        limbs[limb].move_to_joint_positions(joints_close)
        moveBack()
        moveForward()
        moveBack()
        moveForward()
        limbs[limb].move_to_joint_positions(joints_close)


    use_velocity()
    #use_position()


#===========================================================================
# sweep functions
#===========================================================================


def baxter_point_downwards(point, limb=defaultlimb, height=0.15,): 
	# point directly downward at the point
	if not limb: 
		limb=default_limb()
	
	position = Point(point.x, point.y, point.z + height)
	#orientation = align_to_vector([0, 0, -1]) 		# solution is degenerate
	orientation = Quaternion(0.5, 1, 0, 0)

	sol = get_ik_sol(make_pose(position, orientation), limb)
	if sol : 
		limbs[limb].move_to_joint_positions(sol)

def best_fit_line(points): 
	# returns the line that best fits the given points in form 
	# ax+by+c = 0

	def point_to_2d_tuple(p): 
		return (p.x, p.y)

	points = map(point_to_2d_tuple, points)

	def dist(line, point): 
		# ax + by +c = 0
		a, b, c = line
		x, y = point
		num = math.fabs(a*x + b*y + c)
		den = (a**2 + b**2)**0.5
		return num/den

	def objfunc(x, sign=1.0): 
		return sign * sum(map(lambda p : dist(x, p), points))

	res = scipy.optimize.minimize(objfunc,
			[1, 1, 1], 
			method='SLSQP', 
			options={'disp':False})
	x = res['x']
	return x

def baxter_sweep_line(points, limb=defaultlimb): 
	# sweep along the best fit line
	if not limb: 
		limb=default_limb()
	closest = min(points, key=lambda p: point_distance(get_ee_pose()['position'], p))
	farthest = max(points, key=lambda p: point_distance(get_ee_pose()['position'], p))
	line = best_fit_line(points)
	def point_on_line_closest(line, point): 
		a, b, c = line
		x = point.x
		y = point.y
		c_x = (b * (b*x - a*y) - a*c)/(a**2 + b**2)
		c_y = (a * (-1*b*x + a*y) - b*c)/(a**2 + b**2)
		return (c_x, c_y)
	def p2p(tup): 
		return Point(tup[0], tup[1], closest.z)
	limbs[limb].set_joint_position_speed(0.1)
	baxter_point_downwards(p2p(point_on_line_closest(line, closest)))
	baxter_point_downwards(p2p(point_on_line_closest(line, farthest)))

	


	


def baxter_sweep_naive(points, limb=defaultlimb): 
	# point at each point in points
	# too many IK failures to really work. 
	if not limb: 
		limb=default_limb()
	for point in points: 
		baxter_point_downwards(point, limb)


def baxter_sweep_less_naive(points, limb=defaultlimb): 
	# point at the closest point available
	if not limb: 
		limb=default_limb()
	limbs[limb].set_joint_position_speed(0.9)
	points = set(points)
	while points: 
		closest = min(points, key=lambda p: point_distance(get_ee_pose()['position'], p))
		baxter_point_downwards(closest, limb)
		points.remove(closest)





#===========================================================================
# util functions
#===========================================================================

def get_ee_pose(limb=defaultlimb): 
    if not limb: 
        limb=default_limb()
    return limbs[limb].endpoint_pose()

def get_joint_angles(limb=defaultlimb): 
    # Gets the current end effector pose of the specified limb. 
    if not limb: 
        limb=default_limb()
    return limbs[limb].joint_angles()

def move_to_neutral(limb=defaultlimb): 
    if not limb: 
        limb=default_limb()
    limbs[limb].move_to_joint_positions(neutral_pose(limb))

def neutral_pose(limb=defaultlimb): 
    if not limb: 
        limb=default_limb()

    neutral_pose_left = {
            'left_w0': -1.0020,
            'left_w1': 0.8214,
            'left_w2': 3.0553,
            'left_e0': -0.4356,
            'left_e1': 1.8680, 
            'left_s0': 1.7031, 
            'left_s1': 0.0740
            }

    neutral_pose_right = {
            'right_s0': -0.7761,
            'right_s1': -0.8260,
            'right_w0': 0.1142,
            'right_w1': 0.4586,
            'right_w2': -0.0329,
            'right_e0': 0.5303,
            'right_e1': 1.9232
            }
    neutral_pose_right = {
            'right_s0': -1.042,
            'right_s1': -0.836,
            'right_w0': -0.167,
            'right_w1': 0.7481,
            'right_w2': -1.472,
            'right_e0': 0.3393,
            'right_e1': 1.7445
            }
    neutral_pose_right = {'right_s0': -0.881655456829834, 'right_s1': -1.4116458184387208, 'right_w0': -0.03988350043945313, 'right_w1': 0.9303593467895508, 'right_w2': -1.60070895032959, 'right_e0': 0.3271214026428223, 'right_e1': 1.853815780041504}


    neutral_pose = { 
            'right': neutral_pose_right, 
            'left' : neutral_pose_left
            }

    return neutral_pose[limb]


def dict_to_pose(dict_pose): 
    # Converts the dictionary representation of a pose into the object

    ik_pose = Pose( 
            position=dict_pose['position'], 
            orientation=dict_pose['orientation']
            )
    return ik_pose

def get_ik_sol(pose, limb=defaultlimb): 
    # Find a joint configuration to position the end effector through Baxter's IK service


    if not limb: 
        limb=default_limb()
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
    # Returns a quaternion such that the end effector is aligned with vector v


    def list_to_quaternion(l): 
        return Quaternion(l[0], l[1], l[2], l[3])

    o = [0, 0, 1]
    xyz = np.cross(o, v)
    w = (np.linalg.norm(o)**2 *  np.linalg.norm(v)**2)**0.5 + np.inner(o, v)
    orientation = [xyz[0], xyz[1], xyz[2], w]
    return list_to_quaternion(normalize(orientation))


def get_orientation_to(src, dest): 
    #  Get an aligned on a vector from source to destination

    return align_to_vector(normalize([dest.x - src.x, dest.y - src.y, dest.z - src.z]))

def normalize(v): 
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
            method='SLSQP', 
            constraints=cons, 
            options={'disp':False})
    x = res['x']
    return Point(x[0], x[1], x[2])


def move_to_velocity(time, target): 
    # move the end effector to the target joint configuration within a certain time
    start_time = rospy.get_time()

    def joint_delta(joints1, joints2): 
        # joints to find the delta between
            joints_d = {}
            for joint in joints1: 
                if joint[-2] == 'w2':
                    continue
                delta = joints2[joint] - joints1[joint]
                if delta > math.pi : 
                    # pick the shorter of the directions to move
                    print "changing delta"
                    delta = delta - (2 * math.pi)
                joints_d[joint] = delta
            return joints_d

    def joint_distance(joints1, joints2): 
        # "distance" between two joints
        joints_distance = reduce(
                lambda acc, x: acc + x**2,
                joint_delta(joints1, joints2).values(), 
                0)**0.5
        return joints_distance


    def closed_loop_velocity(target, time): 
        # calculate velocity according to a target and a time
        if time <= 0: 
            print "resetting time"
            time = 1.00
        joints_d = joint_delta(limbs[limb].joint_angles(), target)
        joints_d = {k : (v / time) for k, v in joints_d.items()}
        return joints_d


    while joint_distance(target, limbs[limb].joint_angles()) > 0.1:
        remaining_time = time - (rospy.get_time() - start_time)
        limbs[limb].set_joint_velocities(closed_loop_velocity(target,remaining_time))

def joint_mirror(joints): 
    #Convert the joint positions in joints to be the same except
    # for the other arm

    old = "right"
    new = "left"
    if "left" in joints.keys()[0]:
        old = "left"
        new = "right"

    new_joints = {}
    for k, v in joints.iteritems():
        k = k.replace(old, new)
        print k
        if k[-2] in {'s0', 'e0', 'w0', 'w2'}:
            new_joints[k] = -v
        else:
            new_joints[k] = v





def point_diff(src, dst): 
    #Return the vector from point src to point dest
    return [dst.x-src.x, dst.y-src.y, dst.z-src.z]

def point_vec_add(point, vector): 
    return Point(point.x + vector[0], point.y + vector[1], point.z + vector[2])



def make_pose(point, orientation): 
    pose ={
            'position' : point, 
            'orientation' : orientation
            }

    return pose
