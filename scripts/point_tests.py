#!/usr/bin/python

import sys
import gesture_utils as gu
from gesture_utils import Point

gu.init(['right', 'left'])


# NOTES: 
	# z is upward
	# x is outward
	# y is towards baxter



bowl_pos5 = gu.Point(x=0.7403782842323192, y=-0.5016241130334161, z=-0.028004569892429124) 


bowl_pos6 = gu.Point(x=0.7683776614718456, y=-0.1784208449096021, z=0.046471016548638944)

bowl_pos7 = gu.Point(x=0.6601031759244996, y=-0.7296362496398088, z=-0.020453668041184973)


points = [ 
		gu.Point(x=0.6727, y=-0.7721, z=-0.0462), 
		gu.Point(x=0.6794, y=-0.5903, z=-0.0466), 
		gu.Point(x=0.5303, y=-0.4200, z=-0.0028), 
		gu.Point(x=0.7631, y=-0.4258, z=-0.0616),
		gu.Point(x=0.7772, y=-0.146, z=-0.063)
		]


thing = Point(x=0.6638638350142058, y=-0.5640027544109408, z=-0.02337577227693624) 

def test_find_point(): 
	print gu.find_point_at_distance(0.15, Point(0, 0, 0), Point(2, 1, 1))

def test_point_at(): 
	gu.baxter_point_at(points[2])

def test_point_emphatically(): 
	gu.baxter_point_emph_and_back(points[2])

def test_get_ee_pose(): 
	print gu.get_ee_pose()

def test_get_joint_angles(): 
	print gu.get_joint_angles('right')

def test_move_to_neutral(): 
	gu.move_to_neutral()

def mirror_joint():
	joints = gu.get_joint_angles(limb='left')
	print gu.joint_mirror(joints)


def main(): 
	tests = {'find_point' : test_find_point, 
			'point_at' : test_point_at, 
			'point_emph' : test_point_emphatically,
			'get_pose' : test_get_ee_pose,
			'mirror_joint' : mirror_joint,
			'joint_angles' : test_get_joint_angles, 
			'neutral': test_move_to_neutral,
			}
	if len(sys.argv) == 2 and sys.argv[1] in tests: 
		tests[sys.argv[1]]()
	else: 
		print "invalid key"
		print tests.keys()


main()


"""
{'position': Point(x=0.8915614604708697, y=-0.2831747869217132, z=0.16890368769205993), 'orientation': Quaternion(x=0.6440021094791013, y=0.7623931194307774, z=0.02553090288184423, w=0.0580188540830826)}

{'position': Point(x=0.8876176434590063, y=-0.0029000055154102945, z=0.42649446241476363), 'orientation': Quaternion(x=0.49951928867630135, y=0.510833709860458, z=0.5535853563717968, w=-0.4278699035001578)}
"""




