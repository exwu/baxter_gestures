#!/usr/bin/python
from gesture_utils import *

import sys

init(['right'])


# NOTES: 
	# z is upward
	# x is outward
	# y is towards baxter



bowl_pos5 = Point(x=0.7403782842323192, y=-0.5016241130334161, z=-0.028004569892429124) 


def test_find_point(): 
	print find_point_at_distance(0.15, Point(0, 0, 0), Point(2, 1, 1))

def test_point_at(): 
	baxter_point_at(bowl_pos5)

def test_point_emphatically(): 
	baxter_point_emph_and_back(bowl_pos5)

def test_get_ee_pose(): 
	print get_ee_pose()

def test_get_joint_angles(): 
	print get_joint_angles()

def test_move_to_neutral(): 
	move_to_neutral()





def main(): 
	tests = {'find_point' : test_find_point, 
			'point_at' : test_point_at, 
			'point_emph' : test_point_emphatically,
			'get_pose' : test_get_ee_pose,
			'joint_angles' : test_get_joint_angles, 
			'neutral': test_move_to_neutral,
			}
	tests[sys.argv[1]]()



main()


"""
{'position': Point(x=0.8915614604708697, y=-0.2831747869217132, z=0.16890368769205993), 'orientation': Quaternion(x=0.6440021094791013, y=0.7623931194307774, z=0.02553090288184423, w=0.0580188540830826)}

{'position': Point(x=0.8876176434590063, y=-0.0029000055154102945, z=0.42649446241476363), 'orientation': Quaternion(x=0.49951928867630135, y=0.510833709860458, z=0.5535853563717968, w=-0.4278699035001578)}
"""




