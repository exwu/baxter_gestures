#!/usr/bin/python
from gesture_utils import *

import sys

init(['left'])


# NOTES: 
	# z is upward
	# x is outward
	# y is towards baxter


bowl_pos = Point(0.85, -0.09, 0.00)
# 0.15 is a good distance. get points on a sphere, pick one?


bowl_pos2 = Point(x=0.553891693671548, y=0.6621980379880802, z=0.032158606247587984)


bowl_pos3 = Point(x=0.6706101699747452, y=0.4837439381806709, z=0.016091503067306695)

bowl_pos4 = Point(x=0.6752933152172509, y=0.66478088155743593, z=-0.02631370211618276)



def test_find_point(): 
	print find_point_at_distance(0.15, Point(0, 0, 0), Point(2, 1, 1))

def test_point_at(): 
	baxter_point_at(bowl_pos4)

def test_point_emphatically(): 
	baxter_point_emphatically_at(bowl_pos3, 1.0)

def test_get_pose(): 
	print get_pose()

def test_get_joint_angles(): 
	print get_joint_angles()




def main(): 
	tests = {'find_point' : test_find_point, 
			'point_at' : test_point_at, 
			'point_emph' : test_point_emphatically,
			'get_pose' : test_get_pose,
			'joint_angles' : test_get_joint_angles, 
			}
	tests[sys.argv[1]]()



main()


"""
{'position': Point(x=0.8915614604708697, y=-0.2831747869217132, z=0.16890368769205993), 'orientation': Quaternion(x=0.6440021094791013, y=0.7623931194307774, z=0.02553090288184423, w=0.0580188540830826)}

{'position': Point(x=0.8876176434590063, y=-0.0029000055154102945, z=0.42649446241476363), 'orientation': Quaternion(x=0.49951928867630135, y=0.510833709860458, z=0.5535853563717968, w=-0.4278699035001578)}
"""




