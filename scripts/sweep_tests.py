#!/usr/bin/python
from gesture_utils import * 
import time


bowl_pos5 = Point(x=0.7403782842323192, y=-0.5016241130334161, z=-0.028004569892429124) 

points = [ 
		Point(x=0.6727, y=-0.7721, z=-0.0462), 
		Point(x=0.6794, y=-0.5903, z=-0.0466), 
		Point(x=0.5303, y=-0.4200, z=-0.0028), 
		Point(x=0.7631, y=-0.4258, z=-0.0616),
		Point(x=0.7772, y=-0.146, z=-0.063)
		]

points2 = [
		Point(x=0.609501660118675, y=-0.7160124833156033, z=0.1990290350128524),
		Point(x=0.6862176747936293, y=-0.3896140550273823, z=0.15488784679815754)
		]

points3_left = [
	Point(x=0.6116892634541824, y=0.8401892705740867, z=-0.053082826059595876), 
	Point(x=0.6076720474394917, y=0.567320106965907, z=-0.014334800573879834),
	Point(x=0.6933705381898314, y=0.34379881517339006, z=-0.04285888350578518),
	Point(x=0.8005323006905335, y=0.5336026522507158, z=-0.028615759990686253)
	]


init(["left", 'right']);

def test_naive(): 
	baxter_sweep_naive(points3_left)
	time.sleep(0.5)
	baxter_move(neutral_pose())

def test_less_naive(): 
	baxter_sweep_less_naive(points3_left, limb='left')
	time.sleep(0.5)
	baxter_move(neutral_pose())

def test_point_down(): 
	baxter_point_downwards(points[0])

def test_best_fit_line(): 
	print best_fit_line(points)

def test_sweep_line(): 
	baxter_sweep_line(points3_left)
	time.sleep(0.5)
	baxter_move(neutral_pose())

def test_baxter_move(): 
	baxter_move(neutral_pose())

def test_move_to_neutral(): 
	move_to_neutral()

def test_forearm_vector(): 
	print baxter_forearm_vector(limb='right')

def main(): 
	tests = {
			'naive' : test_naive, 
			'less_naive' : test_less_naive, 
			'point_down' : test_point_down,
			'bfl' : test_best_fit_line, 
			'line' : test_sweep_line,
			'neutral': test_move_to_neutral,
			'record' : point_recorder,
			'move' : test_baxter_move, 
			'forearm' : test_forearm_vector
			}
	if len(sys.argv) == 2 and sys.argv[1] in tests: 
		tests[sys.argv[1]]()
	else: 
		print "invalid key"
		print tests.keys()
main()
