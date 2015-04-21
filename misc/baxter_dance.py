#!/usr/bin/python
import baxter_interface
import baxter_interface.settings
import rospy
import gesture_utils
import sys

rospy.init_node("baxter_dance")

gesture_utils.init(['left', 'right'], False)

right_limb = baxter_interface.Limb('right')
left_limb = baxter_interface.Limb('left')




# more twisting air traffic controller like
right_left = {'left_w0': 0.5725583284240723, 'left_w1': 0.5832961939270019, 'left_w2': -3.029612052612305, 'left_e0': -2.7419906552124025, 'left_e1': 1.091043834136963, 'left_s0': 0.05330583231811524, 'left_s1': -0.7278738830200195}

left_left = {'left_w0': -0.32635441225, 'left_w1': 0.567189395673, 'left_w2': 0.396534033215, 'left_e0': -0.0728640873413, 'left_e1': 0.0962572943298, 'left_s0': 0.662296204413, 'left_s1': -1.12977684899}

# more like waving
right_left = {'left_w0': -0.3305728594116211, 'left_w1': 0.4701651109497071, 'left_w2': 0.40880587950439456, 'left_e0': -0.05599029869384766, 'left_e1': -0.050237870745849615, 'left_s0': 0.46939812055664065, 'left_s1': -1.4423254341613772}

left_left = {'left_w0': -0.22319420438232423, 'left_w1': 0.8130098166503906, 'left_w2': 0.43219908649291994, 'left_e0': 0.10852914061889649, 'left_e1': 0.6676651371643066, 'left_s0': 0.5840631843200684, 'left_s1': -1.12900985859375}




right_right = gesture_utils.joint_mirror(left_left)
left_right = gesture_utils.joint_mirror(right_left)

def try_float(x):
    try:#{{{
        return float(x)
    except ValueError:
        return None#}}}
def clean_line(line, names):
	"""#{{{
	Cleans a single line of recorded joint positions

	@param line: the line described in a list to process
	@param names: joint name keys
	"""
	#convert the line of strings to a float or None
	line = [try_float(x) for x in line.rstrip().split(',')]
	#zip the values with the joint names
	combined = zip(names[1:], line[1:])
	#take out any tuples that have a none value
	cleaned = [x for x in combined if x[1] is not None]
	#convert it to a dictionary with only valid commands
	command = dict(cleaned)
	left_command = dict((key, command[key]) for key in command.keys()
			if key[:-2] == 'left_')
	right_command = dict((key, command[key]) for key in command.keys()
			if key[:-2] == 'right_')
	return (command, left_command, right_command, line)#}}}

def getstuff(): 
	filename = 'left.txt'#{{{
	with open(filename, 'r') as f:
		lines = f.readlines()

	keys = lines[0].rstrip().split(',')

	_cmd, lcmd_start, rcmd_start, _raw = clean_line(lines[1], keys)

	print lcmd_start
	print rcmd_start#}}}

def arms_left(): 
	move_arms(left_left, left_right)
	rospy.sleep(0.1)

def arms_right(): 
	move_arms(right_left, right_right)
	rospy.sleep(0.1)

def move_arms(left_pos, right_pos):

	threshold = baxter_interface.settings.JOINT_ANGLE_TOLERANCE * 10
	def genf(limb, joint, angle): 
		def joint_diff(): 
			return abs(angle - gesture_utils.get_joint_angles(limb)[joint])
		return joint_diff

	diffs_left = [genf("left", j, a) for j, a in left_pos.items()]
	diffs_right = [genf("right", j, a) for j, a in right_pos.items()]
	stop_condition = lambda: (all(diff() < threshold for diff in diffs_left) and all (diff() < threshold for diff in diffs_right))


	timeout_time = rospy.get_time() + 30.0
	rate = rospy.Rate(100)

	# Code for the faces either goes here, or inside the loop. 
	while not stop_condition(): 
		if rospy.get_time() >= timeout_time:
			print("baxter_move timed out")
			return
		if rospy.is_shutdown():
			return 

		right_limb.set_joint_positions(right_pos)
		left_limb.set_joint_positions(left_pos)


def main(): 
	if len(sys.argv) != 2: 
		print("usage: baxter_dance.py <number_of_waves>")
		return 
	speed = 0.4
	right_limb.set_joint_position_speed(speed)
	left_limb.set_joint_position_speed(speed)
	for i in xrange(int(sys.argv[1])):
		arms_left()
		arms_right()
	print "done"




main()
