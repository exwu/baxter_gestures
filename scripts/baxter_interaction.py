#!/usr/bin/python
from __future__ import division
from interaction import * 
import gesture_utils as gu
from gesture_utils import Point
import sys
import rospy
from gesture_rec.msg import BayesFilterStateDist

class Object:
	def __init__(self, name, location): 
		# location is an xyz coordinate, represented as point
		self.name = name
		self.location = location

	def __repr__(self): 
		return "Obj(" + self.name + " : " + str(self.location) +  ")"

class One_Obj_State(State): 
	def __init__(self, obj): 
		self.obj = obj

	def transition_function(self, s_next, object_switch_prob=0.1): 
		# output the probability of transitioning from 
		# this function to the next
		if self.obj == s_next.obj:
			return 1-object_switch_prob
		else: 
			return object_switch_prob
	
	def __repr__(self): 
		return str(self.obj)
	
class Multi_Obj_State(State): 
	def __Init__(self, objects): 
		self.objects = objects
	
class Point_Gesture(Observation): 
	def __init__(self, start_pose, target, limb): 
		# target is a xyz location (point)
		self.target = target
		self.ee_pose, self.joint_angles = gu.baxter_point_pos(target, start_pose=start_pose, limb=limb)
		self.limb = limb

	def execute(self): 
		gu.baxter_execute_joint_positions([self.joint_angles], limb=self.limb)

	def prob_given_state(self, state, variance=0.4): 
		origin = gu.baxter_w1_position(self.ee_pose, self.limb)
		sample = gu.angle_between(origin, self.ee_pose['position'], state.obj.location)
		r =  scipy.stats.norm(0.0, math.sqrt(variance)).pdf(sample)
		print "prog_given", self, state, r, sample
		return r

	def __repr__(self): 
		return "Point at " + (str(self.target).replace('\n', ' '))

class Point_Emph(Observation): 
	def __init__(self, target, limb):
		self.target = target
		self.limb = limb

	def execute(self): 
		gu.baxter_point_emphatically_at_execute(self.joints_close, self.joints_far, self.limb)

def Do_Wait(): 
	def __init__(self): 
		pass
	def prob_given_state(self, state): 
		return 1
	def execute(self):
		pass

class NaiveSweep(Observation): 
	def __init__(self, targets): 
		# targets is a list of objects
		pass
	
	def execute(self): 
		gesture_utils.baxter_sweep_less_naive(map(lambda t: tup_to_point(t.location), self.targets))

	def prob_given_state(self, state) : 
		pass


points = [ 
		gu.Point(x=0.6727, y=0.7721, z=-0.0462), 
		gu.Point(x=0.6794, y=0.5903, z=-0.0466), 
		gu.Point(x=0.5303, y=0.4200, z=-0.0028), 
		gu.Point(x=0.7631, y=0.4258, z=-0.0616),
		gu.Point(x=0.7772, y=0.146, z=-0.063)
		]

class MeldonBayesFilterWrapper(BayesFilter): 
	def __init__(self, state_maker, title="Bayes Filter", subplot_num=None,) : 
		rospy.Subscriber('mm_bf_state', BayesFilterStateDist, self.dist_callback, queue_size=10)
		self.belief = Belief.make_uniform([One_Obj_State(Object("obj " + str(p.x), p)) for p in points])
		self.state_maker = state_maker

	def dist_callback(self,data): 
		states = [ self.state_maker(name) for name in data.names] 
		probs = data.probs
		self.belief = Belief(states, probs)

	def update(self, belief, observation=None): 
		return self.belief

	def advance(self, belief, observation=None): 
		raise NotImplementedError("Advance is not available for this")


#def interaction_loop(robot_bf, human_bf, actions, observation_generator, heuristic): 
		
points = [ 
		gu.Point(x=0.6727, y=0.7721, z=-0.0462), 
		gu.Point(x=0.6794, y=0.5903, z=-0.0466), 
		gu.Point(x=0.5303, y=0.4200, z=-0.0028), 
		gu.Point(x=0.7631, y=0.4258, z=-0.0616),
		gu.Point(x=0.7772, y=0.146, z=-0.063)
		]
points = [
	gu.Point(x=0.6116892634541824, y=0.8401892705740867, z=-0.053082826059595876), 
	gu.Point(x=0.6076720474394917, y=0.567320106965907, z=-0.014334800573879834),
	gu.Point(x=0.6933705381898314, y=0.34379881517339006, z=-0.04285888350578518),
	gu.Point(x=0.8005323006905335, y=0.5336026522507158, z=-0.028615759990686253)
	]

#right
points = [
		 Point(x=0.6196082261574809, y=-0.7983807540636199, z=-0.04644755255270027), 
		 Point(x=0.4126024589979365, y=-0.6430370954167054, z=-0.054708151919178216),
		 Point(x=0.47190055611854664, y=-0.39567639471296134, z=-0.03762194048569241), 
		 Point(x=0.7128883274797874, y=-0.4259662521599078, z=-0.04938158447524294),
		 ]


def test(): 
	gu.init(['left'])
	origin = gu.Point(0, 0, 0)
	while True: 
		rospy.Rate(30).sleep()


#def interaction_loop(robot_bf, human_bf, actions, observation_generator, heuristic): 
#	def __init__(self, states, transition_f, observation_f, initial_belief, title="Bayes Filter", subplot_num=None): 
def main(): 
	#robot_bf = MeldonBayesFilterWrapper(title="Robot Belief", subplot_num=211) 


	limb = 'right'
	gu.init([limb])
	start_joints, start_pose = gu.neutral_pose(limb=limb)

	def one_obj_state_maker(name): 
		return One_Obj_State(Object(name, origin))
	meldon = MeldonBayesFilterWrapper(one_obj_state_maker)

	states = [One_Obj_State(Object("obj " + str(p.x), p)) for p in points]

	robot_bf = BayesFilter(states, 
		lambda s, s_p: s.transition_function(s_p), 
		lambda s, o: o.prob_given_state(s), 
		Belief.make_uniform(states), 
		title = "Robot Belief", 
		subplot_num=211)


	human_bf = BayesFilter(states, 
		lambda s, s_p: s.transition_function(s_p), 
		lambda s, o: o.prob_given_state(s), 
		Belief.make_uniform(states), 
		title = "Human Belief", 
		subplot_num=212)


	xlist = [s.obj.location.x for s in states]
	ylist = [s.obj.location.y for s in states]
	zlist = [s.obj.location.z for s in states]
	interval = 0.10

	actions = []
	for x in irange(min(xlist), max(xlist), interval):
		for y in irange(min(ylist), max(ylist), interval):
			for z in irange(min(zlist), max(zlist), interval):
				try:
					actions.append(Point_Gesture(start_pose, gu.tup_to_point((x, y, z)), limb))
				except Exception:
					print "failed to make gesture" 
					print x, y, z
					


	print len(actions)

	observations = [Point_Gesture(start_pose, p, limb) for p in points]


	def observation_generator(): 
		last = 0
		while True: 
			p = raw_input()
			if p == '': 
				p = last
			else : 
				p = int(p)
			yield observations[p]


	interaction_loop(robot_bf, human_bf, actions, observation_generator(), kl_divergence_heuristic)


main()
