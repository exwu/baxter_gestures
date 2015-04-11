#!/usr/bin/python
from interaction import * 
import gesture_utils as gu
import sys

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
	
class Point(Observation): 
	def __init__(self, target, limb): 
		# target is a xyz location (point)
		self.target = target
		self.ee_pose, self.joint_angles = gu.baxter_point_pos(target, limb)
		self.limb = limb

	def execute(self): 
		gu.baxter_execute_joint_positions(self.joint_angles)

	def prob_given_state(self, state, variance=0.4): 
		origin = gu.baxter_w1_position(self.ee_pose, self.limb)
		sample = gu.angle_between(origin, self.ee_pose['position'], self.target)
		return scipy.stats.norm(0.0, math.sqrt(variance)).pdf(sample)

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


class MeldonBayesFilterWrapper(BayesFilter): 
	def __init__(self, title="Bayes Filter", subplot_num=None): 
		pass

	def update(self, belief, observation=None): 
		# TODO
		# get belief from miles
		# set own belief
		pass

	def advance(self, belief, observation=None): 
		raise NotImplementedError("Advance is not available for this")


#def interaction_loop(robot_bf, human_bf, actions, observation_generator, heuristic): 
		
points = [ 
		gu.Point(x=0.6727, y=-0.7721, z=-0.0462), 
		gu.Point(x=0.6794, y=-0.5903, z=-0.0466), 
		gu.Point(x=0.5303, y=-0.4200, z=-0.0028), 
		gu.Point(x=0.7631, y=-0.4258, z=-0.0616),
		gu.Point(x=0.7772, y=-0.146, z=-0.063)
		]

#def interaction_loop(robot_bf, human_bf, actions, observation_generator, heuristic): 
#	def __init__(self, states, transition_f, observation_f, initial_belief, title="Bayes Filter", subplot_num=None): 
def main(): 
	#robot_bf = MeldonBayesFilterWrapper(title="Robot Belief", subplot_num=211) 


	gu.init(['right'])
	limb = 'right'


	states = [One_Obj_State(Object("obj " + str(p.x), p)) for p in points]

	robot_bf = BayesFilter(states, 
		lambda s, s_p: s.transition_function(s_p), 
		lambda s, o: o.prob_given_state(s), 
		Belief.make_uniform(states), 
		title = "Robot Belief", 
		subplot_num=212)


	human_bf = BayesFilter(states, 
		lambda s, s_p: s.transition_function(s_p), 
		lambda s, o: o.prob_given_state(s), 
		Belief.make_uniform(states), 
		title = "Human Belief", 
		subplot_num=212)

	xlist = [s.obj.location.x for s in states]
	ylist = [s.obj.location.y for s in states]
	zlist = [s.obj.location.z for s in states]
	interval = 0.05

	actions = [Point(gu.tup_to_point(p), limb) for p in zip(
		irange(min(xlist), max(xlist), interval), 
		irange(min(ylist), max(ylist), interval), 
		irange(min(zlist), max(zlist), interval)) ]

	def observation_generator(): 
		while True: 
			yield Do_Wait()

	interaction_loop(robot_bf, human_bf, actions, observation_generator(), kl_divergence_heuristic)


main()
