#!/usr/bin/python
from __future__ import division
from interaction import * 
import gesture_utils as gu
from gesture_utils import Point
import sys
import rospy
from gesture_rec.msg import BayesFilterStateDist
from object_recognition_msgs.msg import RecognizedObjectArray
from scipy.stats import multivariate_normal

class Object:
	def __init__(self, name, location): 
		# location is an xyz coordinate, represented as point
		self.name = name
		self.location = location

	def __repr__(self): 
		return "Obj(" + self.name + " : " + str(self.location) +  ")"

	def __eq__(self, other): 
		return self.name == other.name and self.location == other.location

	def __hash__(self): 
		return self.__repr__().__hash__()

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

	def __eq__(self, other): 
		return self.obj == other.obj

	def __hash__(self): 
		return self.__repr__().__hash__()
	
class Multi_Obj_State(State): 
	def __Init__(self, objects): 
		self.objects = objects
	
class Point_Gesture(Observation): 
	def __init__(self, start_pose, target, limb): 
		# target is a xyz location (point)
		self.target = target
		self.ee_pose, self.joint_angles = gu.baxter_point_pos(target, start_pose=start_pose, limb=limb)
		#self.ee_pose, self.joint_angles = gu.baxter_point_downwards_sol(target, limb=limb, height=0.15)
		self.limb = limb


	def execute(self): 
		gu.baxter_execute_joint_positions([self.joint_angles], limb=self.limb)
		gu.move_to_neutral(limb=self.limb)

	def prob_given_state(self, state, variance=0.05): 
                cov = [ [variance, 0], [0, variance]]
                mvn = multivariate_normal([self.target.x, self.target.y], cov)
                p = state.obj.location
                r = mvn.pdf([p.x, p.y])
		return r

	def __repr__(self): 
		return "Point at " + (str(self.target).replace('\n', ' '))



class Point_Emph(Observation): 
	def __init__(self, start_pose, target, limb):
		self.target = target
		self.limb = limb
                self.joints_close, self.joints_far = gu.baxter_point_emphatically_pos(target, start_pose, limb)

	def execute(self): 
		gu.baxter_point_emphatically_at_execute(self.joints_close, self.joints_far, self.limb)

	def prob_given_state(self, state, variance=0.05): 
                cov = [ [variance, 0], [0, variance]]
                mvn = multivariate_normal([self.target.x, self.target.y], cov)
		#r =  scipy.stats.norm(0.0, math.sqrt(variance)).pdf(sample)
                p = state.obj.location
                r = mvn.pdf([p.x, p.y])


		return r

	def __repr__(self): 
		return "Point emph at " + (str(self.target).replace('\n', ' '))

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
	def __init__(self, states, objects_map, state_maker, title="Bayes Filter", subplot_num=None) : 
		rospy.Subscriber('mm_bf_state', BayesFilterStateDist, self.dist_callback, queue_size=10)
		self.belief = Belief.make_uniform(states)
		self.subplot_num = subplot_num
		self.title = title
		self.states = states
		self.state_maker = state_maker
		self.objects_map = objects_map

	def dist_callback(self,data): 
		#states = [ self.state_maker(name) for name in data.names if name in self.objects_map ] 
		#print 'callback', data.names
		states = []
		probs = []
		for i, name in enumerate(data.names): 
			if name in self.objects_map:
				states.append(self.state_maker(name))
				probs.append(data.probs[i])

		self.belief = Belief(states, probs)
		self.states = states

	def update(self, observation=None): 
		return self.belief

	def advance(self, belief, observation=None): 
		raise NotImplementedError("Advance is not available for this")


#def interaction_loop(robot_bf, human_bf, actions, observation_generator, heuristic): 
		
points = [ 
		gu.Point(x=0.6727, y=0.7721, z=-0.0462), #{{{
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
		 ]#}}}


class ObjectsSubscriber(): 
	# does not support objects moving, for now. 
	# just on the left hand for now

	def __init__(self): 
		rospy.Subscriber('ein_right/blue_memory_objects', RecognizedObjectArray, self.callback, queue_size = 10)
		self.objects = {}

	def callback(self, data): 
		self.objects = {}
		for o in data.objects:
			pos = o.pose.pose.pose.position  # point
			name = o.type.key
			self.objects[name] = Object(name, pos)


	def getObjects(self): 
		# return a map from 'name' to 'object'
		# something else should make it to states
		while not self.objects: 
			rospy.sleep(2.)
			print "waiting for objects"
			pass
		return self.objects


def test(): 
	rospy.init_node('thing')
	o = ObjectsSubscriber()
	while not rospy.is_shutdown(): 
		print o.getObjects()
		rospy.Rate(20).sleep()



def main(): 

	limb = 'right'
	gu.init([limb])
	start_joints, start_pose = gu.neutral_pose(limb=limb)
	o = ObjectsSubscriber()
	objects_map = o.getObjects()


	states = [One_Obj_State(obj) for obj in objects_map.values() ]
	print states

	state_maker = lambda name: One_Obj_State(objects_map[name])

	meldon = MeldonBayesFilterWrapper(states, objects_map, state_maker, subplot_num=211)

	#states = [One_Obj_State(Object("obj " + str(p.x), p)) for p in points]

	#robot_bf = BayesFilter(states, 
	#	lambda s, s_p: s.transition_function(s_p), 
	#	lambda s, o: o.prob_given_state(s), 
	#	Belief.make_uniform(states), 
	#	title = "Robot Belief", 
	#	subplot_num=211)


	human_bf = BayesFilter(states, 
		lambda s, s_p: s.transition_function(s_p), 
		lambda s, o: o.prob_given_state(s), 
		Belief.make_uniform(states), 
		title = "Human Belief", 
		subplot_num=212)


	xlist = [s.obj.location.x for s in states]
	ylist = [s.obj.location.y for s in states]
	#zlist = [s.obj.location.z for s in states]
	interval = 0.05

	actions = []
	for x in irange(min(xlist)-0.10, max(xlist)+0.1, interval):
		for y in irange(min(ylist)-0.10, max(ylist)+0.1, interval):
	#		for z in irange(min(zlist)-0.10, max(zlist)+0.1, interval):
				try:
					actions.append(Point_Gesture(start_pose, gu.tup_to_point((x, y, 0)), limb))
				except Exception:
					print "failed to make gesture" 
					print x, y, 0
					
        actions2 = [Point_Gesture(start_pose, obj.location, limb) for obj in objects_map.values() ]

        actions.extend(actions2)
        #actions = actions2


	print actions
        raw_input()

	#observations = [Point_Gesture(start_pose, p, limb) for p in points]


	def input_observation_generator(): 
		last = 0
		while True: 
			p = raw_input()
			if p == '': 
				p = last
			else : 
				p = int(p)
			yield observations[p]

	def none_generator(): 
		while True: 
			yield None 


	interaction_loop(meldon, human_bf, actions, none_generator(), kl_divergence_heuristic)

if __name__ == "__main__":
	main()
