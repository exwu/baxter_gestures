#!/usr/bin/python

from interaction import * 

class Object: 
	def __init__(self, name, location): 
		self.name = name
		self.location = location

	def __repr__(self): 
		return "Obj(" + self.name + " : " + str(self.location) +  ")"


class Human_State(State):
	def __init__(self, target, robot_distr): 
		# target is an Object, 
		# robot_distr is a belief over robot state (simplfied or not)
		self.target = target
		self.robot_distr = robot_distr


class Gesture_XY(Observation): 
	def __init__(self, location): 
		self.location = location

	def observation_function(self, state, covariance=[[1, 0], [0, 1]]): 
		# the probability of seeing this gesture given a state
		# state is a robot state, since the robot observes human gestures
		if state.target.location == None: 
			return state.target.observation_function(self)

		mean = state.target.location
		cov = covariance
		mvn = scipy.stats.multivariate_normal(mean, cov)

		return mvn.pdf(self.location)

	def __repr__(self): 
		return "Gesture_XY(" + str(self.location) + ")"


objects = [ 
		Object("origin", (0, 0)), 
		Object("up", (0, 1)), 
		Object("down", (0, -1)), 
		Object("left", (-1, 0)), 
		Object("right", (1, 0)) 
		]

nil_obj = NilObj(list(objects))

objects.append(nil_obj)



#robot_states = [Robot_State(Belief(objects, h_distr)) for h_distr in binned_distributions(objects)]

# Make the ROBOT BayesFILTER
robot_states = [One_Obj_State(obj) for obj in objects]

Robot_BF = BayesFilter(robot_states, 
		lambda s, s_next: s.transition_function(s_next), 
		lambda s, o: o.observation_function(s), 
		Belief.make_uniform(robot_states),
		"Robot Bayes Filter", 
		211)

# Make the Human BayesFilter
#human_states = [Human_State(obj, Belief(robot_states_s, r_distr)) for obj in objects for r_distr in binned_distributions(robot_states_s)]

human_states = [One_Obj_State(obj) for obj in objects]
Human_BF = BayesFilter(robot_states, 
		lambda s, s_next: s.transition_function(s_next, object_switch_prob=0.5 ), 
		lambda s, o: o.observation_function(s), 
		Belief.make_uniform(robot_states), 
		"Human Bayes Filter",
		212)


actions = [Gesture_XY((x, y)) for x in irange(-2, 2, .4) for y in irange(-2, 2, .4)] + [Gesture_None()]

def hardcoded_observations(): 
	while True: 
		yield Gesture_XY((0.5, 0))
		#yield None

def raw_input_observations(): 
	inpt_dict = {obj.name: Gesture_XY(obj.location) for obj in objects}
	while True: 
		inp = raw_input()
		if inp in inpt_dict: 
			yield inpt_dict[inp]
		else: 
			yield None
		


interaction_loop(Robot_BF, 
				 Human_BF, 
				 actions, 
				 raw_input_observations(), 
				 #hardcoded_observations(), 
				 #entropy_heuristic)
				 kl_divergence_heuristic)
				 #kl_entropy_divergence_heuristic)





