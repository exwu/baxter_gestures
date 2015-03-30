#!/usr/bin/python

from __future__ import division
import scipy.stats
import math
import matplotlib.pyplot as plt
import numpy as np


class Object: 
	def __init__(self, name, location): 
		self.name = name
		self.location = location

	def __repr__(self): 
		return "Obj(" + self.name + " : " + str(self.location) +  ")"

class Human_State: 
	def __init__(self, target, robot_distr): 
		# target is an Object, 
		# robot_distr is a belief over robot state (simplfied or not)
		self.target = target
		self.robot_distr = robot_distr

class One_Obj_State: 
	def __init__(self, target): 
		# target is an object
		self.target = target
	
	def transition_function(self, s_next, object_switch_prob=0.1): 
		# output the probability of transitioning from 
		# this function to the next
		if self.target == s_next.target:
			return 1-object_switch_prob
		else: 
			return object_switch_prob
	
	def __repr__(self): 
		return str(self.target)
	


class Belief: 
	def __init__(self, states, values): 
		self.states = states
		self.dictionary = dict(zip(states, values))
	
	def in_s(self, state): 
		return self.dictionary[state]

	def probs(self): 
		return [self.dictionary[state] for state in self.states]


	@staticmethod
	def make_uniform(states): 
		return Belief(states, [1/len(states) for s in states])

	def __repr__(self): 
		s = ""
		for key, value in self.dictionary.iteritems(): 
			s += str(key)
			s += " : " 
			s += str(value)
			s += "\n"
		return s


		


class Gesture_XY: 
	def __init__(self, location): 
		self.location = location

	def observation_function(self, state, covariance=[[1, 0], [0, 1]]): 
		# the probability of seeing this gesture given a state
		# state is a robot state, since the robot observes human gestures
		mean = state.target.location
		cov = covariance
		mvn = scipy.stats.multivariate_normal(mean, cov)

		return mvn.pdf(self.location)

	def __repr__(self): 
		return "Gesture_XY(" + str(self.location) + ")"

class Gesture_None: 
	# do nothing
	def observation_function(self, state): 
		return 1

	def __repr__(self): 
		return "Gesture_None (Nothing)"



class BayesFilter: 
	def __init__(self, states, transition_f, observation_f, initial_belief, title="Bayes Filter", subplot_num=None): 
		self.states = states
		self.transition_f = transition_f
		self.observation_f = observation_f
		self.belief = initial_belief
		self.title = title
		self.subplot_num = subplot_num

	def update(self, observation=None): 
		self.belief = self.advance(self.belief, observation)

	def advance(self, belief, observation=None): 
		vals = []
		for state in self.states: 
			prob = sum(map(lambda x_p: self.transition_f(x_p, state) * belief.in_s(x_p), 
				self.states))
			if observation != None: 
				prob *= self.observation_f(state, observation)
			vals.append(prob)
		vals = map(lambda s: s/sum(vals), vals)
		return Belief(self.states, vals)

	def __repr__(self): 
		return self.title + "\n" + str(self.belief)

	def plot(self): 
		if self.subplot_num == None: 
			print "No subplot provided"
			return

		ax = plt.subplot(self.subplot_num)
		ax.clear()

		ind = np.arange(len(self.states))
		bar_width = 0.35
		ax.set_ylabel('Probability')
		ax.set_title(self.title)
		ax.set_xticks(ind + bar_width/2)
		ax.set_xticklabels(self.states)
		ax.set_ylim([0, 1.0])

		bars = ax.bar(ind, self.belief.probs(), bar_width)
		for bar in bars: 
			height = bar.get_height()
			ax.text(bar.get_x() + bar.get_width()/2, height + 0.03, 
					'%.3f'%height, ha='center', va='bottom')


		plt.draw()



def binned_distributions(states, resolution=0.2): 
	# produces all possible multinomial distributions over
	# the states
	def make_distribution(add_to): 
		add_to = round(add_to, 4)
		if add_to <= 0: 
			s = set()
			s.add(tuple([0]*len(states)))
			return s
		distrs = make_distribution(add_to - resolution)
		total = set()
		for distr in distrs: 
			for i in range(len(states)): 
				d = list(distr)
				d[i] = round(d[i] + min(resolution, add_to), 4)
				total.add(tuple(d))
		return total
	return make_distribution(1.0)

def kl_divergence(P, Q): 
	# we want the minimum information lost when we use the robot's belief
	# to approximate the human's belief
	# KL divergence : the information lost when using Q to approximate P
	# P and Q are both distriubutions/belief
	return sum([p * math.log(p/q, 2) for p, q in zip(P.probs(), Q.probs())])

def interaction_loop(robot_bf, human_bf, actions, observation_generator, difference_metric=kl_divergence): 
	plt.ion()
	plt.show()
	fig, ax = plt.subplots()
	while True: 
		# Robot observes and makes its update
		robot_bf.update(next(observation_generator))

		# Decide which action is best
		best_action = min(actions, key=lambda a: difference_metric(human_bf.advance(human_bf.belief, a), robot_bf.belief))

		print best_action

		# update our estimate on the human's action
		human_bf.update(best_action)

		print human_bf
		print robot_bf
		robot_bf.plot()
		human_bf.plot()
		raw_input()

objects = [ 
		Object("origin", (0, 0)), 
		Object("up", (0, 1)), 
		Object("down", (0, -1)), 
		Object("left", (-1, 0)), 
		Object("right", (1, 0)) 
		]



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
		lambda s, s_next: s.transition_function(s_next), 
		lambda s, o: o.observation_function(s), 
		Belief.make_uniform(robot_states), 
		"Human Bayes Filter",
		212)


def irange(low, high, interval): 
	assert low < high
	nums = (high - low)/interval
	assert nums % 1 == 0
	return [ round(low + (x * interval), 4) for x in range(int(nums) + 1) ] 

actions = [Gesture_XY((x, y)) for x in irange(-1, 1, .2) for y in irange(-1, 1, .2)] + [Gesture_None()]

def hardcoded_observations(): 
	while True: 
		yield Gesture_XY((2, 0))
		#yield None


interaction_loop(Robot_BF, Human_BF, actions, hardcoded_observations())





