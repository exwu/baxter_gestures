#!/usr/bin/python

from __future__ import division
import scipy.stats
import math
import matplotlib.pyplot as plt
import numpy as np
import rospy

THRESHOLD = 0.1


class State: 
	def transition_function(self, s_next): 
		raise NotImplementedError("no transistion function")

class Observation:
	# Observations are actions
	pass

class One_Obj_State(State):
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
	
class NilObj: 
	def __init__(self, others): 
		self.location = None
		self.name = "Nil"
		self.others = others

	def __repr__(self): 
		return "Nil"

	def observation_function(self, obs, threshold=0.25):
		observation_probs = [obs.observation_function(One_Obj_State(s)) for s in self.others]
		if sum(observation_probs) < threshold: 
			return 2*max(observation_probs)
		return min(observation_probs)

class Belief: 
	def __init__(self, states, values): 
		self.states = states
		self.dictionary = dict(zip(states, values))
		probs = [self.dictionary[state] for state in self.states]
		self.probs = [ prob/sum(probs) for prob in probs]
	
	def in_s(self, state): 
		return self.dictionary[state]


	@staticmethod
	def make_uniform(states): 
		return Belief(states, [1/len(states) for s in states])


	def entropy(self): 
		return sum([ -1*p*math.log(p, 2) for p in self.probs ])

	def __repr__(self): 
		s = ""
		for key, value in self.dictionary.iteritems(): 
			s += str(key)
			s += " : " 
			s += str(round(value, 3))
			s += "\n"
		return s


class Gesture_None(Observation): 
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

		bars = ax.bar(ind, self.belief.probs, bar_width)
		for bar in bars: 
			height = bar.get_height()
			ax.text(bar.get_x() + bar.get_width()/2, height + 0.03, 
					'%.3f'%height, ha='center', va='bottom')


		plt.draw()
		rospy.sleep(0.5)



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
	# KL divergence : the information lost when using Q to approximate P
	# P and Q are both distriubutions/belief
	return sum([p * math.log(p/q, 2) for p, q in zip(P.probs, Q.probs)])

def interaction_loop(robot_bf, human_bf, actions, observation_generator, heuristic): 
	# robot_bf : a Bayes_Filter describing the robot's belief about which object the human wants
	# human_bf : a Bayes_Filter describing the human's belief about which objects the robot wants
	# actions : a list of actions the robot can take
	# observation_generator : a generator which yields observations of the human's actions
	# heurisitic : (Robot Belief, Human Belief, action) => float, some sort of heuristic describing the
	#				desirability of an action
	plt.ion()
	plt.show()
	while True: 
		# Robot observes and makes its update
		obs = next(observation_generator)
		print "observation: " + str(obs)
		robot_bf.update(obs)

		# Decide which action is best
		best_action = min(actions, key=lambda a: heuristic(robot_bf, human_bf, a))

		print "chosen_action: " + str(best_action)
		print "current score:", kl_divergence(robot_bf.belief, human_bf.belief) 

		if True or kl_divergence(robot_bf.belief, human_bf.belief) > THRESHOLD: 
			print 'executing'
			best_action.execute()

		# update our estimate on the human's action
		human_bf.update(best_action)

		robot_bf.plot()
		human_bf.plot()

def irange(low, high, interval): 
	assert low <= high
	nums = round((high - low)/interval)
	return [ round(low + (x * interval), 4) for x in range(int(nums) + 1) ] 



# the kl divergence between the resulting human's belief and our belief
kl_divergence_heuristic = lambda rbf, hbf, a: kl_divergence(hbf.advance(hbf.belief, a), rbf.belief)
# how certain the human is. This is just telling them what they want, doesn't actually have anything to do with what the robot thinks...
entropy_heuristic = lambda rbf, hbf, a: hbf.advance(hbf.belief, a).entropy()

kl_entropy_divergence_heuristic = lambda rbf, hbf, a: kl_divergence_heuristic(rbf, hbf, a) + 5* entropy_heuristic(rbf, hbf, a)
