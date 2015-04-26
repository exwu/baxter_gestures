#!/usr/bin/python
from __future__ import division
from interaction import Belief
from interaction import One_Obj_State
from interaction import irange
from interaction import NilObj
from itertools import tee
from itertools import islice
import interaction
import json
import scipy
import random
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import math

domain_min = -1.0
domain_max = 1.0

class Scenario: 
	def __init__(self,  robot_belief, action=None):
		self.belief = robot_belief
		self.action = action
	def __repr__(self): 
		return "belief: " + str(self.belief) + ", action: " + str(self.action)

class PointAction: 
	def __init__(self, target): 
		# target is a single point (tuple)
		self.target = target
	
	def observation_function(self, state): 
		mean = state.target
		cov = [[0.2, 0], [0, 0.2]]
		mvn = scipy.stats.multivariate_normal(mean, cov)
		return mvn.pdf(self.target)

	def __eq__(self, other):
		if other == None: return False
		return self.target == other.target

	def __hash__(self): 
		return hash(self.target)

	def __repr__(self): 
		return "Point " + str(self.target)

class SweepAction: 
	def __init__(self, targets):
		# targets is  a list of tuples
		self.targets = targets
	
	def observation_function(self, state):  
		return sum([ PointAction(tup).observation_function(state) for tup in self.targets])

	def __eq__(self, other):
		return self.targets == other.targets

	def __hash__(self): 
		return hash(tuple(self.targets))

	def __repr__(self): 
		return "Sweep " + str(self.targets)

def read_scenario_file(filename): 
	ss = json.load(open(filename))
	# scenarios are in 0 to 10 in x to y 
	scenario_min = 0 
	scenario_max = 10
	def convert_to_domain_scale(number): 
		scale = (domain_max - domain_min)/(scenario_max - scenario_min)
		return round(domain_min + scale * (number - scenario_min), 3)
	def ctds_tup(tup): 
		return map(convert_to_domain_scale, tup)
	
	scenarios = []
	for s in ss:
		points = map(lambda s: tuple(ctds_tup(s)), s['points'])
		probs = map(lambda p: p if p > 0 else 0.0000000001, s['probs'])
		action = PointAction(tuple(ctds_tup(s['action']))) if s['action'] else None
		scenarios.append(Scenario(Belief(points, probs), action))
	return scenarios

def nil_fun_maker(states): 
	threshold = 0.4
	def nil_obs_fun(obs): 
		def distance(p1, p2): 
			return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2) ** 0.5
		distances = [ distance(state.target, obs.target) for state in states if state.target != None]
		probs = [obs.observation_function(state) for state in states if state.target != None]
		far_away = len([ distance for distance in distances if distance > 0.4])
		return max(probs)*10 if sum(probs) < threshold else 0.000001
		#return (far_away/len(distances)) ** 2 * 2 * max(probs) + 0.00001

		#observation_probs = [tuple_observation_f(s, obs) for s in states]
		#if sum(observation_probs) < threshold: 
		#	return 2*max(observation_probs)
		#return min(observation_probs)
	return nil_obs_fun



def make_bayes_filter(scenario, use_uniform): 
	states = [One_Obj_State(point) for point in scenario.belief.states]
	non_obj = One_Obj_State(None)
	states.append(non_obj)
	nil_fun = nil_fun_maker(states)
	def nil_observation_f(s, o): 
		if s.target == None: 
			return nil_fun(o) 
		else: 
			return o.observation_function(s)
	def observation_f(s, o): 
		return o.observation_function(s)

	transition_f = lambda s, s_n: s.transition_function(s_n, object_switch_prob=0.1)
	new_belief = Belief(states, scenario.belief.probs + [0.000001])
	initial_belief = Belief.make_uniform(states) if use_uniform else new_belief
	return interaction.BayesFilter(states, transition_f, nil_observation_f, 
					initial_belief)

def distance(t1, t2): 
	return ((t1[0] - t2[0])**2 + (t1[1] - t2[1])**2)**0.5

def test_scenario(scenario, actions, heuristic): 
	robot_bf = make_bayes_filter(scenario, False)
	human_bf = make_bayes_filter(scenario, True)
	vals = { a : heuristic(robot_bf, human_bf, a) for a in actions}
	best_action = min(actions, key=lambda a: vals[a])
	annotated_action = scenario.action
	v = None if annotated_action == None else heuristic(robot_bf, human_bf, annotated_action)
	if False: 
		print "================================"
		print "SCENARIO\n", scenario
		print ""
		print "RESULT ACTION\n", best_action
		print "val", vals[best_action]
		print "bayes filter:\n", human_bf.advance(human_bf.belief, best_action)
		print "GIVEN ACTION\n", annotated_action
		print "val", v
		print "bayes filter:\n", human_bf.advance(human_bf.belief, annotated_action)

	dist = None if annotated_action == None else distance(annotated_action.target, best_action.target)
	difference = None if annotated_action == None else v# - vals[best_action]
	return (dist, difference)


def action_generators(max_length): 
	actions = [ (x, y) for x in irange(domain_min, domain_max, 0.2) for y in irange(domain_min, domain_max, 0.2) ]
	generator_list = []
	def actions_list(x): 
		if x == 1: 
			gen = ( SweepAction([tup]) for tup in actions )
			gen, gen_t = tee(gen)
			generator_list.append(gen_t)
			return gen
		else: 
			p = actions_list(x-1)
			def gen():
				for f in ( SweepAction([tup] + prev.targets) for prev in p for tup in actions):
					yield f
				#for pr in p:
				#	yield pr
			gen, gen_t = tee(gen())
			generator_list.append( gen_t ) 
			return gen
	actions_list(max_length)
	return generator_list


def sample_uniformly(sample_size, generator): 
	sample = list(islice(generator, sample_size))
	for i, v in enumerate(generator, sample_size):
		r = random.randint(0, i)
		if r < sample_size: 
			sample[r] = v 
	return sample


def entropy(probs): 
	return sum([ -1 * p * math.log(p, 2) for p in probs])

def main(): 
	#scenarios = read_scenario_file('scenarios.csv')
	scenarios = [example1(), example2(), example3()]  + read_scenario_file('scenarios.csv')
	random.shuffle(scenarios)
	scenarios = scenarios[:50]
	heuristics = [ ('kl divergence', interaction.kl_divergence_heuristic),
				  ('entropy + kl', interaction.kl_entropy_divergence_heuristic), 
				  ('entropy', interaction.entropy_heuristic), 
				  ('random', lambda rbf, hbf, a: random.random())]
	actions = [ (x, y) for x in irange(-1.4, 1.4, 0.2) for y in irange(-1.4, 1.4, 0.2) ]
	point_actions = [ PointAction(tup) for tup in actions ]
	print 'actions done'
	print 'number of actions', len(point_actions)
	
	#generators = action_generators(3)
	#num = 200
	#sweep_actions = sample_uniformly(num, generators[0]) + sample_uniformly(num, generators[1]) + sample_uniformly(num, generators[2])

	read_scenario_file('scenarios.csv')

	names = ['kl divergence', 'entropy + kl', 'entropy', 'random']
	diffs = []
	dists = []
	show = []
	colors = []
	sizes = [] #according to entropy
	colormap = {names[0]:'r',names[1]:'g', names[2]:'b', names[3]:'k'}

	dists_h = {names[0]:[], names[1]:[], names[2]:[], names[3]:[]}
	diffs_h = {names[0]:[], names[1]:[], names[2]:[], names[3]:[]}



	for s in scenarios:
		for name, heuristic in heuristics: 
			#print name
			dist, diff = test_scenario(s, sample_uniformly(100,iter(point_actions)), heuristic)
			if dist != None and diff != None: 
				show.append(s)
				dists.append(dist)
				diffs.append(diff)
				dists_h[name].append(dist)
				diffs_h[name].append(diff)
				colors.append(colormap[name])
				sizes.append(entropy(s.belief.probs)*60+10)

	#print diffs
	#print dists

	m = max(diffs)
	diffs = map(lambda a: a/m, diffs)

	print 'avg difference', sum(diffs)/len(diffs)
	print 'avg distance', sum(dists)/len(dists)

	for name in names:
		print name, ' avg difference', sum(diffs_h[name])/len(diffs_h[name])
		print name, ' avg distance', sum(dists_h[name])/len(dists_h[name])
	

	plt.scatter(diffs, dists,s=sizes, c=colors,marker='o', alpha=0.4)
	plt.xlabel('score of annotated action')
	plt.ylabel('distance between annontated action and chosen action')

	blue = mlines.Line2D([], [], color='blue', marker='o', markersize=10)
	red = mlines.Line2D([], [], color='red', marker='o', markersize=10)
	green = mlines.Line2D([], [], color='green', marker='o', markersize=10)
	black = mlines.Line2D([], [], color='black', marker='o', markersize=10)
	small = mlines.Line2D([], [], color='black', marker='o', markersize=5)
	large = mlines.Line2D([], [], color='black', marker='o', markersize=10)

	plt.legend([red, green, blue, black, small, large], ['kl divergence', 'kl divergence + entropy', 'entropy', 'scenario entropy low', 'scenario entropy = high'])
	#for scenario, diff, dist in zip(show, diffs, dists): 
	#	plt.annotate(
	#			str(scenario.belief), 
	#			xy=(diff, dist), xytext=(-10, 10),
	#			textcoords = 'offset points', ha = 'right', va = 'bottom',
	#			arrowprops = dict(arrowstyle = '->', connectionstyle = 'arc3,rad=0'))

	plt.show()

	

def example1(): 
	# one more likely than the other
	#points = [ (-0.5, -0.5), (0.5, 0.5), None]
	points = [ (-0.6, -0.6), (0.6, 0.6)]
	probs = [ 0.00001, 1.0]
	action = PointAction((0.6, 0.6))
	#action = SweepAction([(0.6, 0.6)])
	return Scenario(Belief(points, probs), action)

def example2(): 
	# one far away, unlikely, two close together, likely
	#points = [ (-0.5, -0.5), (0.3, 0.5), (0.5, 0.3), None]
	points = [ (-0.6, -0.6), (0.2, 0.4), (0.2, 0.4)]
	probs = [0.10, 0.45, 0.45]
	action = PointAction((0.4, 0.4))
	#action = SweepAction([(0.4, 0.4)])
	return Scenario(Belief(points, probs), action)


def example3(): 
	# one more likely than the other
	#points = [ (-0.5, -0.5), (0.5, 0.5), None]
	points = [ (-0.6, -0.6), (0.6, 0.6)]
	probs = [ 0.20, 0.8]
	action = PointAction((0.5, 0.6))
	#action = SweepAction([(0.6, 0.6)])
	return Scenario(Belief(points, probs), action)



main()

