#!/usr/bin/python
import matplotlib.pyplot as plt
import random
from interaction import Belief
from interaction import binned_distributions
import numpy as np
import json

class Scenario: 
	def __init__(self,  robot_belief, action=None):
		self.belief = robot_belief
		self.action = action
	def __repr__(self): 
		return "belief: " + str(self.belief) + ", action: " + str(self.action)

class Visualizer: 
	def __init__(self, scenarios): 
		self.scenarios = scenarios

		plt.ion()
		plt.show()
		self.fig = plt.figure()
		cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)
	
		self.annotated = set()

	def draw_points(self, point=None): 
		ax = plt.subplot(211)
		ax.clear()
		# static points
		points = self.current.belief.states
		probs = self.current.belief.probs
		xs = [p[0] for p in points]
		ys = [p[1] for p in points]
		colors = ['blue' for p in points]
		size = [prob * 30 + 10 for prob in probs]
		# new point
		if point: 
			xs.append( point[0])
			ys.append( point[1])
			colors.append('red')
			size.append(20)

		ax.scatter(xs, ys, c=colors, s=size)
		plt.draw()

	def draw_distribution(self): 
		ax = plt.subplot(212)

		ax.clear()
		belief = self.current.belief

		ind = np.arange(len(belief.states))

		bar_width = 0.35
		ax.set_ylabel('Probability')
		ax.set_title("Probs")
		ax.set_xticks(ind + bar_width/2)
		ax.set_xticklabels(belief.states)
		ax.set_ylim([0, 1.0])

		bars = ax.bar(ind, belief.probs, bar_width)
		for bar in bars: 
			height = bar.get_height()
			ax.text(bar.get_x() + bar.get_width()/2, height + 0.03, 
					'%.3f'%height, ha='center', va='bottom')


	def onclick(self, event):
		print 'button=%d, x=%d, y=%d, xdata=%f, ydata=%f'%(	event.button, event.x, event.y, event.xdata, event.ydata)
		if event.xdata != None: 
			self.current.action = (event.xdata, event.ydata)
			self.draw_points( (event.xdata, event.ydata) )


	def run(self): 
		for scenario in self.scenarios:
			self.current = scenario
			self.annotated.add(self.current)
			self.draw_points(scenario.action)
			self.draw_distribution()
			inp = raw_input()
			while True:
				if inp == 'n':
					break
				elif inp == 'q':
					return
				else: 
					inp = raw_input()


	def get_annotated(self): 
		return self.annotated


def read_points(filename): 
	with open(filename) as f: 
		lines = f.readlines()
	scenarios = []
	for line in lines: 
		bits = [x.strip() for x in line.split(',')]
		points = []
		i = 0
		while i < len(bits):
			if bits[i] != '':
				points.append((int(bits[i]), int(bits[i+1])))
				i+=2
			else: 
				break
		scenarios.append(points)
	return scenarios

def write_scenarios(filename, scenarios): 
	f = open(filename, 'w')
	json_scenarios = []
	for s in scenarios: 
		points = s.belief.states
		probs = s.belief.probs
		action = s.action
		json_s = {}
		json_s['points'] = points
		json_s['probs'] = probs
		json_s['action'] = action
		json_scenarios.append(json_s)

	json.dump(json_scenarios, f)





def main(): 
	points_list = read_points('points.csv')
	scenarios = []
	annotated = []
	for points in points_list: 
		distrs = binned_distributions(points, 0.333)
		distrs = [dist for dist in distrs if not 0.999 in dist]
		distrs = [dist for dist in distrs if not 0.666 in dist]
		distrs = [dist for dist in distrs if not 0.001 in dist]

		for distr in distrs: 
			scenario = Scenario(Belief(points, distr), None)
			if 1.0 in distr: 
				scenario.action = points[distr.index(1.0)]
				annotated.append(scenario)
			else: 
				scenarios.append(scenario)
	
	v = Visualizer(scenarios)
	v.run()
	print len(annotated)
	annotated += v.get_annotated()
	write_scenarios('scenarios.csv', annotated)

	

	


main()



