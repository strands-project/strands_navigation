#!/usr/bin/env python

from svgwrite import Drawing
import rospy
from yaml import loads


class Map2Svg:

	def __init__(self):
		self.map = None
		self.drawing = None

	def load_file(self, filename):
		with open(filename) as file:  
			data = file.read() 
			self.map = loads(data)

	def _create_svg(self):
		if self.map:
			self.drawing = Drawing('topological.svg')

		for n in self.map:
			node = n['node']
			name = n['name']
			
			self.drawing.circle(center=())

	def write_svg(self):



dwg = svgwrite.Drawing('test.svg')
dwg.add(dwg.line((30, 30), (10, 30), stroke=svgwrite.rgb(10, 10, 16, '%')))
dwg.add(dwg.text('Test', insert=(10, 0.2), fill='red'))
dwg.save()
