import gesture_utils as gu
from geometry_msgs.msg import Point
import math


def tests(): 
	origin = Point(0, 0, 0)
	p1 = Point(1, 0, 0)
	p2 = Point(1, 0, 1)
	print 180 / math.pi * gu.angle_between(origin, p1, p2) # should be 45, and is. 

	origin = Point(0, 0, 0)
	p1 = Point(1, 0, 0)
	p2 = Point(0, 0, 1)
	print 90 == 180 / math.pi * gu.angle_between(origin, p1, p2)


	origin = Point(0, 0, 0)
	p1 = Point(1, 0, 0)
	p2 = Point(2, 0, 0 )
	print 0 == 180 / math.pi * gu.angle_between(origin, p1, p2)


	origin = Point(1, 0, 0)
	p1 = Point(1, 1, 0)
	p2 = Point(2, 0, 0 )
	print 90 == 180 / math.pi * gu.angle_between(origin, p1, p2)


tests()
