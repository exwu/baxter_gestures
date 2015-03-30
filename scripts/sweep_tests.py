#!/usr/bin/python
from gesture_utils import * 


bowl_pos5 = Point(x=0.7403782842323192, y=-0.5016241130334161, z=-0.028004569892429124) 

points = [ 
         Point(x=0.6727, y=-0.7721, z=-0.0462), 
         Point(x=0.6794, y=-0.5903, z=-0.0466), 
         Point(x=0.5303, y=-0.4200, z=-0.0028), 
         Point(x=0.7631, y=-0.4258, z=-0.0616),
         Point(x=0.7772, y=-0.146, z=-0.063)
        ]

init(["right"]);

def test_naive(): 
    baxter_sweep_naive(points)

def test_less_naive(): 
    baxter_sweep_less_naive(points)

def test_point_down(): 
    baxter_point_downwards(points[0])

def test_best_fit_line(): 
    print best_fit_line(points)

def test_sweep_line(): 
    baxter_sweep_line(points)
    

def main(): 
    tests = {
            'naive' : test_naive, 
            'less_naive' : test_less_naive, 
            'point_down' : test_point_down,
            'bfl' : test_best_fit_line, 
            'line' : test_sweep_line
            }
    if sys.argv[1] in tests: 
        tests[sys.argv[1]]()
    else: 
        print "invalid key"
        print tests.keys()

main()
