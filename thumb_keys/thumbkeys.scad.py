from __future__ import division


corners = [
    [0,0],
    [20.83, 0],
    [55.97, 11.71],
    [53.41, 18.45],
    [69.63,36.61],
    [80, 41.05],
    [80, 85.87],
    [46.80, 85.87],
    [0, 68.02],
]

# centers
holes = [
    [11.27, 6.39],
    [74.17, 45.73],
    [48.25, 79.90],
    [7.07, 64.20],
]

#center, angle
key_angle = -70
keys = [
    [35.55, 15.91],
    [29.08, 33.86], 
    [13.48, 48.63],
    [31.64, 55.30],
    [49.82, 61.97], 
    [56.34, 43.92], 
]

from solid import *
import solid
from solid.utils import *
import math
import math
import os

openscad_path = os.path.abspath(__file__)
openscad_path = openscad_path[:openscad_path.lower().find('sync') + 4]
openscad_path = os.path.join(openscad_path, 'hobby','reprap','models', 'openscad')
sys.path.append(openscad_path)
import openscad_ff
from openscad_ff import *
import collections

use(os.path.join(openscad_path, "triangles.scad"))
use(os.path.join(openscad_path, "roundCornersCube.scad"))
use(os.path.join(openscad_path, "nuts_and_bolts.scad"))

def tr(t, x,y):
    return t[0] +x, t[1] + y

def model():
    height = 5
    body =  union()
    for h in holes:
        c = cy(r=4/2+2, h=height)
        c = c.translate(h[0], h[1],height/2 )
        body += c

    body += linear_extrude(5) (
        polygon([
            holes[0],
            tr(holes[0], -0, 3),
            tr(holes[1], 0, 3),
            holes[1]
        ]).translate(0,-1.5,0)
    )

    body += linear_extrude(5) (
        polygon([
            holes[1],
            tr(holes[1], -0, 3),
            tr(holes[2], 0, 3),
            holes[2]
        ]).translate(0,-1.5,0)
    ) 

    body += linear_extrude(5) (
        polygon([
            holes[2],
            tr(holes[2], -0, 3),
            tr(holes[3], 0, 3),
            holes[3]
        ]).translate(0,-1.5,0)
    ) 

    body += linear_extrude(5) (
        polygon([
            holes[3],
            tr(holes[3], -3, 3),
            tr(holes[0], -3, 3),
            holes[0]
        ]).translate(1.5,0,0)
    ) 

    body += cube([40, 3.5, 8]).rotate(0,0,20).translate(15, 54,0)
    body += cube([40, 3.5, 8]).rotate(0,0,20).translate(15, 40.75,0)
    body += cube([30, 3.5, 8]).rotate(0,0,20).translate(30, 25.75+13.25,0)
    body += cube([30, 3.5, 8]).rotate(0,0,20).translate(30, 25.75,0)

    body += cube([3.5, 30, 8]).rotate(0,0,20).translate(27, 15.75,0)
    body += cube([3.5, 30, 8]).rotate(0,0,20).translate(62.5, 40.75,0)


    e = union()
    for h in holes:
        c = cy(r=4.2/2, h=100)
        c = c.translate(h[0], h[1],0 )
        e += c.hole()
    
    k = union()
    for h in keys:
        #c = cy(r=3.7/2, h=100)
        c = cube([13, 12, height * 2])
        c = c.translate(-6.5,-6, -1)
        c = c.rotate(0,0,-70)
        c = c.translate(h[0], h[1], 0)
        k += c.hole().d()
        c = cube([14.3, 14.1, height * 3])
        c = c.translate(-7,-7, 0)
        c = c.rotate(0,0,-70)
        c = c.translate(h[0], h[1], height)
        k += c.hole()

        w = 16
        c = cube([w, w, 8])
        l = 8
        #c -= cube([l, w+2, 5-1.6]).translate(w/2-l/2,-1,3).hole()
        c = c.translate(-w/2,-w/2, 0)
        c = c.rotate(0,0,-70)
        c = c.translate(h[0], h[1], 0)
        k += c



    return body - e + k

dump(model(),"kinesis_thumb_keys.scad")