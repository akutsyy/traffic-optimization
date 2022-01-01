import scenariogeneration.xodr.links
from scenariogeneration import xodr
from scenariogeneration import esmini

import os
import numpy as np
import generator_fixes
import random

FOURWAY = [0, np.pi / 2, np.pi, 3 * np.pi / 2]

def toy_example():
    roads = []

    for i in range(6):
        roads.append(xodr.create_straight_road(i))

    intersections = [[4, 3, 0, 5], [0, 1, -1, -1], [1, 2, -1, -1], [2, 3, -1, -1]]
    return roads,intersections

def square_grid(m,n):
    num_inner = 2*m*n-m-n
    num_outer = 2*m+2*n
    num_roads = num_outer+num_inner
    roads = []
    for i in range(num_roads):
        roads.append(xodr.create_straight_road(i))
    intersections = []
    x = 0
    for i in range(n):
        for j in range(m):
            intersections.append([x,x+m,x+2*m+1,x+m+1])
            x = x+1
        x = x+m+1
    return roads,intersections



def generate_road(roads,intersections,filename="generated_road.xodr",debug_level=0,visualize=False,esmini_path = "esmini-linux"):
    junction_roads = []
    junctions = []

    for id, intersection in enumerate(intersections):
        local_roads = [roads[i] for i in intersection if i >= 0]
        angles = [FOURWAY[i] for i, x in enumerate(intersection) if x >= 0]
        to_flip = []
        jr = generator_fixes.create_junction_roads(local_roads, angles, [40], junction=id + 1, startnum=1000 * (id + 1),debug_level=debug_level)
        junction_roads.append(jr)

        j = xodr.create_junction(jr, id + 1, local_roads)
        junctions.append(j)

    if debug_level>20:
        for r in roads:
            print(r.id)
            if not r.predecessor is None:
                print("pred_link: " + str(r.predecessor.element_id))
            if not r.successor is None:
                print("succ_link: " + str(r.successor.element_id))

        print("junction roads:")
        for jr in junction_roads:
            for r in jr:
                print(r.id)
                if not r.predecessor is None:
                    print("pred_link: " + str(r.predecessor.element_id))
                if not r.successor is None:
                    print("succ_link: " + str(r.successor.element_id))

    odr = xodr.OpenDrive('myroads')

    for j in junctions:
        odr.add_junction(j)

    for r in roads:
        odr.add_road(r)

    for j in junction_roads:
        for r in j:
            odr.add_road(r)

    odr.adjust_roads_and_lanes()

    # write the OpenDRIVE file as xodr using current script name
    odr.write_xml(filename)

    # uncomment the following lines to display the road using esmini
    # Download from https://github.com/esmini/esmini/releases/tag/v2.18.2
    if(visualize):

        esmini(odr, os.path.join('esmini-linux'), car_density=5)

if __name__ == '__main__':
    roads,intersections = square_grid(300,300)
    generate_road(roads,intersections,visualize=True,debug_level=0)