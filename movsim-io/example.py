"""
    An example, using the generators, showing how to create different types of junctions
    All the roads in a junction are formed by clothoid-arc-clothoid geotrietries
    In this example we can create a junction given

        option1. the distance R of every road from the center of the junction - R can change for each road

        option2. the radius of the inner arc geometry - for this case you can only use
                                                        numintersections = 3 or 4
                                                        angles = [0,np.pi/2, 3*np.pi/2] or [0, np.pi/2, np.pi, 3*np.pi/2]
    Some features used:

    - create_straight_road
    - create_junction_roads
    - create_junction
"""

import numpy as np
import os
from scenariogeneration import xodr

roads = []
numintersections = 4
angles = []
for i in range(numintersections):
    roads.append(xodr.create_straight_road(i))
    # use this instead to change the number of lanes in the crossing
    # roads.append(xodr.generators.create_straight_road(i, length=100, junction=-1, n_lanes=5, lane_offset=3))
    angles.append(i * 2 * np.pi / numintersections)

# option 1. uncomment this if you want to create the junction from the distance R of every road from the center of the junction
junction_roads = xodr.create_junction_roads(roads, angles, [8])
# option 2. creation of junction given the radius of the inner arc geometry
# junction_roads = xodr.create_junction_roads_from_arc(roads,angles,8)

# create the junction
junction = xodr.create_junction(junction_roads, 1, roads)

# create the opendrive and add all roads and the junction

odr = xodr.OpenDrive('myroad')

odr.add_junction(junction)

for r in roads:
    odr.add_road(r)

for j in junction_roads:
    odr.add_road(j)

for r in roads:
    print(r.id)
    if not r.predecessor is None:
        print("pred_link: " + str(r.predecessor.element_id))
    if not r.successor is None:
        print("succ_link: " + str(r.successor.element_id))
odr.adjust_roads_and_lanes()

# write the OpenDRIVE file as xodr using current script name
odr.write_xml(os.path.basename(__file__).replace('.py', '.xodr'))

# uncomment the following lines to display the road using esmini
# from scenariogeneration import esmini
# esmini(odr,os.path.join('esmini'))