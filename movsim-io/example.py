"""
    An example, using the generators, showing how to create a simple highway with exits and entries

    Shows how to patch created roads together with successor/predecessor, together with the lane_offset option

    Some features used:

    - create_road

    - add_successor/add_predecessor with and without the lane_offset option

    - create_junction
"""
from scenariogeneration import xodr
import os

# create some simple roads
from generate_road import post_process
import generate_road

# create some roads
roads = []
roads.append(xodr.create_road(xodr.Line(100), id=0, left_lanes=1, right_lanes=1))
roads.append(xodr.create_road(xodr.Line(100), id=1, left_lanes=1, right_lanes=1))

roads.append(xodr.create_road(xodr.Line(100), id=2, left_lanes=1, right_lanes=1))

# add some connections to non junction roads
roads[0].add_successor(xodr.ElementType.junction, 1)
roads[1].add_predecessor(xodr.ElementType.junction, 1)

roads[2].add_predecessor(xodr.ElementType.road,0,contact_point=xodr.ContactPoint.end)
roads[2].add_successor(xodr.ElementType.road,1,contact_point=xodr.ContactPoint.start)

junction = xodr.create_junction([roads[2]], 1, [roads[0], roads[1]])

# create the opendrive
odr = xodr.OpenDrive('myroad')
for r in roads:
    odr.add_road(r)
odr.adjust_roads_and_lanes()
odr.add_junction(junction)

# write the OpenDRIVE file as xodr using current script name
odr.write_xml(os.path.basename(__file__).replace('.py','.xodr'))
post_process(os.path.basename(__file__).replace('.py','.xodr'))
# uncomment the following lines to display the road using esmini
from scenariogeneration import esmini
esmini(odr,os.path.join('esmini-linux'))