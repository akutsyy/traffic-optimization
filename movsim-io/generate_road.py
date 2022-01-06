import scenariogeneration.xodr.links
from scenariogeneration import xodr
from scenariogeneration import esmini

import os
import numpy as np
import generator_fixes
import random

FOURWAY = [0, np.pi / 2, np.pi, 3 * np.pi / 2]
START = xodr.ContactPoint.start
END = xodr.ContactPoint.end


def toy_example():
    roads = []

    for i in range(6):
        roads.append(xodr.create_straight_road(i))

    intersections = [[4, 3, 0, 5], [0, 1, -1, -1], [1, 2, -1, -1], [2, 3, -1, -1]]
    return roads, intersections


def square_grid(m, n, lanes=1):
    num_inner = 2 * m * n - m - n
    num_outer = 2 * m + 2 * n
    num_roads = num_outer + num_inner
    roads = []
    for i in range(num_roads):
        roads.append(xodr.create_straight_road(i, n_lanes=lanes))
    intersections = []
    x = 0
    for i in range(n):
        for j in range(m):
            intersections.append([x, x + m, x + 2 * m + 1, x + m + 1])
            x = x + 1
        x = x + m + 1
    return roads, intersections


def generate_road(roads, intersections, intersection_type="direct", filename="generated_road.xodr", debug_level=0,
                  visualize=False, esmini_path="esmini-linux"):
    junction_roads = []
    junctions = []

    for id, intersection in enumerate(intersections):
        local_roads = [roads[i] for i in intersection if i >= 0]
        if intersection_type == "simple":
            angles = [FOURWAY[i] for i, x in enumerate(intersection) if x >= 0]
            jr = generator_fixes.create_junction_roads(local_roads, angles, [40], junction=id + 1,
                                                       startnum=1000 * (id + 1), debug_level=debug_level)
            junction_roads.append(jr)

            j = xodr.create_junction(jr, id + 1, local_roads)

            junctions.append(j)
        elif intersection_type == "roundabout":
            angles = [FOURWAY[i] for i, x in enumerate(intersection) if x >= 0]
            lanes = len(local_roads[0].lanes.lanesections[-1].leftlanes)
            roundabout_sections = []
            roundabout_section_geometry = xodr.Arc()

    if debug_level > 20:
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
    if (visualize):
        esmini(odr, os.path.join('esmini-linux'), car_density=5)


def generate_circle(lanes, r, d_road, startnum):
    sections = 4
    short = xodr.Arc(0.5 / r, angle=3.14 / 180 * 15)
    long = xodr.Arc(0.5 / r, angle=(360 - sections * 15) / sections * 3.14 / 180)
    d = d_road
    little_r = (d ** 2 + 2 * d * r) / (2 * r)
    in_arc = xodr.Arc(0.5 / little_r, angle=np.arctan2((r + d), little_r))
    out_arc = xodr.Arc(0.5 / little_r, angle=np.arctan2((r + d), little_r))

    in_roads = []
    circle_roads = []
    junction_roads = []
    junctions = []

    print("in roads")
    for i in range(sections):
       print(i)
       in_roads.append(xodr.create_straight_road(i))
       in_roads[i].add_successor(xodr.ElementType.junction, i)

    print("circle roads")
    for i in range(sections):
        circle_roads.append(xodr.create_road([long], id=startnum + i, left_lanes=lanes, right_lanes=lanes))
        circle_roads[i].add_predecessor(xodr.ElementType.junction, startnum + i)
        circle_roads[i].add_successor(xodr.ElementType.junction, startnum + (i - 1) % sections)
        print(startnum + i)

    print("junction roads")
    for i in range(sections):
        local_joints = []
        junction_id = i
        # Within the roundabout
        print(startnum + i + sections)
        in_circle = xodr.create_road([short], id=startnum + i + sections, left_lanes=lanes, right_lanes=lanes,
                                     road_type=junction_id)
        in_circle.add_predecessor(xodr.ElementType.road, startnum + ((i - 1) % sections), contact_point=END)
        in_circle.add_successor(xodr.ElementType.road, startnum + i, contact_point=START)

        print(startnum + i + sections * 2)
        # Into the roundabout
        in_junct = xodr.create_road([in_arc], id=startnum + i + sections * 2, left_lanes=lanes, right_lanes=lanes,
                                    road_type=junction_id)
        in_junct.add_successor(xodr.ElementType.road, startnum + i, contact_point=END)

        print(startnum + i + sections * 3)
        print(str(startnum + ((i - 1) % sections))+"   "+str(startnum+i))
        # Out of roundabout
        out_junct = xodr.create_road([out_arc], id=startnum + i + sections * 3, left_lanes=lanes, right_lanes=lanes,
                                     road_type=junction_id)
        out_junct.add_predecessor(xodr.ElementType.road, startnum + ((i - 1) % sections), contact_point=START)
        out_junct.add_successor(xodr.ElementType.road, i, contact_point=END)
        in_junct.add_predecessor(xodr.ElementType.road, i, contact_point=END)

        local_joints.append(out_junct)
        local_joints.append(in_circle)

        local_joints.append(in_junct)


        junction_roads.append(local_joints)
        junctions.append(xodr.create_junction(local_joints, id=junction_id,
                                              roads=[circle_roads[i - 1],circle_roads[i],in_roads[i]]))

    odr = xodr.OpenDrive('myroads')

    for r in in_roads:
        odr.add_road(r)
    for r in circle_roads:
        odr.add_road(r)
    for j in junction_roads:
        for r in j:
            odr.add_road(r)
    odr.adjust_roads_and_lanes()

    for j in junctions:
        odr.add_junction(j)
        print("junction "+str(j.id))


    # write the OpenDRIVE file as xodr using current script name
    odr.write_xml("circle.xodr")

    # uncomment the following lines to display the road using esmini
    # Download from https://github.com/esmini/esmini/releases/tag/v2.18.2
    esmini(odr, os.path.join('esmini-linux'), car_density=5)


if __name__ == '__main__':
    generate_circle(1, 10, 5, startnum=100)
    # roads,intersections = square_grid(1,1,lanes=2)
    # generate_road(roads,intersections,intersection_type="simple",visualize=True,debug_level=0)
