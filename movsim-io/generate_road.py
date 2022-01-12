import os
import re
import xml.etree.cElementTree as ET
from xml.dom import minidom

import numpy as np
from scenariogeneration import esmini
from scenariogeneration import xodr

import scenariogeneration_fixes as fixes

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
            jr = fixes.create_junction_roads(local_roads, angles, [40], junction=id + 1,
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


def generate_roundabout_2lane(spoke_roads, in_mask, r, d, startnum):
    sections = len(spoke_roads)
    spoke_contacts = [END if x else START for x in in_mask]
    merge_portion = 0.25

    exit_r = (d ** 2 + 2 * d * r) / (2 * r)

    exit_arc = xodr.Arc(0.5 / exit_r, angle=np.arctan2((r + d), exit_r))

    short_angle = np.arctan2(exit_r, r + d) * 2
    long_angle = ((2 * np.pi - sections * short_angle) / sections) * (1 - 2 * merge_portion)
    merge_angle = ((2 * np.pi - sections * short_angle) / sections) * merge_portion

    short_arc = xodr.Arc(0.5 / r, angle=short_angle)
    long_arc = xodr.Arc(0.5 / r, angle=long_angle)
    merge_arc = xodr.Arc(0.5 / (r + 6), angle=merge_angle)

    long_roads = []
    f_merge_roads = []
    s_merge_roads = []
    junction_roads = []
    junctions = []

    def jn_spoke(i):
        return startnum + (i % sections)

    def jn_merge_in(i):
        return startnum + (i % sections) + sections

    def jn_merge_out(i):
        return startnum + (i % sections) + 2 * sections

    def spoke_n(i):
        return spoke_roads[i % sections].id

    def long_n(i):
        return startnum + (i % sections) + sections

    def f_merge_n(i):
        return startnum + (i % sections) + 2 * sections

    def s_merge_n(i):
        return startnum + (i % sections) + 3 * sections

    def in_n(i):
        return startnum + (i % sections) + 4 * sections

    def out_n(i):
        return startnum + (i % sections) + 5 * sections

    def continue_n(i):
        return startnum + (i % sections) + 6 * sections

    for i in range(sections):
        if (in_mask[i]):
            spoke_roads[i].add_successor(xodr.ElementType.junction, jn_spoke(i))
        else:
            spoke_roads[i].add_predecessor(xodr.ElementType.junction, jn_spoke(i))

    # Create roundabout components
    for i in range(sections):
        long_roads.append(xodr.create_road([long_arc], id=long_n(i), left_lanes=0, right_lanes=2))
        long_roads[i].name = "circle road " + str(i)
        long_roads[i].add_predecessor(xodr.ElementType.road, f_merge_n(i), contact_point=END)
        long_roads[i].add_successor(xodr.ElementType.road, s_merge_n(i), contact_point=START)

        # TWO MERGE SECTIONS
        f_merge_roads.append(xodr.create_road([merge_arc], id=f_merge_n(i), left_lanes=0, right_lanes=4))
        f_merge_roads[i].name = "first merge road " + str(i)
        f_merge_roads[i].add_predecessor(xodr.ElementType.junction, jn_merge_in(i))
        f_merge_roads[i].add_successor(xodr.ElementType.road, long_n(i), contact_point=START)

        s_merge_roads.append(xodr.create_road([merge_arc], id=s_merge_n(i), left_lanes=0, right_lanes=4))
        s_merge_roads[i].name = "second merge road " + str(i)
        s_merge_roads[i].add_predecessor(xodr.ElementType.road, long_n(i), contact_point=END)
        s_merge_roads[i].add_successor(xodr.ElementType.junction, jn_merge_out(i + 1))

    # Create junctions
    for i in range(sections):
        in_road = xodr.create_road([exit_arc], id=in_n(i), left_lanes=0, right_lanes=2)
        in_road.name = "in_road " + str(i)
        in_road.add_predecessor(xodr.ElementType.junction, jn_merge_in(i))
        in_road.add_successor(xodr.ElementType.road, spoke_n(i), contact_point=spoke_contacts[i])

        out_road = xodr.create_road([exit_arc], id=out_n(i), left_lanes=0, right_lanes=2)
        out_road.name = "out_road " + str(i)
        out_road.add_predecessor(xodr.ElementType.road, spoke_n(i), contact_point=spoke_contacts[i])
        out_road.add_successor(xodr.ElementType.junction, jn_merge_out(i))

        continue_road = xodr.create_road([short_arc], id=continue_n(i), left_lanes=0, right_lanes=2)
        continue_road.name = "continue_road " + str(i)
        continue_road.add_predecessor(xodr.ElementType.junction, jn_merge_out(i))
        continue_road.add_successor(xodr.ElementType.junction, jn_merge_out(i))

        junction_roads.append(in_road)
        junction_roads.append(out_road)
        junction_roads.append(continue_road)

        junctions.append(xodr.create_junction([], id=jn_merge_out(i),
                                              roads=[s_merge_roads[i - 1], out_road, continue_road]))

        junctions.append(xodr.create_junction([], id=jn_merge_in(i),
                                              roads=[continue_road, in_roads, f_merge_roads[i]]))

        junctions.append(xodr.create_junction([], id=jn_spoke(i),
                                              roads=[in_road, spoke_roads[i], out_road]))

    all_roads = long_roads + f_merge_roads + s_merge_roads + junction_roads

    return all_roads, junctions


def generate_faulty_roundabout(lanes, in_roads, in_mask, r, d_road, startnum):
    sections = len(in_roads)
    lane_width = 3
    d = d_road

    exit_r = (d ** 2 + 2 * d * r) / (2 * r)
    in_arc = xodr.Arc(0.5 / (exit_r - lane_width * lanes / 2), angle=np.arctan2((r + d), exit_r))
    out_arc = xodr.Arc(-0.5 / (exit_r), angle=np.arctan2((r + d), exit_r))

    short_angle = np.arctan2(exit_r, r + d) * 2
    short = xodr.Arc(0.5 / r, angle=short_angle)
    long = xodr.Arc(0.5 / r, angle=(2 * np.pi - sections * short_angle) / sections)

    circle_roads = []
    junction_roads = []
    junctions = []

    for i in range(sections):
        if (in_mask[i]):
            in_roads[i].add_successor(xodr.ElementType.junction, startnum + i)
        else:
            in_roads[i].add_predecessor(xodr.ElementType.junction, startnum + i)

    for i in range(sections):
        circle_roads.append(xodr.create_road([long], id=startnum + sections + i, lane_width=lane_width,
                                             left_lanes=0, right_lanes=lanes))
        circle_roads[i].name = "circle road " + str(i)
        circle_roads[i].add_predecessor(xodr.ElementType.junction, startnum + i + sections)
        circle_roads[i].add_successor(xodr.ElementType.junction, startnum + (i - 1) % sections)

    for i in range(sections):
        local_joints = []
        junction_id = startnum + i
        # Within the roundabout
        in_circle = xodr.create_road([short], id=startnum + i + 2 * sections, lane_width=lane_width,
                                     left_lanes=0, right_lanes=lanes, road_type=junction_id)
        in_circle.add_predecessor(xodr.ElementType.road, startnum + sections + ((i - 1) % sections), contact_point=END)
        in_circle.add_successor(xodr.ElementType.road, startnum + sections + i, contact_point=START)
        in_circle.name = "in_circle " + str(i)

        # Into the roundabout
        in_junct = xodr.create_road([in_arc], id=startnum + i + sections * 3, lane_width=lane_width,
                                    left_lanes=0, right_lanes=lanes, road_type=junction_id)
        in_junct.add_predecessor(xodr.ElementType.road, in_roads[i].id, contact_point=END, lane_offset=lanes)
        in_junct.add_successor(xodr.ElementType.road, startnum + sections + i, contact_point=END)
        in_junct.name = "in_junct " + str(i)

        # Out of roundabout
        out_junct = xodr.create_road([out_arc], id=startnum + i + sections * 4,
                                     lane_width=lane_width, left_lanes=0, right_lanes=lanes, road_type=junction_id)
        out_junct.add_successor(xodr.ElementType.road, startnum + sections + ((i - 1) % sections),
                                contact_point=START, lane_offset=0)
        out_junct.add_predecessor(xodr.ElementType.road, in_roads[i].id, contact_point=END)
        out_junct.name = "out_junct " + str(i)

        local_joints.append(out_junct)
        local_joints.append(in_circle)
        local_joints.append(in_junct)

        junction_roads.append(local_joints)
        junctions.append(xodr.create_junction(local_joints, id=junction_id,
                                              roads=[circle_roads[i - 1], circle_roads[i], in_roads[i]]))

    all_roads = circle_roads
    for j in junction_roads:
        all_roads = all_roads + j

    return all_roads, junctions


def join_all(road_sets, junction_sets, filename):
    odr = fixes.FixedOpenDrive('myroads')

    for rs in road_sets:
        for r in rs:
            odr.add_road(r)

    odr.adjust_roads_and_lanes()
    for js in junction_sets:
        for j in js:
            odr.add_junction(j)
    odr.write_xml(filename)
    return odr


# Download from https://github.com/esmini/esmini/releases/tag/v2.18.2
def show_road(odr, esmini_path='esmini-linux'):
    esmini(odr, os.path.join(esmini_path), car_density=20)


def post_process(filename):
    tree = ET.parse(filename)
    root = tree.getroot()

    def filter_attribute(element, attribute):
        for e in root.iter(element):
            if attribute in e.attrib:
                e.attrib.pop(attribute)

    def filter_element(condition):
        def iterator(parents, nested=False):
            for child in parents:
                if condition(child):
                    parents.remove(child)
                if nested:
                    iterator(child, nested=True)

        iterator(root, nested=True)

    def change_attribute(element, attribute, old_value, new_value):
        for e in root.iter(element):
            if attribute in e.attrib:
                if e.attrib[attribute] == old_value:
                    e.attrib[attribute] = new_value

    def reverse_order(parent, wrongfirst):
        for e in root.iter(parent):
            if e.tag == parent and len(e) > 0 and list(e)[0].tag == wrongfirst:
                l2 = list(e)
                l2.reverse()
                e[:] = l2

    filter_attribute('road', 'rule')
    change_attribute('predecessor', 'id', '0', '1')
    filter_element(condition=(lambda x: x.tag == 'roadMark'))
    filter_element(condition=(lambda x: x.tag == 'center'))
    filter_element(condition=(lambda x: x.tag == 'elevationProfile'))
    filter_element(condition=(lambda x: 'type' in x.attrib and x.attrib['type'] == 'none'))

    reverse_order('link', 'successor')

    xmlstr = minidom.parseString(ET.tostring(root)).toprettyxml(indent="   ")
    xmlstr = "\n".join([ll.rstrip() for ll in xmlstr.splitlines() if ll.strip()])
    with open(filename, 'w') as f:
        f.write(xmlstr)


if __name__ == '__main__':
    in_roads = []
    for i in range(4):
        in_roads.append(xodr.create_straight_road(i + 1, 100))
    in_mask = [True, False, False, False]  # Defines which way the roads "point"
    roads, junctions = generate_roundabout_2lane(spoke_roads=in_roads, in_mask=in_mask, r=10, d=4, startnum=100)
    odr = join_all([in_roads, roads], [junctions], "generated_road.xodr")
    post_process("generated_road.xodr")
    show_road(odr)
    # roads,intersections = square_grid(5,5,lanes=2)
    # generate_road(roads,intersections,intersection_type="simple",visualize=True,debug_level=0)
