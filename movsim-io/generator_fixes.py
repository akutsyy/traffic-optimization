import numpy as np
import pyclothoids as pcloth
import scenariogeneration.xodr.generators as g
from scenariogeneration.xodr.enumerations import ContactPoint, ElementType
from scenariogeneration.xodr.exceptions import GeneralIssueInputArguments
from scenariogeneration.xodr.geometry import Line, PlanView
from scenariogeneration.xodr.lane import LaneSection, Lanes
from scenariogeneration.xodr.opendrive import Road


def create_junction_roads(roads, angles, R, junction=1, arc_part=1 / 3, startnum=100,debug_level=0):
    """ creates all needed roads for some simple junctions, the curved parts of the junction are created as a spiral-arc-spiral combo
        R is value to the the radius of the whole junction (meaning R = distance between the center of the junction and any external road attached to the junction)
        Supportes all angles and number of roads.

        Parameters
        ----------
            roads (list of Road): all roads that should go into the junction

            angles (list of float): the angles from where the roads should be coming in (see description for what is supported),
                                    to be defined in mathimatically positive order, beginning with the first incoming road [0, +2pi]

            R (list of float): the radius of the whole junction, meaning the distance between roads and the center of the junction. If only one value is specified, then all roads will have the same distance.

            junction (int): the id of the junction
                default: 1

            spiral_part (float): the part of the curve that should be spirals (two of these) spiral_part*2 + arcpart = angle of the turn
                default: (1/3)

            arc_part (float): the part of the curve that should be an arc:  spiral_part*2 + arcpart = angle of the turn
                default: (1/3)

            startnum (int): start number of the roads in the junctions (will increase with 1 for each road)

        Returns
        -------
            junction_roads (list of Road): a list of all roads needed for all traffic connecting the roads
    """

    if (len(roads) is not len(angles)):
        raise GeneralIssueInputArguments('roads and angles do not have the same size.')

    if len(R) == 1:
        R = R * np.ones(len(roads))
    elif len(R) > 1 and len(R) is not len(roads):
        raise GeneralIssueInputArguments('roads and R do not have the same size.')

    # arc_part = 1 - 2*spiral_part
    spiral_part = (1 - arc_part) / 2

    # linelength = 2*R
    junction_roads = []

    use_sucessor = [r.successor is None for r in roads]
    if(debug_level>=10):
        print(use_sucessor)

    # loop over the roads to get all possible combinations of connecting roads
    for i in range(len(roads)-1):
        # for now the first road is place as base,
        if use_sucessor[i]:
            roads[i].add_successor(ElementType.junction, junction)
        else:
            roads[i].add_predecessor(ElementType.junction, junction)

        for j in range(1+i,len(roads)):
            # check angle needed for junction [-pi, +pi]
            an1 = angles[j] - angles[i] - np.pi
            # adjust angle if multiple of pi
            if an1 > np.pi:
                an1 = -(2 * np.pi - an1)

            angle_arc = an1 * arc_part
            angle_cloth = an1 * spiral_part
            sig = np.sign(an1)

            # create road, either straight or curved
            n_lanes, lanes_offset = g.get_lanes_offset(roads[i], roads[j], ContactPoint.end if use_sucessor[i] else ContactPoint.start)
            if sig == 0:
                # create straight road
                linelength = R[i] + R[j]
                tmp_junc = g.create_straight_road(startnum, length=linelength, junction=junction, n_lanes=n_lanes,
                                                lane_offset=lanes_offset)
            else:
                clothoids = pcloth.SolveG2(-R[i], 0, 0, g.STD_START_CLOTH, R[j] * np.cos(an1), R[j] * np.sin(an1), an1,
                                           g.STD_START_CLOTH)
                tmp_junc = g.create_3cloths(clothoids[0].KappaStart, clothoids[0].KappaEnd, clothoids[0].length,
                                          clothoids[1].KappaStart, clothoids[1].KappaEnd, clothoids[1].length,
                                          clothoids[2].KappaStart, clothoids[2].KappaEnd, clothoids[2].length, startnum,
                                          junction, n_lanes=n_lanes, lane_offset=lanes_offset)

            # add predecessor and successor
            tmp_junc.add_predecessor(ElementType.road, roads[i].id, ContactPoint.end if use_sucessor[i] else ContactPoint.start)
            tmp_junc.add_successor(ElementType.road, roads[j].id, ContactPoint.end if use_sucessor[j] else ContactPoint.start)

            startnum += 1
            junction_roads.append(tmp_junc)

    # add junction to the last road aswell since it's not part of the loop
    if use_sucessor[-1]:
        roads[-1].add_successor(ElementType.junction, junction)
    else:
        roads[-1].add_predecessor(ElementType.junction, junction)

    return junction_roads