from scenariogeneration import xodr


def get_xml_for_road(r):
    odr = xodr.OpenDrive('myroad')
    odr.add_road(r)
    odr.adjust_roads_and_lanes()
    odr.write_xml('temp.xodr')

if __name__ == '__main__':
    roads = []
    roads.append(xodr.create_road(xodr.Line(100), id=0, left_lanes=2, right_lanes=2))
    roads[0].add_successor(xodr.ElementType.road,1,contact_point=xodr.ContactPoint.start)
    roads[0].add_predecessor(xodr.ElementType.junction,1,contact_point=xodr.ContactPoint.start)
    get_xml_for_road(roads[0])

