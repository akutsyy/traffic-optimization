import re
import xml.etree.cElementTree as ET
from xml.dom import minidom

def post_process_generated(filename):
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

def post_process_osm(filename):
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
    filter_element(condition=(lambda x: x.tag == 'speed'))
    filter_element(condition=(lambda x: x.tag == 'geoReference'))
    filter_element(condition=(lambda x: 'type' in x.attrib and x.attrib['type'] == 'none'))

    reverse_order('link', 'successor')

    xmlstr = minidom.parseString(ET.tostring(root)).toprettyxml(indent="   ")
    xmlstr = "\n".join([ll.rstrip() for ll in xmlstr.splitlines() if ll.strip()])
    with open(filename, 'w') as f:
        f.write(xmlstr)

if __name__ == '__main__':
    post_process_osm("output.xodr")