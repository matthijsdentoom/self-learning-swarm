import xml.etree.ElementTree as ET
import os


class MissionChanger:
    """This class changes can be used to change a mission"""

    def __init__(self, mission_file):
        self.tree = ET.parse(mission_file)

    def add_attribute(self, node, parameter, value):
        """"Add an attribute the the given node."""
        root = self.tree.getroot()
        run_node = root.find(node)
        run_node.attrib[parameter] = value

    def set_log_file(self, path_name):
        """"Sets the logfile directory to the given pathname."""
        root = self.tree.getroot()
        log_dir_node = root.find('log_dir')
        log_dir_node.text = path_name
