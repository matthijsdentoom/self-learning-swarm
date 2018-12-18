import xml.etree.ElementTree as ET


class MissionChanger:
    """This class changes can be used to change a mission-file"""

    def __init__(self, mission_file):
        self.tree = ET.parse(mission_file)
        self.root = self.tree.getroot()

    def add_attribute(self, node, parameter, value):
        """"Add an attribute the the given node."""
        run_node = self.root.find(node)
        run_node.attrib[parameter] = value

    def set_log_file(self, path_name):
        """"Sets the logfile directory to the given pathname."""
        log_dir_node = self.root.find('log_dir')
        log_dir_node.text = path_name

    def enable_multi_threading(self, num_threads):
        """Enables multi-threading on the given number of threads."""
        threaded_node_name = 'multi_threaded'
        threaded_node = self.root.find(threaded_node_name)

        if threaded_node is None:
            print('None for a strange reason.')
            threaded_node = ET.Element(threaded_node_name)
            threaded_node.text = "true"
            self.root.insert(-1, threaded_node)

        threaded_node.attrib['num_threads'] = str(num_threads)

    def write(self, filename):
        """This function writes the new mission file to the given location."""
        self.tree.write(filename)

    def update_entity_count(self, entity_autonomy_name, count):
        """This function updates the entity count of the entity whose autonomy module is called entity_autonomy_name
            in the mission file."""
        for entity_node in self.root.findall('entity'):
            autonomy_node = entity_node.find('autonomy')
            if autonomy_node.text == entity_autonomy_name:

                count_node = entity_node.find('count')
                if count_node is None:  # If the count node does not exist create it.
                    count_node = ET.Element('count')
                    entity_node.insert(0, count_node)

                count_node.text = str(count)
