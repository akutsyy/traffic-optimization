import xml.etree.cElementTree as ET
from xml.dom import minidom

save_path_file = "generated_xprj.xprj"


# Not used: per-road initial conditions
def create_xprj(network_filename, vehicle_types, simulation_properties, traffic_composition, road_composition=None):
    root = ET.Element('Movsim')

    vehicle_prototypes = ET.SubElement(root, "VehiclePrototypes")
    for v in vehicle_types:
        v_config = ET.SubElement(vehicle_prototypes, "VehiclePrototypeConfiguration")
        for key, value in v.properties.items():
            v_config.set(key, value)
        a_wrapper = ET.SubElement(v_config, "AccelerationModelType")
        a_config = ET.SubElement(a_wrapper, v.acceleration_model_type)
        for key, value in v.acceleration_properties.items():
            a_config.set(key, value)
        l_wrapper = ET.SubElement(v_config, "LaneChangeModelType")
        for key, value in v.lane_change_model_properties.items():
            l_wrapper.set(key, value)
        l_config = ET.SubElement(l_wrapper, v.lane_change_model)
        for key, value in v.lane_change_properties.items():
            l_config.set(key, value)

    scenario = ET.SubElement(root, "Scenario", network_filename=network_filename)
    simulation = ET.SubElement(scenario, "Simulation")
    for key, value in simulation_properties.items():
        simulation.set(key, value)
    global_composition = ET.SubElement(simulation, "TrafficComposition")
    for tc in traffic_composition:
        type = ET.SubElement(global_composition, "VehicleType")
        for key, value in tc.items():
            type.set(key, value)
    if not road_composition is None:
        for r in road_composition:
            road = ET.SubElement(simulation, "Road", id=r.id)
            if not r.composition is None:
                local_composition = ET.SubElement(road, "TrafficComposition")
                for tc in r.composition:
                    type = ET.SubElement(local_composition, "VehicleType")
                    for key, value in tc.items():
                        type.set(key, value)
            if not r.source is None:
                source = ET.SubElement(road, "TrafficSource")
                for i in r.source:
                    inflow = ET.SubElement(source, "Inflow")
                    for key, value in i.items():
                        inflow.set(key, value)

    xmlstr = minidom.parseString(ET.tostring(root)).toprettyxml(indent="   ")
    with open(save_path_file, 'w') as f:
        f.write(xmlstr)


default_acceleration_model_type = "ModelParameterACC"
default_acceleration_properties = {"v0": "35", "T": "1.2", "s0": "3", "s1": "2", "delta": "4", "a": "1.2", "b": "2.0",
                                   "coolness": "1"}

default_lane_change_model = {"european_rules": "true", "crit_speed_eur": "20"}
default_lane_change_model_properties = {"european_rules": "true", "crit_speed_eur": "20"}
default_lane_change_properties = {"safe_deceleration": "5.0", "minimum_gap": "2.0", "threshold_acceleration": "0.1",
                                  "right_bias_acceleration": "0.05", "politeness": "0.1"}

default_simulation_properties = {"timestep": "0.2", "crash_exit": "false"}


class VehicleType():
    def __init__(self, label, length=6, maximum_deceleration=9,
                 acceleration_model_type=default_acceleration_model_type,
                 acceleration_properties=default_acceleration_properties,
                 lane_change_model="ModelParameterMOBIL",
                 lane_change_model_properties=default_lane_change_model_properties,
                 lane_change_properties=default_lane_change_properties):
        self.label = str(label)
        self.properties = {"label": str(label), "length": str(length),
                           "maximum_deceleration": str(maximum_deceleration)}
        self.acceleration_model_type = acceleration_model_type
        self.acceleration_properties = acceleration_properties
        self.lane_change_model = lane_change_model
        self.lane_change_model_properties = lane_change_model_properties
        self.lane_change_properties = lane_change_properties


if __name__ == '__main__':
    network_filename = "generated_road.xodr"
    vehicle_types = [VehicleType("car"), VehicleType("truck", length=16, maximum_deceleration=2)]
    simulation_properties = default_simulation_properties
    traffic_composition = [{"label": "car", "fraction": "0.8", "relative_v0_randomization": "0.2"},
                           {"label": "truck", "fraction": "0.2", "relative_v0_randomization": "0.2"}]

    create_xprj(network_filename, vehicle_types, simulation_properties, traffic_composition)
