from sumolib import checkBinary  # noqa
import traci
import pandas as pd
import matplotlib.pyplot as plt
from scipy import integrate
import xml.etree.ElementTree as ET
import os
import numpy as np
import multiprocessing as mp

cccc = 0

NO_GUI = True
TRAFFIC_TYPES = ['car1', 'truck1', 'bike1']
pd.set_option('display.max_columns', None)
METRIC = 'meanSpeedRelative'
GRIDLOCK_PENALTY = 0.1

N_LIMIT = 3600
N = 1000

NO_SIM = True  # skip re-simulating

# Setup sumo
if NO_GUI:
    sumoBinary = checkBinary('sumo')
else:
    sumoBinary = checkBinary('sumo-gui')


# MAIN FUNCTION TO CALL SIMULATOR
# Input: params - 21 length long vector - definition below
# Returns: Health metric depending on that set above. A higher value for this is better.

def call_sim_parallel(dparams):
    global cccc
    cccc += len(dparams)
    print("Call number "+str(cccc))
    pool = mp.Pool(len(dparams))
    calls = [(x, "number_" + str(i)) for i, x in enumerate(dparams)]
    out = pool.map(call_sim_zipped,calls)
    pool.close()
    return np.array([[x] for x in out]) # Package into 2D array


def call_sim_zipped(zipped):
    return call_sim(*zipped)


def call_sim(params, name="test", network_type='intersection', early_stop=False):
    ret_code = update_network(params,name)
    if not ret_code:
        return 0.0
    update_sumoconfig(name)
    grid_pen = go(name)
    grid_pen = grid_pen * GRIDLOCK_PENALTY
    return get_sum_stats(grid_pen, name)

def get_sum_stats(grid_pen, name, metric='meanSpeedRelative'):
    # metric = one of 'halting-ratio', 'meanTravelTime' or 'meanSpeedRelative'
    df = pd.read_xml("in_use_files/sum_" + str(name) + ".xml")[:-1]
    df = df.head(int(len(df) * 0.9)).tail(int(len(df) * 0.8))
    if metric == 'halting-ratio':
        df['halting-ratio'] = df['halting'] / df['running']
    metric_out = np.average(df[metric])
    print(metric_out, " - ", grid_pen, " = ", metric_out - grid_pen)

    return metric_out


def set_session():
    global session
    session = True


def go(name):
    if os.path.isfile("in_use_files/sum_" + str(name) + ".xml"):
        os.remove("in_use_files/sum_" + str(name) + ".xml")
    cmd = [sumoBinary, "-c", "in_use_files/"+str(name)+".sumocfg", "--start", "--quit-on-end", "--no-warnings",
           "--summary", "in_use_files/sum_" + str(name) + ".xml", "--tripinfo-output", "in_use_files/tripinfo_" + str(name) + ".xml",
           "--time-to-teleport", "-1"]
    traci.start(cmd, label=name)
    conn = traci.getConnection(name)
    iterations = 0

    last_inb_sp = -1
    last_outb_sp = -1
    while conn.simulation.getMinExpectedNumber() > 0:
        conn.simulationStep()
        iterations += 1
        if iterations % 500 == 0:
            cur_inb_sp = conn.edge.getLastStepMeanSpeed("end1_junction")
            cur_outb_sp = conn.edge.getLastStepMeanSpeed("junction_end1")

            if (cur_inb_sp == last_inb_sp and cur_outb_sp == last_outb_sp):
                print("Traffic Stabilized. Breaking...")
                conn.close()
                return 0
            else:
                last_outb_sp = cur_outb_sp
                last_inb_sp = cur_inb_sp
    print("---------------------"+str(iterations))
    conn.close()
    return 0


# total is total traffic flow on edge, ratios is ratio of proportions
# e.g. total flow of 10 and truck:car:bike ratio of 5:3:2 = 5 trucks, 3 cars, 2 bikes
# selected is which ratio element (by value)
def get_props_from_total(total, ratios, selected):
    return str(round(total * selected / sum(ratios), 5))

def update_sumoconfig(name):
    with open('./inter1.sumocfg','r') as f:
        s = f.read()
        s = s.replace("draft",str(name))
        with open("./in_use_files/"+str(name)+".sumocfg",'w+') as f2:
            f2.write(s)

def update_network(param_list, savename='test'):
    # Must have all routes flowing to stop it falling into just allowing the busiest lane to flow only.
    """    space = ParameterSpace([ContinuousParameter('traffic_light_1', 1, 4),   0 
                                        ContinuousParameter('traffic_light_2', 1, 4), 1
                                        ContinuousParameter('traffic_light_3', 1, 4),2
                                        ContinuousParameter('traffic_light_4', 1, 4),3
                                        ContinuousParameter('trucks', 1, 1),4
                                        ContinuousParameter('cars', 1, 2),5
                                        ContinuousParameter('bikes', 1, 1),6
                                        ContinuousParameter('heaviness',0,1),7
                                        ContinuousParameter('NB', 0, 1),8
                                        ContinuousParameter('EB', 0, 1),9
                                        ContinuousParameter('SB', 0, 1),10
                                        ContinuousParameter('WB', 0, 1) 11
                                        ])
                                        """
    # print(list(zip(param_list,range(0,len(param_list)))))

    with open(os.path.join(os.path.dirname(__file__), 'draft.rou.xml')) as f:
        root = ET.parse(f)

        heaviness = param_list[7] * N
        traffs = param_list[4:7]
        ps = param_list[8:]
        normalised_traffs = list(x / sum(traffs) for x in traffs)
        for i, n in enumerate(root.iter('flow')):
            source_direction = i // 9

            # 0: truck, east
            # 1: truck, south
            # 3: truck, west
            # 4: car, east

            # List of what probability to send E/S/W
            normalised_flows = get_normalised_flows(source_direction, ps)

            veh_prop = normalised_traffs[(i % 9) // 3]
            n.set('probability', str(round(veh_prop * normalised_flows[i % 3], 5)))
            n.set('end', str(heaviness))
            # print(9+k, prob, param_list[9+k],n.attrib)
            # so truck_ps[0] is flow for any incoming edge, for the going NORTH
    root.write("./in_use_files/" + str(savename) + ".rou.xml")

    # Update the traffic lights
    with open('./draft.net.xml') as f:
        root = ET.parse(f)
        zs = list(x for x in root.find('tlLogic') if 'y' not in x.get('state'))
        for i, phase in enumerate(zs):
            # print(phase.attrib, param_list[i], get_props_from_total(1,param_list[0:4],param_list[i]))
            phase.set('duration', get_props_from_total(60, param_list[0:4], param_list[i]))
        root.write("./in_use_files/" + str(savename) + ".net.xml")

    return True

def get_normalised_flows(excl, ps):
    ps2 = list(ps)
    del ps2[excl]
    return list(x / sum(ps2) for x in ps2)


# this is the main entry point of this script
if __name__ == "__main__":
    update_sumoconfig("test")
    go("test")
    get_sum_stats("test")
    """
    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if  NO_SIM: go()
    param_list = [99, 3, 106, 12,
                  0.5, 0.5,
                  1, 1, 1,
                  0.9, 0.9, 0.9, 0.9, 0.9, 0.45, 0.45, 0.45, 0.1, 0.1, 0.1, 0.1]
    update_network(param_list)
    go()"""