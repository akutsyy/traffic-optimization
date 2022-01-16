from sumolib import checkBinary  # noqa
import traci
import pandas as pd
import matplotlib.pyplot as plt
from scipy import integrate
import xml.etree.ElementTree as ET
import os

global session
session = False
NO_GUI = False
TRAFFIC_TYPES = ['car1', 'truck1', 'bike1']
pd.set_option('display.max_columns', None)
METRIC = 'meanSpeedRelative'

NO_SIM = True  # skip re-simulating

# Setup sumo
if NO_GUI:
    sumoBinary = checkBinary('sumo')
else:
    sumoBinary = checkBinary('sumo-gui')


#MAIN FUNCTION TO CALL SIMULATOR
#Input: params - 21 length long vector - definition below
#Returns: Health metric depending on that set above. A higher value for this is better.
def call_sim(params, network_type='intersection'):
    ret_code = update_network(network_type, params)
    if not ret_code:
        return 0.0
    go()
    return get_sum_stats(METRIC)


def get_sum_stats(metric='meanSpeedRelative'):
    # metric = one of 'halting-ratio', 'meanTravelTime' or 'meanSpeedRelative'
    df = pd.read_xml("sum.xml")[:-1]
    if metric == 'halting-ratio':
        df['halting-ratio'] = df['halting'] / df['running']
    # df.plot(x="ended",y=metric)
    # plt.show()
    metric_out = int(int(integrate.trapezoid(df['ended'], df[metric])) / df['ended'].values[-1])
    return metric_out

def set_session():
    global session
    session = True

def go():
    cmd = [sumoBinary, "-c", "inter1.sumocfg", "--start",
           "--summary", "sum.xml", "--tripinfo-output", "tripinfo.xml"]
    if not session:
        traci.start(cmd)
        set_session()
    else:
        traci.load(cmd[1:])
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()


# total is total traffic flow on edge, ratios is ratio of proportions
# e.g. total flow of 10 and truck:car:bike ratio of 5:3:2 = 5 trucks, 3 cars, 2 bikes
#selected is which ratio element (by value)
def get_props_from_total(total, ratios, selected):
    return str(round(total * selected / sum(ratios),5))


def update_network(network_type, param_list):
    # Must have all routes flowing to stop it falling into just allowing the busiest lane to flow only.
    if 0 in param_list[0:4]:
        return False

    #Mock param list:
    param_list = [99,3,106,12,
                  0.5,0.5,
                  1,1,1,
                  0.9,0.9,0.9,0.9,0.9,0.45,0.45,0.45,0.1,0.1,0.1,0.1]
    assert len(param_list) == 21

    """
    Params 0 - 3 = Traffic light setups - ratio values (are normalised before use!)
            Default phases are
            0: N->S and S->N
            1: E->W and W->E
            2: N->E and S->W
            3: W->N and E->S
    Params 4, 5 = Sigma and tau - see Krauss Model -  https://sumo.dlr.de/docs/Definition_of_Vehicles%2C_Vehicle_Types%2C_and_Routes.html
    Params 6, 7, 8 = Ratio of trucks (6) , cars (7) , bikes (8) (as whole-number ratios/parts).
    Params 9 to 20 (including 20) are matrix of flow probabilities for each source->dest; skipping the leading diagonal.
    Flow probability = probability of a vehicle being emitted in any one second, yields binomial -
    see: https://sumo.dlr.de/docs/Simulation/Randomness.html#flows_with_a_random_number_of_vehicles
    NB: Not modelled per lane so can have duplicate spawns of vehicle types; sumo handles these rare cases by dropping extra vehicles.
        (NB in the xml files, SUMO uses 1=North 2=East 3=West 4=South)
        9: N->E (i.e. 1->2) 10: N->S  11: N->W
        12: E->N  13: E->S  14: E->W
        15: S->N  16: S->E  17: S->W
        18: W->N  19: W->E  20: W->S

    """
    print(list(zip(param_list,range(0,len(param_list)))))

    with open(os.path.join(os.path.dirname(__file__), './draft.rou.xml')) as f:
        root = ET.parse(f)

        # Update probability of emissions ( = flow):
        for i, n in enumerate(root.iter('flow')):
            # i is the index of the specific ((source,dest),vehicle). There are 3 vehicle types.
            # k is the index into the parameters for the specific source->dest. These do not distinguish by vehicle,
            #so there are a third of the overall number of flows (i). So i = 3k (trucks) or 3k + 1 (cars) or 3k+2 (bikes)
            k = i // 3

            #First arg - the overall flow (sum of all vehicle types) which should be assigned to the flow. This is what
            #is passed in as a parameter for the specific flow - add 9 as this starts from index 9.
            #Second arg - the ratio values for the three types of vehicle.
            #Third arg - the ratio value for whichever of the vehicle types are currently being assigned - add 6 as this
            #starts form index 6. i - 3k = 1 or 2 or 3 i.e. the vehicle type.
            prob = get_props_from_total(param_list[9+k],param_list[6:9],param_list[6+i-(3*k)])
            n.set('probability',prob)
            #print(9+k, prob, param_list[9+k],n.attrib)

        #Set tau and sigma for all vTypes
        for vType in root.findall('vType'):
            vType.set('sigma',str(param_list[4]))
            vType.set('tau',str(param_list[5]))

        root.write('./draft.rou.xml')

    #Update the traffic lights
    with open(os.path.join(os.path.dirname(__file__), './draft.net.xml')) as f:
        root = ET.parse(f)
        for i, phase in enumerate(root.find('tlLogic')):
            #print(phase.attrib, param_list[i], get_props_from_total(1,param_list[0:4],param_list[i]))
            phase.set('duration', get_props_from_total(1,param_list[0:4],param_list[i]))
        root.write('./draft.net.xml')

    return True

# this is the main entry point of this script
if __name__ == "__main__":

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run

    if  NO_SIM: go()
    update_network('intersection',[])
    go()
    traci.close()
