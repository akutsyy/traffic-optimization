
from sumolib import checkBinary  # noqa
import traci
import pandas as pd


session = False
sumoBinary = None


def get_sum_stats():
    df = pd.read_xml('sum.xml')
    print(df)



def go():
    cmd = [sumoBinary, "-c", "inter1.sumocfg", "--start", "--quit-on-end",
           "--summary", "sum.xml", "--tripinfo-output", "tripinfo.xml"]
    if not session:
        traci.start(cmd)
    else:
        traci.load(cmd.remove('--quit-on-end'))
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

    get_sum_stats()
    traci.close()

#total is total traffic flow on edge, ratios is ratio of proportions
#e.g. total flow of 10 and truck:car:bike ratio of 5:3:2 = 5 trucks, 3 cars, 2 bikes
def get_props_from_total(total, ratios):
    multiplier = total/sum(ratios)
    return list(map(ratios,lambda x: x*multiplier))


def update_network(network_type, param_list):
    if network_type == 'intersection':
        #W->E:
        print()
        #Trucks-to-cars-ratio

        #

        #E->W:

        #N->S:

        #S->N:

    else:
        raise NotImplementedError('That intersection type does not exist! Try one of [intersection]')



# this is the main entry point of this script
if __name__ == "__main__":

    NO_GUI = False
    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if NO_GUI:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')
    go()
