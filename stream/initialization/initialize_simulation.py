# IMPORTS
import copy
import numpy as np

from .validate_and_complete_scenario import update_link_DF


def initialize_simulation(Simulation):  # , User):
    print("Original initialization...")
    '''Main function'''
    # ...
    #Links, Nodes, VehicleClass, General, Entries, Exits, Routes, Periods = inputs_to_variables(Inputs)
    # ...
    # ---- (1) Initialisation of the dynamic variables of nodes and links
    Simulation["Events"] = initial_Events(
        Simulation["Nodes"], Simulation["Links"])
    # ...
    # ---- (2) Initialisation of the arrivals at entries
    Simulation["Events"] = initial_arrivals(
        Simulation["Entries"], Simulation["Events"], Simulation["tmp"]["vehArray"])
    # ...
    # --- (3) Initialization of the actualization times in the simulation
    Simulation["General"] = complete_general(Simulation["General"])
    # ...
    # ---- (4) Initialization of regulations
    Simulation = initialize_regulations(Simulation)
    # ...
    # ---- (5) Add action
    Simulation = initialize_actions(Simulation)
    # ...
    del Simulation["tmp"]
    return (Simulation)


def initial_Events(Nodes, Links):
    '''initialise the structure for Events'''
    Events = {}
    # ...
    for node in list(Nodes.keys()):
        # init the Arrivals
        Arrivals = {}
        for j in range(Nodes[node]["NumIncomingLinks"] + 1):
            # Arrivals on the nodes
            Arrival = {}
            Arrival.update({"Time": np.zeros(0)})
            Arrival.update({"VehID": np.zeros(0)})
            Arrival.update({"IsExit": np.zeros(0)})
            Arrival.update({"NextLinkID": np.zeros(0)})
            Arrival.update({"Num": 0})
            Arrivals.update({j: Arrival})
        Events.update({node: {"Arrivals": Arrivals}})
        # ...
        # Initialization of Exits
        Exits = {}
        for j in range(Nodes[node]["NumOutgoingLinks"] + 1):
            # Sorties du noeud
            Exit = {}
            Exit.update({"Time": np.zeros(0)})
            Exit.update({"Regime": np.zeros(0)})
            Exit.update({"VehID": np.zeros(0)})
            Exit.update({"PreviousLinkID": np.zeros(0)})
            Exit.update({"Num": 0})
            Exits.update({j: Exit})
        Events[node].update({"Exits": Exits})
        # ...
        # ---- Initial initial_node_supply_time ---
        if len(Nodes[node]["OutgoingLinksID"]) == 0:
            VehiclesNum = 0
        else:
            VehiclesNum = 1 + np.floor(np.max(np.array([Links[link]["Length"] for link in Nodes[node]["OutgoingLinksID"]]) * np.array(
                [Links[link]["NumLanes"] for link in Nodes[node]["OutgoingLinksID"]]) * [Links[link]["FD"]["kx"] for link in Nodes[node]["OutgoingLinksID"]]))
        VehiclesNum = max([VehiclesNum, 1])
        OutgoingLinksNum = 1 + Nodes[node]["NumOutgoingLinks"]
        IncomingLinksNum = 1 + Nodes[node]["NumIncomingLinks"]
        SupplyTimes = {}
        SupplyTimes["Downstream"] = -np.inf * \
            np.ones([int(VehiclesNum), int(OutgoingLinksNum)])
        SupplyTimes["DownCapacity"] = -np.inf * np.ones(int(OutgoingLinksNum))
        SupplyTimes["UpCapacity"] = -np.inf * np.ones(int(IncomingLinksNum))
        Events[node].update({"SupplyTimes": SupplyTimes})
    # ...
    return Events


def initial_arrivals(Entries, Events, vehArray):
    '''initialise arrivals at entries in the Events structure'''
    if len(vehArray) != 0:
        # ...
        for entry in list(Entries):
            subArray = vehArray[np.where(vehArray[:, 1] == entry)[0], :]
            # sorting
            subArray = subArray[subArray[:, 2].argsort()]
            # ...
            Events[entry]["Arrivals"][0]["Time"] = subArray[:, 2]
            Events[entry]["Arrivals"][0]["VehID"] = subArray[:, 0]
            Events[entry]["Arrivals"][0]["IsExit"] = np.zeros(
                subArray.shape[0])
            # np.zeros(subArray.shape[0])
            Events[entry]["Arrivals"][0]["NextLinkID"] = subArray[:, 3]
        # ...
    return Events


def complete_general(General):
    '''complete General dictionary to integrate the save of computational times'''
    # ...
    Computation = {}
    Computation.update({"NumEvent": 0,
                        "NodeEvent": np.array([]),
                        "CurrentSimulationTime": np.array([]),
                        "Time": np.array([]),
                        })
    General.update({"Computation": Computation})
    # ...
    return General


def initialize_regulations(Simulation):
    '''account for regulations and modify Simulation dictioaries'''
    # ...
    nextVehID = len(list(Simulation['Vehicles']))+1
    for reg in list(Simulation["Regulations"]):
        Regulation = Simulation["Regulations"][reg]
        # ...
        # Adapt the simulation with a managed Lane
        if Regulation['Type'] == 'managed_lane':
            # ...
            # IF "Links_HOL" is not set in the regulation dict.
            if 'Links_HOL' not in Regulation['Args']:
                print('Applying change to network to adapt to managed_lane')
                for managedLaneLink in Regulation['Args']['Links']:
                    # Creation of a new link
                    newLinkID = max(list(Simulation["Links"]))+1
                    newLink = copy.deepcopy(
                        Simulation["Links"][managedLaneLink])
                    newLink["NumLanes"] = 1
                    Simulation["Links"].update({newLinkID: newLink})
                    if "Capacity" in list(Regulation['Args'].keys()):
                        newLink["Capacity"] = Regulation['Args']['Capacity']
                        update_link_DF(Simulation["Links"], newLinkID)
                    else:
                        Simulation["Links"][newLinkID]["Capacity"] = Simulation["Links"][managedLaneLink]["FD"]["C"]
                    # ...
                    # Modify the existing link
                    NumLanes = Simulation["Links"][managedLaneLink]['NumLanes']
                    ratio1 = (NumLanes-1)/NumLanes
                    LaneProbability = [ratio1, 1-ratio1]
                    LaneProbabilities = [LaneProbability for vehclass in list(
                        Simulation['VehicleClass'])]
                    # Capacity
                    if "Capacity" in list(Regulation['Args'].keys()):
                        cap = NumLanes * \
                            Simulation["Links"][managedLaneLink]["FD"]["C"] - \
                            Regulation['Args']['Capacity']
                    else:
                        cap = (NumLanes-1) * \
                            Simulation["Links"][managedLaneLink]["FD"]["C"]
                    Simulation["Links"][managedLaneLink].update({
                        'AssociatedLink': newLinkID,
                        'LaneProbabilities': LaneProbabilities,
                        'NumLanes': (NumLanes-1),
                        'Capacity': cap
                    })
                    update_link_DF(Simulation["Links"], managedLaneLink)
                    # ...
                    # Modify the nodes
                    nodeup = Simulation["Links"][managedLaneLink]["NodeUpID"]
                    Simulation["Nodes"][nodeup]["OutgoingLinksID"] = np.concatenate((
                        Simulation["Nodes"][nodeup]["OutgoingLinksID"], np.array([newLinkID])))
                    Simulation["Nodes"][nodeup]["NumOutgoingLinks"] += 1
                    # ...
                    nodedown = Simulation["Links"][managedLaneLink]["NodeDownID"]
                    Simulation["Nodes"][nodedown]["IncomingLinksID"] = np.concatenate((
                        Simulation["Nodes"][nodedown]["IncomingLinksID"], np.array([newLinkID])))
                    Simulation["Nodes"][nodedown]["NumIncomingLinks"] += 1
                    Simulation["Nodes"][nodedown]["CapacityDrop"] = np.array([0.] *
                                                                             (Simulation["Nodes"][nodedown]["NumIncomingLinks"] + 1))
                    Simulation["Nodes"][nodedown].update(recalculateAlphaOD(
                        Simulation["Nodes"][nodedown], Simulation["Links"]))
            # ...
            # if "Links_HOL" is not set in the regulation dict
            else:
                for index, managedLaneLinkID in enumerate(Regulation['Args']['Links']):
                    # ...
                    # Modifying the link
                    # 'AssociatedLink'
                    associated_link_id = Regulation['Args']['Links_HOL'][index]
                    associated_link = Simulation['Links'][associated_link_id]
                    # 'LaneProbabilities' : initialisation same probability for all vehicles
                    managed_link = Simulation['Links'][managedLaneLinkID]
                    ratio_nb_lanes = managed_link['NumLanes'] / (
                        managed_link['NumLanes'] + associated_link['NumLanes'])
                    lane_probability = [ratio_nb_lanes, 1 - ratio_nb_lanes]
                    lane_probabilities = [
                        lane_probability for vehclass in list(Simulation['VehicleClass'])]
                    # ...
                    # Update the link
                    Simulation["Links"][managedLaneLinkID].update({
                        'AssociatedLink': associated_link_id,
                        'LaneProbabilities': lane_probabilities,
                    })

        # ...
        # Adapt the simulation with a dynamic_speed_adaptation
        if Regulation['Type'] == 'dynamic_speed_adaptation':
            pass
        
        if False:
            if Regulation['Type'] == 'demand_variation_path':
                path = Regulation['Args']['Links']
                nodelist = [Simulation['Links'][linkid]['NodeUpID'] for linkid in path]
                exitid = Simulation['Links'][path[len(path)-1]]['NodeDownID']
                nodelist.append(exitid)
                Times = Regulation['Args']['Times']
                Flow = Regulation['Args']['Flow']
                for k,flow in enumerate(Flow):
                    time0,time1 = Times[k],Times[k+1]
                    time01 = np.linspace(time0,time1,nbveh)
                    for arrivaltime in time01:
                        for vehclass in VehicleClass:
                            for k in range(nbveh[vehclass]):
                                vehicle = { 'EntryID' : nodelist[0], 'ExitID' : exitid, 'VehicleClass' : vehclass, 
                                       'NetworkArrivalTime' : arrivaltime, 'IDRoute': 0, 'Path' : path, 'NodeList' : nodelist,
                                       'CurrentNode' : -1, 'RealPath' : []}
                            
                                Simulation['Vehicles'][nextVehID] = vehicle
                            
                            
                                nextVehID = nextVehID +1
        # ...
    return Simulation


def initialize_actions(Simulation):
    '''initialize Action dictionary in Simulation'''
    # ...
    # initialize if necessary
    if not "Actions" in list(Simulation):
        Simulation["Actions"] = []
    Actions = []
    SimulationDuration = Simulation['General']['SimulationDuration']
    [begin_simu,end_simu] = SimulationDuration 

    # ...
    
    #Init dictionnary link2exitnodes
    link2exitnodes = {}
    for exitnode in Simulation['Exits']:
        Links = Simulation['Exits'][exitnode]['IncomingLinksID']
        for link in Links:
            link2exitnodes[link] = exitnode
    #...
    next_signal_id = max(list(Simulation['Signals']))
    
    
    # Display times
    step_time = Simulation["General"]["TimesStepByDefault"]  # sec
    if step_time != None and step_time > 0:
        display_times = np.arange(Simulation["General"]["SimulationDuration"][0],
                                  Simulation["General"]["SimulationDuration"][1] + 2 * step_time, step_time)
        display_times = display_times - \
            (Simulation["General"]["SimulationDuration"][0] % step_time)
        for disp_time in display_times:
            # ...
            Action = {}
            Action['Time'] = disp_time
            Action['Type'] = 'display_time_simulation'
            Action['Args'] = {}
            Actions.append(Action)
    # ...
    # Loop for all the regulations
    for reg in list(Simulation["Regulations"]):
        Regulation = Simulation["Regulations"][reg]
        # ...
        # Managed lanes
        if Regulation['Type'] == 'managed_lane':
            for managedLaneLink in Regulation['Args']['Links']:
                activated = False
                for time in Regulation['Args']['Times']:
                    # ...
                    Action = {}
                    Action['Time'] = time
                    if activated:
                        Action['Type'] = 'managed_lane_deactivation'
                        activated = False
                    else:
                        Action['Type'] = 'managed_lane_activation'
                        activated = True
                    Action['Args'] = {
                        'LinkID': managedLaneLink, 'Class': Regulation['Args']['Class'], 'Display': True}
                    Actions.append(Action)
        # ...
        # Speed Limit
        if Regulation['Type'] == 'speed_limit':
            for concerned_link in Regulation['Args']['Links']:
                # ...
                base_speed = Simulation['Links'][concerned_link]['Speed']
                base_capacity = Simulation['Links'][concerned_link]['Capacity']
                for timeframe in Regulation['Args']['timeframes']:
                    # ...
                    # Limit the actions to the SimulationDuration range
                    if timeframe['start'] >= Simulation['General']['SimulationDuration'][1]:
                        continue
                    # ...
                    # new Speed and new Capacity preparation
                    new_speed = base_speed
                    if timeframe['parameters']['speed']:
                        new_speed = timeframe['parameters']['speed']
                    new_capacity = base_capacity
                    if timeframe['parameters']['increase_capacity']:
                        new_capacity = base_capacity * \
                            (1+timeframe['parameters']['increase_capacity'])
                    # ...
                    # Action creation
                    Action = {}
                    Action['Time'] = timeframe['start']
                    Action['Type'] = 'speed_limit'
                    Action['Args'] = {
                        'LinkID': concerned_link,
                        'Speed': new_speed,
                        'Capacity': new_capacity,
                        'Display': True
                    }
                    Actions.append(Action)
        # ...
        # Custom
        if Regulation['Type'] == 'custom':
            function_to_call = Regulation['Args']['FunctionToCall']
            for time in Regulation['Args']['Times']:
                Actions.append({
                    'Time': time,
                    'Type': 'custom',
                    'Args': {
                        'FunctionToCall': function_to_call
                    }
                })

        #exit supply                
        if Regulation['Type'] == 'exit_supply':
            ExitCapacity  = [parameter['exit_capacity'] for parameter in Regulation['Args']['Parameters']]
            Times = Regulation['Args']['Times']
            if Times[0] is None:
                Times[0] = begin_simu
            if len(Times) > 1:
                if Times[len(Times)-1] is None:
                    Times[len(Times)-1] = end_simu
            else:
                Times.append(end_simu)
            Links = Regulation['Args']['Links']
            Times,ExitCapacity = addapt_vector_to_simu(begin_simu,end_simu,Times,ExitCapacity)
            #
            for link in Links:
                try : 
                    exitnode = link2exitnodes[link]
                except:
                    exitnode = None
                    print(" 'exit_node' doesn't exist for linkid : ", link)
                if exitnode is not None:
                    current_times,current_data = Simulation['Exits'][exitnode]['Supply']['Time'],Simulation['Exits'][exitnode]['Supply']['Data']
                    for k in range(len(ExitCapacity)):
                        begin,end = Times[k],Times[k+1]
                        data = ExitCapacity[k]
                        if data is None:
                            data = np.inf
                        current_times,current_data = add_single_element_to_sorted_data(begin,end,data,current_times,current_data)
                    Simulation['Exits'][exitnode]['Supply']['Time'] = current_times
                    Simulation['Exits'][exitnode]['Supply']['Data'] = current_data
            # ...

        #ramp_metering 
        if Regulation['Type'] == 'ramp_metering':
            #en aval du lien -> on prend le NodeDownID, et on récupère ses IncomingLinksID
            Links = Regulation['Args']['Links']
            Times = Regulation['Args']['Times']
            signalcapacities  = [parameter['signal_capacity'] for parameter in Regulation['Args']['Parameters']]
            #for each ramp metering
            for k,signalcapacity in enumerate(signalcapacities):
                if signalcapacity is not None:
                    cycle_time = 1/signalcapacity
                    green = min(0.5,cycle_time/2)
                    duration = [Times[k],Times[k+1]]
                    Simulation['Signals'][next_signal_id] = makeSignalTimes(green, cycle_time-green, duration)        
                    print('Le signal ', Simulation['Signals'][next_signal_id], ' de ID ', next_signal_id, ' à été ajouté' )
                    
                    #tackle all concerned link
                    for link in Links:
                        nodeid = Simulation['Links'][link]['NodeDownID']
                        signals = Simulation['Nodes'][nodeid]['SignalsID']
                        incominglinksid = Simulation['Nodes'][nodeid]['IncomingLinksID']
                        pos = np.where(incominglinksid == link)[0][0]
                        while len(signals) <= pos:
                            signals.append(None)
                        signals[pos] = next_signal_id                    
                        print('Affectation du signal ', next_signal_id, ' sur le lien ', link , ' qui correspond', \
                              ' au nodeid ',nodeid, ' en position ', pos)

        
                    next_signal_id = next_signal_id + 1
                    # ...
                            
        if Regulation['Type'] == 'speed_limit_neovya':
            Speed = [parameter['speed'] for parameter in Regulation['Args']['Parameters']]
            IncreaseCapacity = [parameter['increase_capacity'] for parameter in Regulation['Args']['Parameters']]
            Times = Regulation['Args']['Times']
            Links = Regulation['Args']['Links']
            useless_times,Speed = addapt_vector_to_simu(begin_simu,end_simu,Times,Speed)
            Times,IncreaseCapacity = addapt_vector_to_simu(begin_simu,end_simu,Times,Speed)
            
            for link in Links:
                for k,(speed,increasecapacity) in enumerate(zip(Speed,IncreaseCapacity)):
                    if speed is None:
                        speed = np.inf
                    if increasecapacity is None:
                        increasecapacity = 0
                    Actions.append({'LinkID': link, 'Times' : [Times[k],Times[k+1]], 'Status' : 'Begin',
                        'Time': Times[k],
                        'Type': 'speed_limit_neovya',
                        'Args': {
                            'Speed': speed,
                            'Increase_capacity' : increasecapacity
                        }
                    })  
                    Actions.append({'LinkID': link, 'Times' : [Times[k],Times[k+1]],'Status' : 'End',
                        'Time': Times[k+1],
                        'Type': 'speed_limit_neovya',
                        'Args': {
                            'Speed': speed,
                            'Increase_capacity' : increasecapacity
                        }
                    }) 
            # ...
             
            
        if Regulation['Type'] == 'lane_reduction' or Regulation['Type'] == 'crash' :
            remaining_lanes = Regulation['Args']['remaining_lanes']
            new_time = Regulation['Args']['Times']
            links = Regulation['Args']['Links']
            for link in links:
                Actions.append({'LinkID' : link, 
                                'Times' : new_time, 
                                'Status' : 'Begin',
                                'Time': new_time[0],
                                'Type': 'lane_reduction',
                                'Args':{'remaining_lane': remaining_lanes,                      
                                }})     
                Actions.append({'LinkID' : link, 
                                'Times' : new_time, 
                                'Status' : 'End',
                                'Time': new_time[1],
                                'Type': 'lane_reduction',
                                'Args':{
                                'remaining_lane': remaining_lanes, 
                                }})
            # ...
                      
        if Regulation['Type'] == 'variable_free_speed':
            speed_drop_at_capacity = Regulation['Args']['speed_drop_at_capacity']
            Times = SimulationDuration
            Links = list(Simulation['Links'])
            for link in Links:
                Actions.append({'LinkID' : link,'Times' : SimulationDuration,
                    'Time': Times[0],
                    'Type': 'variable_free_speed',
                    'speed_drop_at_capacity': speed_drop_at_capacity
                    })               
            # ...
            
        if Regulation['Type'] == 'storm': 
            Times = SimulationDuration
            Actions.append({'Links' : list(Simulation['Links']),
                'Times' : SimulationDuration,
                'Time': Times[0],
                'Type': 'storm',
                'Args': Regulation['Args']
            })                      
            # ...    
            
    # ...
    # Sort actions
    if Actions != []:
        Actions = sortActionsByTime(Actions)
    Simulation['Actions'] = Actions
    # ...
    return Simulation


# =============================================================================
# Sub-functions
# =============================================================================
def init_action_by_link(links,period):
    actions_by_links = {}    
    for link in links:
        actions_by_links[link] = ([np.inf,np.inf],period)
    return actions_by_links

def addapt_vector_to_simu(begin,end,Times,Carac):
    new_times = []
    new_carac = []
    for k in range(len(Times)):
        if Times[k] is None:
            Times[k] = begin
        if Times[k] >= begin and Times[k] < end:
            new_times.append(Times[k])
            new_carac.append(Carac[k])
    new_times.append(end)
    return(new_times,new_carac)

def add_single_element_to_sorted_data(begin,end,data,list_time,list_data):
    ''' list_time, list_data = [t0,..,tn],[d0,..,dn]  -> [t0,..,ti-1, begin, ti,..tj-1, end, tj,..,tn]'''
    i,j = get_pos_in_list(begin,end,list_time)
    list_time = list(list_time)
    list_data = list(list_data)
    new_list_time = list_time[:i]
    new_list_data = list_data[:i]
    
    #begin
    if i < len(list_time):
        if begin == list_time[i]:
            new_data =  min(data,list_data[i])
            i = i+1
        else:
            new_data = min(data,list_data[i-1])
        new_list_time.append(begin)
        new_list_data.append(new_data)    
    #...
    
    #general case
    for k in range(i,j-1):
        new_data = min(data,list_data[k])
        new_list_time.append(list_time[k])
        new_list_data.append(new_data)
        #...
    
    #end
    if j < len(list_time):
        if end == list_time[j]:
            new_data = list_data[j]
            j = j+1
        else:
            new_data = list_data[j-1]
        new_list_time.append(end)
        new_list_data.append(new_data)
    
    new_list_data = new_list_data + list_data[j:] 
    new_list_time = new_list_time + list_time[j:]
        # ...
    
    #delete duplicate
    keep_index = []
    n = len(new_list_data)
    k = 0
    p = 0
    while k<n :
        while p < n and new_list_data[k] == new_list_data[p]:
            p = p +1
        keep_index.append(k)
        k = p
    keep_index.append(n-1)
    new_list_time_bis,new_list_data_bis = [],[]
    for k in keep_index:
        new_list_time_bis.append(new_list_time[k])
        new_list_data_bis.append(new_list_data[k])
    # ...
        
    return(new_list_time_bis,new_list_data_bis)
 
def get_pos_in_list(begin,end,list_time):
    i = 0
    n = len(list_time)
    while i<n and begin > list_time[i]:
        i = i +1
    j = i
    while j<n and end > list_time[j]:
        j = j+1
    return(i,j)


def recalculateAlphaOD(node, Links):
    node.update({"AlphaOD": np.array([])})
    if node["NumIncomingLinks"] >= 1:  # this is a merge
        NumIncomingLanes = [Links[link]["NumLanes"]
                            for link in node["IncomingLinksID"]]
        classic = 1/sum(NumIncomingLanes) * np.array(NumIncomingLanes)
        node.update({"AlphaOD": np.hstack((classic, np.array([0])))})
    if node["NumIncomingLinks"] == 0:
        node["AlphaOD"] = np.array([1.])
    return node


def sortActionsByTime(Actions):
    actionsArray = np.zeros(len(Actions))
    for i, action in enumerate(Actions):
        actionsArray[i] = action["Time"]
        sortedIndexes = np.argsort(actionsArray)
    # ...
    # sorting
    newActions = []
    for i in range(len(sortedIndexes)):
        newActions.append(Actions[sortedIndexes[i]])
    Actions = newActions
    return Actions



def makeSignalTimes(green_time, red_time, duration, startGreen=True):
    step = green_time+red_time
    if startGreen:
        green_starts = np.arange(
            start=duration[0], stop=duration[1] + step, step=green_time+red_time)
        red_starts = np.arange(
            start=duration[0]+green_time, stop=duration[1] + step, step=green_time+red_time)
    else:
        red_starts = np.arange(
            start=duration[0], stop=duration[1] + step, step=green_time+red_time)
        green_starts = np.arange(
            start=duration[0]+red_time, stop=duration[1] + step, step=green_time+red_time)

    red_starts = np.append(red_starts, np.inf)
    green_starts = np.append(green_starts, np.inf)
    return {'green_starts': green_starts, 'red_starts': red_starts}
