import os
os.environ['NUMPY_EXPERIMENTAL_ARRAY_FUNCTION'] = '0'
import numpy as np

import random


def main_simulation_meso(S, T, disp='time'):
    print("Original simulation loop...")
# S is the dict containing all the information about the simulation
# nProcess is the number of the calculated process
# T is the instant of end of the simulation
    dic_action_in_progress = init_dic_action(S)
        
    # ---- (1) Reinitialization of the event list, including the next
    if 'Signals' in S.keys():
        Signals = S['Signals']
    else:
        Signals = None
    ListEvents, NextEvent = compute_next_event(S['General'],
                                               S['Links'],
                                               S['Nodes'],
                                               S['Actions'],
                                               S['Events'],
                                               Signals)

    ListActions, NextAction = compute_next_action(S['Actions'])
    # NextAction = {'Time': S["General"]["SimulationDuration"][1] + 100}

    # ---- (2) Mesoscopic Simulation
    while min(NextEvent['Time'], NextAction['Time']) < T:
        # While their is still events on the network

        if NextEvent['Time'] < NextAction['Time']:
            # Faire comme d'hab
            # (modifier list et next)
            ListEvents, NextEvent = tackle_vehicle_event(
                S, ListEvents, NextEvent)

        else:
            ListActions, NextAction,dic_action_in_progress = tackle_action(
                S, ListActions, NextAction, dic_action_in_progress, disp=disp)

    return S


# =============================================================================
# --------------------- Sub-functions of action computation -------------------
# =============================================================================


def compute_next_action(Actions, ListActions=None, NextAction=None):
    # ...
    if ListActions == None and NextAction == None:
        # Initialisation of Event in main_simulation_meso*
        if len(Actions) != 0:
            ListActions = Actions
            NextAction = ListActions[0]
            ListActions = ListActions[1:]
        else:
            ListActions = []
            NextAction = {'Time': np.inf}
        # ...
    elif len(ListActions) == 0:
        NextAction = {'Time': np.inf}
        # ...
    else:
        # ...
        NextAction = ListActions[0]
        ListActions = ListActions[1:]
    # ...
    return (ListActions, NextAction)


def tackle_action(S, ListActions, NextAction, dic_action_in_progress, disp='time'):
    # ...
    # we deals with actions
    # ...
    NextVehID = len(list(S['Vehicles']))
    if NextAction['Type'] == 'display_time_simulation':
        if disp == 'time':
            print('Time in simulation : ' +
                  str(int(NextAction['Time'])) + ' sec')
        elif disp == '%':
            progress = max(0, 100 * (NextAction['Time'] - S['General']['SimulationDuration'][0]) /
                           (S['General']['SimulationDuration'][1] - S['General']['SimulationDuration'][0]))
            print('%%%'+str(progress))

    # ...
    if NextAction['Type'] == 'managed_lane_activation':
        # First Link = principal
        # Second Link = reserved Lane
        firstLinkID = NextAction['Args']['LinkID']
        classID = NextAction['Args']['Class']
        secondLinkID = S['Links'][firstLinkID]["AssociatedLink"]
        numLanesFirst = S['Links'][firstLinkID]["NumLanes"]
        numLanesSecond = S['Links'][secondLinkID]["NumLanes"]
        numLanesTotal = numLanesFirst + +numLanesSecond
        values = []
        for i in range(len(S["VehicleClass"])):
            if i == classID:
                values.append([0, 1])
            else:
                values.append([1, 0])
        if NextAction["Args"]["Display"]:
            print(f"@Stream[{NextAction['Time']}]: modifying the link {firstLinkID} to activate the managed lane link {secondLinkID} for the class {S['VehicleClass'][classID]['Name']}")
            print("Links sharing values by class :")
            print(f".\t\t{firstLinkID}\t\t{secondLinkID}")
            for index, classID in enumerate(S['VehicleClass'].keys()):
                print(
                    f"{S['VehicleClass'][classID]['Name']}\t\t{values[index][0]:0.2f}\t\t{values[index][1]:0.2f}")
        S["Links"][firstLinkID]["LaneProbabilities"] = values

    # ...
    if NextAction['Type'] == 'managed_lane_deactivation':
        # First Link = principal
        # Second Link = reserved Lane
        firstLinkID = NextAction['Args']['LinkID']
        classID = NextAction['Args']['Class']
        secondLinkID = S['Links'][firstLinkID]["AssociatedLink"]

        numLanesFirst = S['Links'][firstLinkID]["NumLanes"]
        numLanesSecond = S['Links'][secondLinkID]["NumLanes"]
        numLanesTotal = numLanesFirst + numLanesSecond

        values = []
        for i in range(len(S["VehicleClass"])):
            values.append([numLanesFirst/numLanesTotal,
                           1. - numLanesFirst/numLanesTotal])
        S["Links"][firstLinkID]["LaneProbabilities"] = values
        # ...
        if NextAction["Args"]["Display"]:
            print(f"@Stream[{NextAction['Time']}]: modifying the link {firstLinkID} to deactivate the managed lane link {secondLinkID} for the class {S['VehicleClass'][classID]['Name']}")
            print("Links sharing values by class :")
            print(f".\t\t{firstLinkID}\t\t{secondLinkID}")
            for index, classID in enumerate(S['VehicleClass'].keys()):
                print(
                    f"{S['VehicleClass'][classID]['Name']}\t\t{values[index][0]:0.2f}\t\t{values[index][1]:0.2f}")

    # ...
    if NextAction['Type'] == 'speed_limit':
        Args = NextAction['Args']
        concerned_link = Args['LinkID']
        # ...
        # Keep track of changes
        if 'Changes' not in S['Links'][concerned_link]:
            S['Links'][concerned_link]['Changes'] = {
                'Speed' : {'Times' : [], 'Values' : []},
                'Capacity' : {'Times' : [], 'Values' : []}
            }
        S['Links'][concerned_link]['Speed'] = Args['Speed']
        S['Links'][concerned_link]['Capacity'] = Args['Capacity']
        S['Links'][concerned_link]['Changes']['Speed']['Times'].append(NextAction['Time'])
        S['Links'][concerned_link]['Changes']['Capacity']['Times'].append(NextAction['Time'])
        S['Links'][concerned_link]['Changes']['Speed']['Values'].append(Args['Speed'])
        S['Links'][concerned_link]['Changes']['Capacity']['Values'].append(Args['Capacity'])
        # ...
        if NextAction['Args']['Display']:
            print(
                f"@Stream[{NextAction['Time']}] : adpatating the speed for link {concerned_link} : Speed {Args['Speed']} and Capacity {Args['Capacity']}")
    # ...
    if NextAction['Type'] == 'custom':
        func = NextAction['Args']['FunctionToCall']
        print(
            f"@Stream: [{NextAction['Time']}], custom regulation calling function {func.__name__}")
        func(S)       
         # ...
         
    if NextAction['Type'] == 'speed_limit_neovya':
         #speed limit en amont ou aval du lien, au choix  -> je choisi en Amont
        #increase capacity en amont et en aval du lien.
        concerned_link = NextAction['LinkID']
        try:
            Link = S['Links'][concerned_link]
        except:
            Link = None
            print('Link ',concerned_link, 'does not exist in the network')
        if Link is not None:
            nodeup,nodedown = S['Links'][concerned_link]['NodeUpID'],S['Links'][concerned_link]['NodeDownID']
            outgoinglinksid = S['Nodes'][nodeup]['OutgoingLinksID']
            incominglinksid = S['Nodes'][nodedown]['IncomingLinksID']
    
            IN = np.where(incominglinksid == concerned_link)[0][0]
            OUT = np.where(outgoinglinksid == concerned_link)[0][0]
            
            speed_action_in_progress = dic_action_in_progress[nodeup]['OutgoingLinksID'][OUT]['speed_limit']
            
            capacityUp_action_in_progress = dic_action_in_progress[nodeup]['OutgoingLinksID'][OUT]['Capacity']
            capacityDown_action_in_progress = dic_action_in_progress[nodedown]['IncomingLinksID'][IN]['Capacity']
            
            if NextAction['Status'] == 'Begin':
                speed_action_in_progress.append(NextAction)
                capacityUp_action_in_progress.append(NextAction)
                capacityDown_action_in_progress.append(NextAction)
            else:
                speed_action_in_progress = delete_action_from_list(NextAction,speed_action_in_progress)
                capacityUp_action_in_progress = delete_action_from_list(NextAction,capacityUp_action_in_progress)  
                capacityDown_action_in_progress = delete_action_from_list(NextAction,capacityDown_action_in_progress)  
               
            #tackle speedlimit
            if speed_action_in_progress == []:
                speedlimit = np.inf
            else:
                speedlimits = [action['Args']['Speed'] for action in speed_action_in_progress]
                speedlimit = min(speedlimits)
            S['Nodes'][nodeup]['Tmp_DownSpeedLimit'][OUT] = speedlimit
            
            #tackle increasecapacityUp
            if capacityUp_action_in_progress == []:
                capacityup = None
            else:
                capacityup = get_capacity(S,capacityUp_action_in_progress,concerned_link)
            S['Nodes'][nodeup]["Tmp_DownCapacity"][OUT] = capacityup
            
            # ...      
            
            #tackle increasecapacityDown      
            if capacityDown_action_in_progress == []:
                capacitydown = None
            else:
                capacitydown = get_capacity(S,capacityDown_action_in_progress,concerned_link)
            S['Nodes'][nodedown]["Tmp_UpCapacity"][IN] = capacitydown
             # ...
            
    if NextAction['Type'] == 'lane_reduction':
        concerned_link = NextAction['LinkID']
        try:
            Link = S['Links'][concerned_link]
        except:
            Link = None
            print('Link ',concerned_link, 'does not exist in the network')
        if Link is not None:
            nodeup,nodedown = S['Links'][concerned_link]['NodeUpID'],S['Links'][concerned_link]['NodeDownID']
            outgoinglinksid = S['Nodes'][nodeup]['OutgoingLinksID']
            incominglinksid = S['Nodes'][nodedown]['IncomingLinksID']
            IN = np.where(incominglinksid == concerned_link)[0][0]
            OUT = np.where(outgoinglinksid == concerned_link)[0][0]
                    
            capacityUp_action_in_progress = dic_action_in_progress[nodeup]['OutgoingLinksID'][OUT]['Capacity']
            capacityDown_action_in_progress = dic_action_in_progress[nodedown]['IncomingLinksID'][IN]['Capacity']
            
            if NextAction['Status'] == 'Begin':
                capacityUp_action_in_progress.append(NextAction)
                capacityDown_action_in_progress.append(NextAction)
            else:
                capacityUp_action_in_progress = delete_action_from_list(NextAction,capacityUp_action_in_progress)  
                capacityDown_action_in_progress = delete_action_from_list(NextAction,capacityDown_action_in_progress)  
               
            #tackle increasecapacityUp
            if capacityUp_action_in_progress == []:
                capacityup = None
            else:
                capacityup = get_capacity(S,capacityUp_action_in_progress,concerned_link)
            S['Nodes'][nodeup]["Tmp_DownCapacity"][OUT] = capacityup
            # ...      
            
            #tackle increasecapacityDown      
            if capacityDown_action_in_progress == []:
                capacitydown = None
            else:
                capacitydown = get_capacity(S,capacityDown_action_in_progress,concerned_link)
            S['Nodes'][nodedown]["Tmp_UpCapacity"][IN] = capacitydown
             # ...

      
    if NextAction['Type'] == 'variable_free_speed':
        concerned_link = NextAction['LinkID']
        speed_drop_at_capacity  = NextAction['speed_drop_at_capacity']
        ''' J'APPLIQUE CA OU  ? '''    
        # ...
        
    if NextAction['Type'] == 'storm':
        speed_modulation = NextAction['Args']['Parameters']['speed_modulation']
        wavespeed_modulation = NextAction['Args']['Parameters']['wavespeed_modulation'] 
        capacity_modulation = NextAction['Args']['Parameters']['capacity_modulation']

        for linkid in S['Links']:
            S['Links'][linkid]['Ref_Capacity'] = S['Links'][linkid]['Capacity']*capacity_modulation
            S['Links'][linkid]['Ref_Speed'] = S['Links'][linkid]['Speed']*speed_modulation
            S['Links'][linkid]['Ref_Wavespeed'] = S['Links'][linkid]['FD']['w']*wavespeed_modulation
        #...
        
    # ...
    ListActions, NextAction = compute_next_action(
        S['Actions'], ListActions, NextAction)
    # ...
    return ListActions, NextAction,dic_action_in_progress


# =============================================================================
# --------------------- Sub-functions of compute_next_event -------------------
# =============================================================================
def tackle_vehicle_event(S, ListEvents, NextEvent):
    # --- Calculation of the event ---
    Event_NodeID, NodeID, Event_NodeDownID, NodeDownID, Event_NodeUpID, NodeUpID =\
        execute_and_update_event(S["Links"],
                                 S["Exits"],
                                 S["Nodes"],
                                 S["General"],
                                 S["Vehicles"],
                                 S["VehicleClass"],
                                 S["Events"],
                                 NextEvent)

    # --- Writing in the database ---
    # - (a) Event -
    if NodeID:
        S["Events"][NodeID] = Event_NodeID
    if NodeDownID:
        S["Events"][NodeDownID] = Event_NodeDownID
    if NodeUpID:
        S["Events"][NodeUpID] = Event_NodeUpID

    # - (b) Vehicle -
    S["Vehicles"][NextEvent["VehID"]
                  ]["CurrentNode"] = S["Vehicles"][NextEvent["VehID"]]["CurrentNode"] + 1
    S["Vehicles"][NextEvent["VehID"]]["NodeTimes"][S["Vehicles"]
                                                   [NextEvent["VehID"]]["CurrentNode"]] = NextEvent["Time"]
    if S["Nodes"][NextEvent["Node"]]["Type"] != 2:
        S["Vehicles"][NextEvent["VehID"]]["RealPath"].append(
            S["Nodes"][NextEvent["Node"]]["OutgoingLinksID"][NextEvent["NextLink"]])

    # - (c) Computation - DEGUB
    S["General"]["Computation"]["NumEvent"] = S["General"]["Computation"]["NumEvent"] + 1
    # Computational TIME... TODO
    # S["General"]["Computation"]["NodeEvent"] = np.concatenate(
    #     (S["General"]["Computation"]["NodeEvent"], np.array([NextEvent["Node"]])))

    # --- Update the list of events
    if 'Signals' in S.keys():
        Signals = S['Signals']
    else:
        Signals = None
    ListEvents, NextEvent = compute_next_event(S['General'],
                                               S['Links'],
                                               S['Nodes'],
                                               S['Actions'],
                                               S['Events'],
                                               Signals,
                                               ListEvents,
                                               NextEvent)

    return (ListEvents, NextEvent)


def compute_next_event(General, Links, Nodes, Actions, Events, Signals, ListEvents=None, NextEvent=None):

    # Update of ListEvents : Event time at current node and Nodes immediatly downstream have to be updated
    if not(ListEvents) and not(NextEvent):
        # Initialisation of Event in main_simulation_meso*
        NextEvent = {}
        NodesToUpdate = list(Events.keys())
        ListEvents = {}
    else:
        # Update of Events in main_simulation_meso
        NodeID = NextEvent["Node"]
        NodeDownID = [Links[linkID]["NodeDownID"]
                      for linkID in Nodes[NodeID]["OutgoingLinksID"]]  # Nodes Downstream
        NodeUpID = [Links[linkID]["NodeUpID"]
                    for linkID in Nodes[NodeID]["IncomingLinksID"]]  # Nodes Upstream
        _uptimes = np.array([ListEvents[nodeup]["Time"]
                             for nodeup in NodeUpID])
        if len(np.where(_uptimes == np.inf)[0]) > 0:
            NodesToUpdate = [NodeID] + NodeUpID + NodeDownID
        else:
            NodesToUpdate = [NodeID] + NodeDownID

    for n in NodesToUpdate:
        # ---- (1) Computation of the next arrival times at node ----
        NextArrivals = next_arrival_time(Nodes, Events, n)

        # ---- (2) Compuation of the next supply times at node ----
        NextSupplyTimes = next_supply_time(Events, n, General)

        # ---- (3) Computation of the next passage time at node ----
        NextPassageTime, NextRegime = next_passage_time(
            Nodes, n, NextArrivals, NextSupplyTimes, Signals)

        # ---- (4) Selection of the next event to compute ----
        NextPassage = select_next_event(
            Nodes, n, General, NextArrivals, NextPassageTime, NextRegime)

        # ---- (5) Update the next event at node ----
        ListEvents[n] = {}
        ListEvents[n]["VehID"] = int(NextPassage["VehID"])
        ListEvents[n]["Time"] = NextPassage["Time"]
        ListEvents[n]["NextTime"] = np.min(NextPassage["Time"])
        ListEvents[n]["Regime"] = int(NextPassage["Regime"])
        ListEvents[n]["PreviousLink"] = int(NextPassage["InIndex"])
        ListEvents[n]["NextLink"] = int(NextPassage["OutIndex"])

    # ---- (6) Identify the next event ----

    vect = [ListEvents[n]["NextTime"] for n in list(ListEvents.keys())]
    nodeind = np.argmin(vect)
    nodeid = list(ListEvents.keys())[nodeind]
    NextEvent["Node"] = nodeid
    NextEvent["VehID"] = ListEvents[NextEvent["Node"]]["VehID"]
    NextEvent["Regime"] = ListEvents[NextEvent["Node"]]["Regime"]
    NextEvent["Time"] = ListEvents[NextEvent["Node"]]["Time"]
    NextEvent["PreviousLink"] = ListEvents[NextEvent["Node"]]["PreviousLink"]
    NextEvent["NextLink"] = ListEvents[NextEvent["Node"]]["NextLink"]
    NextEvent["Type"] = "vehicle_event"

    return (ListEvents, NextEvent)


###############################
def next_arrival_time(Nodes, Events, NodeID):

    # Initialisation of variables
    NumIns = len(Events[NodeID]["Arrivals"])
    NumOuts = len(Events[NodeID]["Exits"])
    # ---
    NextArrivals = {}
    NextArrivals["VehID"] = np.zeros([NumIns, NumOuts])
    NextArrivals["Time"] = np.inf * np.ones([NumIns, NumOuts])
    NextArrivals["PreviousLinkIndex"] = np.zeros([NumIns, NumOuts])
    NextArrivals["LinkIndex"] = np.zeros([NumIns, NumOuts])

    # For each incoming links
    for IN in range(NumIns):

        if True:  # Nodes[nodeID]["FIFO"]:
            # We find the last vehicle that have reached the node NodeID
            iveh = np.where(
                np.array(Events[NodeID]["Arrivals"][IN]["IsExit"] == 0))[0]

            # FIFO assumption : we inspect the first arrived vehicle
            if len(iveh) > 0:
                iveh = iveh[0]
                # NextVehID
                NextArrivals["VehID"][IN,
                                      :] = Events[NodeID]["Arrivals"][IN]["VehID"][iveh] * np.ones((1, NumOuts))
                # Next Arrival Time
                NextArrivals["Time"][IN,
                                     :] = Events[NodeID]["Arrivals"][IN]["Time"][iveh] * np.ones((1, NumOuts))
                # Previous Link Index
                NextArrivals["PreviousLinkIndex"][IN,
                                                  :] = IN * np.ones((1, NumOuts))
                # Next Link Index
                NextArrivals["LinkIndex"][IN,
                                          :] = Events[NodeID]["Arrivals"][IN]["NextLinkID"][iveh] * np.ones((1, NumOuts))
        # else:

    return NextArrivals


def next_supply_time(Events, NodeID, General):
    NextSupplyTimes = {}
    # Supply time upstream
    # !!!! ATTENTION ! Le choix qui suit est déterminant dans les résultats !!!
    if General["ActiveUpStreamCapacity"]:  # Pour une visualitsation correcte en video
        NextSupplyTimes["Up"] = Events[NodeID]["SupplyTimes"]["UpCapacity"]
    else:
        NextSupplyTimes["Up"] = -np.inf * \
            np.ones(Events[NodeID]["SupplyTimes"]["UpCapacity"].shape)

    # Supply time downstream
    NumOuts = len(Events[NodeID]["Exits"])
    NextSupplyTimes["Down"] = -np.inf * np.ones(NumOuts)
    for out in range(NumOuts):
        # --- Capacity
        # Last passing time at the nodeID
        t1 = Events[NodeID]["SupplyTimes"]["DownCapacity"][out]

        # --- Downstream
        # index of the next vehicle at the node node ID
        n = Events[NodeID]["Exits"][out]["Num"]
        t2 = -np.inf
        if n < Events[NodeID]["SupplyTimes"]["Downstream"].shape[0]:
            t2 = Events[NodeID]["SupplyTimes"]["Downstream"][n, out]
        # SuppluTime
        NextSupplyTimes["Down"][out] = max(t1, t2)

    return NextSupplyTimes


def next_passage_time(Nodes, NodeID, NextArrivals, NextSupplyTimes, Signals):

    NumIns, NumOuts = NextArrivals["Time"].shape
    NextPassageTime = np.inf * np.ones([NumIns, NumOuts])
    NextRegime = np.zeros([NumIns, NumOuts])

    for IN in range(NumIns):
        for out in range(NumOuts):

            # --- maximum between demand and supplies (un and down)
            # effective out, independant from FIFO or not
            out_eff = int(NextArrivals["LinkIndex"][IN, out])
            NextPassageTime[IN, out] = max([NextArrivals["Time"][IN, out], NextSupplyTimes["Up"][IN], NextSupplyTimes["Down"][out_eff]])
            #np.max(np.hstack((np.array(
            #    NextArrivals["Time"][IN, out]), NextSupplyTimes["Up"][IN], NextSupplyTimes["Down"][out_eff])))

            # -- Effect of traffic signal
            # If signals has been set for this node
            # if the number of signals is equal to the number of entries AND
            # if the id of the signal corresponding to the in is more than 0 AND
            # if the current passage time is less than Infinity
            if Signals != None and 'SignalsID' in Nodes[NodeID].keys() \
            and len(Nodes[NodeID]['SignalsID']) >= IN and NextPassageTime[IN, out] < np.inf \
            and Nodes[NodeID]['SignalsID'][IN] is not None and Nodes[NodeID]['SignalsID'][IN] in list(Signals) : 
                PassageTime = NextPassageTime[IN, out]
                # Identification of the number of the traffic light strategy number
                SignalsID = Nodes[NodeID]['SignalsID'][IN]
                # Get the trafficlight cycle
                red_starts = Signals[SignalsID]['red_starts']
                green_starts = Signals[SignalsID]['green_starts']
                # Calcul des prochains temps de calcul au vert ou au rouge
                f_red = np.where(red_starts > PassageTime)[0][0]
                f_green = np.where(green_starts > PassageTime)[0][0]
                # if the next switching time is green, the light is now red
                if green_starts[f_green] < red_starts[f_red]:
                    PassageTime = green_starts[f_green]
                NextPassageTime[IN, out] = PassageTime

            # --- Regime
            if NextPassageTime[IN, out] > NextArrivals["Time"][IN, out]:
                NextRegime[IN, out] = 1  # 1 congestion, 0 free

    return (NextPassageTime, NextRegime)


def select_next_event(Nodes, NodeID, General, NextArrivals, NextPassageTime, NextRegime):

    # --- Next event time and potential candidats
    NumIns, NumOuts = NextArrivals["Time"].shape
    PassageTimeMinimum = np.min(NextPassageTime)
    inNext, outNext = np.where(NextPassageTime == PassageTimeMinimum)

    # --- Conservation of unique in-candidats
    _, in_selected = np.unique(inNext, return_index=True)
    inNext = inNext[in_selected]
    outNext = outNext[in_selected]

    if len(inNext) > 1:
        # there is a conflict
        if not(np.prod(1 - NextRegime[inNext, outNext]) == 0):
            # all vehicles are free : they only reaches the node in the same time
            # the vehicle from the most priority link passes
            priority = np.argmax(Nodes[NodeID]["AlphaOD"][inNext])
        elif np.prod(NextRegime[inNext, outNext]) == 0:
            # at least one vehicle is free : it passes
            priority = np.argwhere(NextRegime[inNext, outNext] == 0)[0][0]
        else:
            # All vehicles node are congested : the Daganzo coefficients apply
            if General["Stochasticity"]["Merge"]:
                DaganzoCoefficients = np.array(
                    Nodes[NodeID]["AlphaOD"][inNext])
                CumulCoeff = np.cumsum(DaganzoCoefficients)
                valuePriority = CumulCoeff[-1] * np.random.rand()
                priority = np.where(CumulCoeff >= valuePriority)[0][0]
            else:
                print("General Stochasticity Merge must be set to true.")
                print("The deterministic function is not implemented yet.")
        inNext = inNext[priority]
        outNext = outNext[priority]
    else:
        inNext = inNext[0]
        outNext = outNext[0]

    NextPassage = {}
    NextPassage["VehID"] = NextArrivals["VehID"][inNext, outNext]
    NextPassage["Time"] = NextPassageTime[inNext, outNext]
    NextPassage["Regime"] = NextRegime[inNext, outNext]
    NextPassage["InIndex"] = inNext
    NextPassage["OutIndex"] = NextArrivals["LinkIndex"][inNext, outNext]

    return NextPassage


# =============================================================================
# Function to modify the simulation database depending on the next event
# =============================================================================
def execute_and_update_event_bis(Links, Exits, Nodes, General, Vehicles, VehicleClass, Events, NextEvents): 
    # ---- (0) Initialisation 
    NodeID = NextEvents["Node"]
    current_time = NextEvents["Time"]
    out = NextEvents["NextLink"]
    IN = NextEvents["PreviousLink"]
    veh = NextEvents["VehID"]
    Event_NodeID = Events[NodeID]

    # ---- (1) Recording the current event
    # Record of the passage time in the varaible 'Exits' at the current node
    n = Event_NodeID["Exits"][out]["Num"] + 1
    Event_NodeID["Exits"][out]["Num"] = n
    Event_NodeID["Exits"][out]["VehID"] = np.concatenate(
        (Event_NodeID["Exits"][out]["VehID"], np.array([veh])))
    Event_NodeID["Exits"][out]["Time"] = np.concatenate(
        (Event_NodeID["Exits"][out]["Time"], np.array([current_time])))
    Event_NodeID["Exits"][out]["Regime"] = np.concatenate(
        (Event_NodeID["Exits"][out]["Regime"], np.array([NextEvents["Regime"]])))
    Event_NodeID["Exits"][out]["PreviousLinkID"] = np.concatenate(
        (Event_NodeID["Exits"][out]["PreviousLinkID"], np.array([IN])))
    # ---
    in_veh = np.where(Event_NodeID["Arrivals"][IN]["VehID"] == veh)[0]
    Event_NodeID["Arrivals"][IN]["IsExit"][in_veh] = True

    # ---- (2) New exit contraint at the current node
    # --- next time due to capacity downstream
    if out >= Nodes[NodeID]["NumOutgoingLinks"]:
        if Nodes[NodeID]["NumOutgoingLinks"] == 0:
            # Si le noeud est un noeud de sortie : la capacité est forcée pour la sortie
            h_down = 0
            ExitID = NodeID
            # Si une mise à jour de la capacité est nécessaire
            f = np.where(Exits[ExitID]["Supply"]["Time"] < current_time)[0]
            if len(f) > 0:
                f = f[-1]
                h_down = 1 / Exits[ExitID]["Supply"]["Data"][f]
        else:  # sortie d'un noeud interne (trançon pas défini)
            h_down = 0
    else: #traitement des noeuds internes 
        # If NodeID is an entry node or an internal node : the capacity
        # depends on the capacity of the downstream link
        Capacity = (1 - Nodes[NodeID]["CapacityDrop"][IN]) * \
            Links[Nodes[NodeID]["OutgoingLinksID"][out]]["Capacity"]
        h_down = 1/Capacity
        # s'il y a une valeur de Capacity Forced indique au noeud, alors elle s'applique
        if Nodes[NodeID]["CapacityForced"] < np.inf:
            h_down = 1/Nodes[NodeID]["CapacityForced"]
    h_down = h_down * General["Peloton"]
    Event_NodeID["SupplyTimes"]["DownCapacity"][out] = current_time + h_down

    # --- next time due to capacity upstream
    if IN > Nodes[NodeID]["NumIncomingLinks"] - 1:
        # Si NodeID est un noeud d'entrée
        h_up = 0
    else:
        h_up = 1 / Links[Nodes[NodeID]["IncomingLinksID"][IN]]["Capacity"]

    h_up = h_up * General["Peloton"]
    Event_NodeID["SupplyTimes"]["UpCapacity"][IN] = current_time + h_up

    # --- future constraint for the vehicle "veh+dn" at the node
    if out < Nodes[NodeID]["NumOutgoingLinks"]:
        # there is a downstream link: this vehicle "veh" is a constraint for
        # vehicle "veh + dn" at the current node, until it leaves the link
        # ID of the incoming link
        LinkDownUpID = Nodes[NodeID]["OutgoingLinksID"][out]
        # --- calculation of the next supply yime at the current node
        dn = Links[LinkDownUpID]["Length"] * \
            Links[LinkDownUpID]["NumLanes"] * Links[LinkDownUpID]["FD"]["kx"]
        if (dn / General["Peloton"]) != np.floor(dn / General["Peloton"]):
            dn = int(dn / General["Peloton"]) + 1
        else:
            dn = int(dn / General["Peloton"])

        sp = Event_NodeID["SupplyTimes"]["Downstream"].shape
        if n + dn >= sp[0]:
            nbLinesToAdd = n + dn - sp[0] + 1
            Event_NodeID["SupplyTimes"]["Downstream"] = np.vstack(
                (Event_NodeID["SupplyTimes"]["Downstream"], -np.inf * np.ones((nbLinesToAdd, sp[1]))))

        Event_NodeID["SupplyTimes"]["Downstream"][n + dn, out] = np.inf

    # ---- (3) Arrival Computation at the next node
    # Update of the 'Arrivals' at the node Downstream
    # The vehicles are added to the list of the arrival at the node downstream
    # Ajout du véhicule dans la liste des arrivés au noeud en aval
    if Nodes[NodeID]["NumOutgoingLinks"] > 0 and Nodes[NodeID]["NumOutgoingLinks"] >= out:
        # this is an internal node
        # -- transit time at the previous node
        transit_time = Nodes[NodeID]["TransitTime"]
        # -- Searching time at the previous node
        LinkID = Nodes[NodeID]["OutgoingLinksID"][out]
        NodeDownID = Links[LinkID]["NodeDownID"]
        Event_NodeDownID = Events[NodeDownID]
        indown = np.where(Nodes[NodeDownID]["IncomingLinksID"] == LinkID)[0][0]

        n_down = Event_NodeDownID["Arrivals"][indown]["Num"] + 1
        Event_NodeDownID["Arrivals"][indown]["Num"] = n_down
        if False:
            classID = Vehicles[veh]["VehClassID"]
            VehicleSpeed = VehicleClass[classID]["Speed"]
            VehicleSpeed = min(VehicleSpeed, Links[LinkID]["FD"]["u"])
        else:
            VehicleSpeed = Links[LinkID]["Speed"]

        # Recording information
        # Adding in the events of the downstream node
        Event_NodeDownID["Arrivals"][indown]["VehID"] = np.concatenate(
            (Event_NodeDownID["Arrivals"][indown]["VehID"], np.array([veh])))
        Event_NodeDownID["Arrivals"][indown]["IsExit"] = np.concatenate(
            (Event_NodeDownID["Arrivals"][indown]["IsExit"], np.array([0])))
        # Event_NodeDownID["Arrivals"][indown]["NextLinkID"] = np.concatenate((Event_NodeDownID["Arrivals"][indown]["NextLinkID"], np.array([0])))
        CurrentNodePath = Vehicles[veh]["CurrentNode"] + 2
        if CurrentNodePath < len(Vehicles[veh]["Path"]):
            NextLinkID = Vehicles[veh]["Path"][CurrentNodePath]
            # Manage the reserved lanes
            if Links[NextLinkID]["AssociatedLink"] != None:
                probVect = Links[NextLinkID]["LaneProbabilities"][int(
                    Vehicles[veh]["VehicleClass"])]
                rng = random.random()
                if rng > probVect[0]:
                    #                    print("Vehicle  " + str(veh) + " as " + VehicleClass[int(Vehicles[veh]["VehicleClass"])]["Name"] + " had probability " + str(probVect[0]) + " to take principal link and has got " + str(rng) + " so he goes to the alternate...")
                    NextLinkID = Links[NextLinkID]["AssociatedLink"]
#                else:
#                    print("Vehicle  " + str(veh) + " as " + VehicleClass[int(Vehicles[veh]["VehicleClass"])]["Name"] + " had probability " + str(probVect[0]) + " to take principal link and has got " + str(rng) + " so he goes to the principal...")
            _outid = np.where(
                Nodes[NodeDownID]["OutgoingLinksID"] == NextLinkID)[0][0]
            Event_NodeDownID["Arrivals"][indown]["NextLinkID"] = np.concatenate(
                (Event_NodeDownID["Arrivals"][indown]["NextLinkID"], np.array([_outid])))
        else:
            Event_NodeDownID["Arrivals"][indown]["NextLinkID"] = np.concatenate(
                (Event_NodeDownID["Arrivals"][indown]["NextLinkID"], np.array([Nodes[NodeDownID]["NumOutgoingLinks"]])))

        _dTime = current_time + transit_time + \
            Links[LinkID]["Length"] / VehicleSpeed
        Event_NodeDownID["Arrivals"][indown]["Time"] = np.concatenate(
            (Event_NodeDownID["Arrivals"][indown]["Time"], np.array([_dTime])))
    else:
        NodeDownID = []
        Event_NodeDownID = []

    # ---- (4) new constraints upstream
    # Update of the "supply times downstream" of the node upstream
    if Nodes[NodeID]["NumIncomingLinks"] > 0 and IN <= Nodes[NodeID]["NumIncomingLinks"]:
        # this is an internal node
        # --- searching the node and the link upstream
        LinkUpID = Nodes[NodeID]["IncomingLinksID"][IN]
        NodeUpID = Links[LinkUpID]["NodeUpID"]
        j = np.where(Nodes[NodeUpID]["OutgoingLinksID"] == LinkUpID)[0]
        n = np.where(Events[NodeID]["Arrivals"][IN]["VehID"] == veh)[0]
        # Calculation of the next supply time downstream
        dn = Links[LinkUpID]["Length"] * \
            Links[LinkUpID]["NumLanes"] * Links[LinkUpID]["FD"]["kx"]
        if (dn / General["Peloton"]) != np.floor(dn / General["Peloton"]):
            dn_exact = int(dn / General["Peloton"]) + 1
        else:
            dn_exact = int(dn / General["Peloton"])
        dt = Links[LinkUpID]["Length"] / Links[LinkUpID]["FD"]["w"]
        dt_exact = dt + \
            (dn_exact*General["Peloton"]-dn) * (1*Links[LinkUpID]["FD"]["C"])
        Event_NodeUpID = Events[NodeUpID]
        Event_NodeUpID["SupplyTimes"]["Downstream"][n +
                                                    dn_exact, j] = current_time + dt_exact
    else:
        NodeUpID = []
        Event_NodeUpID = []
    return Event_NodeID, NodeID, Event_NodeDownID, NodeDownID, Event_NodeUpID, NodeUpID


def execute_and_update_event(Links, Exits, Nodes, General, Vehicles, VehicleClass, Events, NextEvents):

    # ---- (0) Initialisation
    NodeID = NextEvents["Node"]
    current_time = NextEvents["Time"]
    out = NextEvents["NextLink"]
    IN = NextEvents["PreviousLink"]
    veh = NextEvents["VehID"]
    Event_NodeID = Events[NodeID]
    
    '''           
    linkOUT = Nodes[NodeID]["OutgoingLinksID"][out]
    linkIN = Nodes[NodeID]["IncomingLinksID"][IN]
    '''
    
    # ---- (1) Recording the current event
    # Record of the passage time in the varaible 'Exits' at the current node
    n = Event_NodeID["Exits"][out]["Num"] + 1
    Event_NodeID["Exits"][out]["Num"] = n
    Event_NodeID["Exits"][out]["VehID"] = np.concatenate(
        (Event_NodeID["Exits"][out]["VehID"], np.array([veh])))
    Event_NodeID["Exits"][out]["Time"] = np.concatenate(
        (Event_NodeID["Exits"][out]["Time"], np.array([current_time])))
    Event_NodeID["Exits"][out]["Regime"] = np.concatenate(
        (Event_NodeID["Exits"][out]["Regime"], np.array([NextEvents["Regime"]])))
    Event_NodeID["Exits"][out]["PreviousLinkID"] = np.concatenate(
        (Event_NodeID["Exits"][out]["PreviousLinkID"], np.array([IN])))
    # ---
    in_veh = np.where(Event_NodeID["Arrivals"][IN]["VehID"] == veh)[0]
    Event_NodeID["Arrivals"][IN]["IsExit"][in_veh] = True

    # ---- (2) New exit contraint at the current node
    # --- next time due to capacity downstream
    if out >= Nodes[NodeID]["NumOutgoingLinks"]:
        if Nodes[NodeID]["NumOutgoingLinks"] == 0:
            # Si le noeud est un noeud de sortie : la capacité est forcée pour la sortie
            h_down = 0
            ExitID = NodeID
            # Si une mise à jour de la capacité est nécessaire
            f = np.where(Exits[ExitID]["Supply"]["Time"] < current_time)[0]
            if len(f) > 0:
                f = f[-1]
                h_down = 1 / Exits[ExitID]["Supply"]["Data"][f]
        else:
            h_down = 0
    else:
        # If NodeID is an entry node or an internal node : the capacity
        # depends on the capacity of the downstream link
        Capacity = (1 - Nodes[NodeID]["CapacityDrop"][IN]) * \
            Links[Nodes[NodeID]["OutgoingLinksID"][out]]["Ref_Capacity"]
            
        ''' On récupère la capacité temporaire de sortie du noeud, pour le lien de sortie. On prend ensuite le minimum des deux capacités '''
        Capacity = min(Capacity, Nodes[NodeID]["Tmp_DownCapacity"][out])
        
        h_down = 1/Capacity
        # s'il y a une valeur de Capacity Forced indique au noeud, alors elle s'applique
        if Nodes[NodeID]["CapacityForced"] < np.inf:
            h_down = 1/Nodes[NodeID]["CapacityForced"]
    h_down = h_down * General["Peloton"]
    Event_NodeID["SupplyTimes"]["DownCapacity"][out] = current_time + h_down


    # --- next time due to capacity upstream
    if IN > Nodes[NodeID]["NumIncomingLinks"] - 1:
        # Si NodeID est un noeud d'entrée
        h_up = 0
    else:
        Capacity = Links[Nodes[NodeID]["IncomingLinksID"][IN]]["Ref_Capacity"]
        tmp_capacity =  Nodes[NodeID]["Tmp_UpCapacity"][IN]
        Capacity = min(Capacity,tmp_capacity)
        h_up = 1 / Capacity

    h_up = h_up * General["Peloton"]
    Event_NodeID["SupplyTimes"]["UpCapacity"][IN] = current_time + h_up

    # --- future constraint for the vehicle "veh+dn" at the node
    if out < Nodes[NodeID]["NumOutgoingLinks"]:
        # there is a downstream link: this vehicle "veh" is a constraint for
        # vehicle "veh + dn" at the current node, until it leaves the link
        # ID of the incoming link
        LinkDownUpID = Nodes[NodeID]["OutgoingLinksID"][out]
        # --- calculation of the next supply yime at the current node
        dn = Links[LinkDownUpID]["Length"] * \
            Links[LinkDownUpID]["NumLanes"] * Links[LinkDownUpID]["FD"]["kx"]
        if (dn / General["Peloton"]) != np.floor(dn / General["Peloton"]):
            dn = int(dn / General["Peloton"]) + 1
        else:
            dn = int(dn / General["Peloton"])

        sp = Event_NodeID["SupplyTimes"]["Downstream"].shape
        if n + dn >= sp[0]:
            nbLinesToAdd = n + dn - sp[0] + 1
            Event_NodeID["SupplyTimes"]["Downstream"] = np.vstack(
                (Event_NodeID["SupplyTimes"]["Downstream"], -np.inf * np.ones((nbLinesToAdd, sp[1]))))

        Event_NodeID["SupplyTimes"]["Downstream"][n + dn, out] = np.inf

    # ---- (3) Arrival Computation at the next node
    # Update of the 'Arrivals' at the node Downstream
    # The vehicles are added to the list of the arrival at the node downstream
    # Ajout du véhicule dans la liste des arrivés au noeud en aval
    if Nodes[NodeID]["NumOutgoingLinks"] > 0 and Nodes[NodeID]["NumOutgoingLinks"] >= out:
        # this is an internal node
        # -- transit time at the previous node
        transit_time = Nodes[NodeID]["TransitTime"]
        # -- Searching time at the previous node
        LinkID = Nodes[NodeID]["OutgoingLinksID"][out]
        NodeDownID = Links[LinkID]["NodeDownID"]
        Event_NodeDownID = Events[NodeDownID]
        indown = np.where(Nodes[NodeDownID]["IncomingLinksID"] == LinkID)[0][0]

        n_down = Event_NodeDownID["Arrivals"][indown]["Num"] + 1
        Event_NodeDownID["Arrivals"][indown]["Num"] = n_down
        if False:
            classID = Vehicles[veh]["VehClassID"]
            VehicleSpeed = VehicleClass[classID]["Ref_Speed"]
            speed_limit = Nodes[NodeID]['Tmp_DownSpeedLimit'][out]
            VehicleSpeed = min(VehicleSpeed, Links[LinkID]["FD"]["u"],speed_limit)
        else:
            speed_limit = Nodes[NodeID]['Tmp_DownSpeedLimit'][out]
            VehicleSpeed = Links[LinkID]["Ref_Speed"]
            VehicleSpeed = min(VehicleSpeed,speed_limit)

        # Recording information
        # Adding in the events of the downstream node
        Event_NodeDownID["Arrivals"][indown]["VehID"] = np.concatenate(
            (Event_NodeDownID["Arrivals"][indown]["VehID"], np.array([veh])))
        Event_NodeDownID["Arrivals"][indown]["IsExit"] = np.concatenate(
            (Event_NodeDownID["Arrivals"][indown]["IsExit"], np.array([0])))
        # Event_NodeDownID["Arrivals"][indown]["NextLinkID"] = np.concatenate((Event_NodeDownID["Arrivals"][indown]["NextLinkID"], np.array([0])))
        CurrentNodePath = Vehicles[veh]["CurrentNode"] + 2
        if CurrentNodePath < len(Vehicles[veh]["Path"]):
            NextLinkID = Vehicles[veh]["Path"][CurrentNodePath]
            # Manage the reserved lanes
            if Links[NextLinkID]["AssociatedLink"] != None:
                probVect = Links[NextLinkID]["LaneProbabilities"][int(
                    Vehicles[veh]["VehicleClass"])]
                rng = random.random()
                if rng > probVect[0]:
                    #                    print("Vehicle  " + str(veh) + " as " + VehicleClass[int(Vehicles[veh]["VehicleClass"])]["Name"] + " had probability " + str(probVect[0]) + " to take principal link and has got " + str(rng) + " so he goes to the alternate...")
                    NextLinkID = Links[NextLinkID]["AssociatedLink"]
#                else:
#                    print("Vehicle  " + str(veh) + " as " + VehicleClass[int(Vehicles[veh]["VehicleClass"])]["Name"] + " had probability " + str(probVect[0]) + " to take principal link and has got " + str(rng) + " so he goes to the principal...")
            _outid = np.where(
                Nodes[NodeDownID]["OutgoingLinksID"] == NextLinkID)[0][0]
            Event_NodeDownID["Arrivals"][indown]["NextLinkID"] = np.concatenate(
                (Event_NodeDownID["Arrivals"][indown]["NextLinkID"], np.array([_outid])))
        else:
            Event_NodeDownID["Arrivals"][indown]["NextLinkID"] = np.concatenate(
                (Event_NodeDownID["Arrivals"][indown]["NextLinkID"], np.array([Nodes[NodeDownID]["NumOutgoingLinks"]])))

        _dTime = current_time + transit_time + \
            Links[LinkID]["Length"] / VehicleSpeed
        Event_NodeDownID["Arrivals"][indown]["Time"] = np.concatenate(
            (Event_NodeDownID["Arrivals"][indown]["Time"], np.array([_dTime])))
    else:
        NodeDownID = []
        Event_NodeDownID = []

    # ---- (4) new constraints upstream
    # Update of the "supply times downstream" of the node upstream
    if Nodes[NodeID]["NumIncomingLinks"] > 0 and IN <= Nodes[NodeID]["NumIncomingLinks"]:
        # this is an internal node
        # --- searching the node and the link upstream
        LinkUpID = Nodes[NodeID]["IncomingLinksID"][IN]
        NodeUpID = Links[LinkUpID]["NodeUpID"]
        j = np.where(Nodes[NodeUpID]["OutgoingLinksID"] == LinkUpID)[0]
        n = np.where(Events[NodeID]["Arrivals"][IN]["VehID"] == veh)[0]
        # Calculation of the next supply time downstream
        dn = Links[LinkUpID]["Length"] * \
            Links[LinkUpID]["NumLanes"] * Links[LinkUpID]["FD"]["kx"]
        if (dn / General["Peloton"]) != np.floor(dn / General["Peloton"]):
            dn_exact = int(dn / General["Peloton"]) + 1
        else:
            dn_exact = int(dn / General["Peloton"])
        dt = Links[LinkUpID]["Length"] / Links[LinkUpID]['Ref_Wavespeed']
        dt_exact = dt + \
            (dn_exact*General["Peloton"]-dn) * (1*Links[LinkUpID]["FD"]["C"])
        Event_NodeUpID = Events[NodeUpID]
        Event_NodeUpID["SupplyTimes"]["Downstream"][n +
                                                    dn_exact, j] = current_time + dt_exact
    else:
        NodeUpID = []
        Event_NodeUpID = []
    return Event_NodeID, NodeID, Event_NodeDownID, NodeDownID, Event_NodeUpID, NodeUpID
# =============================================================================
# Sub function
# =============================================================================
def init_dic_action(S):
    dic_action_in_progress = {}
    for nodeid in S['Nodes']:
        NumIn,NumOut = len(S['Nodes'][nodeid]['IncomingLinksID']),len(S['Nodes'][nodeid]['OutgoingLinksID'])
        dic_action_in_progress[nodeid] = {'IncomingLinksID' : {},'OutgoingLinksID': {}}
        for k in range(NumIn):
            dic_action_in_progress[nodeid]['IncomingLinksID'][k] = {'Capacity' : [], 'speed_limit' : []}
        for k in range(NumOut):
            dic_action_in_progress[nodeid]['OutgoingLinksID'][k] = {'Capacity' : [], 'speed_limit' : []}            
    return(dic_action_in_progress)

def get_capacity(S,actions,link):
    #sort capacity action by "type"
    dic_capacity = sort_capacity(actions)
    
    #Either there is juste "speed limit" as action 
    if dic_capacity['additional_factor'] != [] and ( dic_capacity['lane_reduction'] == [] and dic_capacity['substraction_factor'] == [] ):
        C_ref = S['Links'][link]['Ref_Capacity']
        additional = dic_capacity['additional_factor']
        add = max(additional)
        capacity = (1+add)*C_ref
        
        return(capacity)
    
    # Or we doesn't consider "speed_limit"
    else:
        lane_reductions = dic_capacity['lane_reduction']
        if lane_reductions != []:
            multiplication = min(lane_reductions)/S['Links'][link]['NumLanes']
        else:
            multiplication = 1
        C_ref = S['Links'][link]['Ref_Capacity'] * multiplication
        
        substraction_factor = dic_capacity['substraction_factor']
        if substraction_factor != []:
            sub = min(substraction_factor)
        else:
            sub = 1
        sub = C_ref - sub*C_ref    
        
        capacity = C_ref - sub
        return(capacity)
    
def sort_capacity(actions):
    dic_capacity = {'lane_reduction': [], 'additional_factor': [], 'substraction_factor': []}
    for action in actions:
        if action['Type'] == 'lane_reduction':
            dic_capacity['lane_reduction'].append(action['Args']['remaining_lane'])
        if action['Type'] == 'speed_limit' :
            dic_capacity['additional_factor'].append(action['Args']['Increase_capacity'])
    return(dic_capacity)

    
def delete_action_from_list(action,actions):
    for k,action_i in enumerate(actions):
        if action['Times'] == action_i['Times'] and action['Args'] == action_i['Args']:
            actions.pop(k)
            return(actions)
    return('Try to delete action which does not exists in dict')
