import math
import CPM
import SDUR

colors = ['red','green','blue','orange','grey','brown','black']

def GetLocations(no_of_regions, no_of_process_locations = 0):
    task_locations = []
    robot_locations = []
    process_locations = []
    no_of_robot_locations = 8

    for i in range(0, no_of_regions):
        # Robot locations
        for j in range(0, no_of_robot_locations):
            robot_locations.append(CPM.NavLocation(j+1, 0.0 + j*1.0, 7.0, 0.0, 1.0, 0.0, 0.0, 0.0, "RL"))

        # Process locations
        for j in range(0, no_of_process_locations-1):
            process_locations.append(CPM.NavLocation(j+1, 0.0, 7.0 + (j+1)*2.0, 0.0, 1.0, 0.0, 0.0, 0.0, "PL"))
        process_locations.append(CPM.NavLocation(no_of_process_locations, 12.0, 7.0 + (no_of_process_locations-1)*2.0, 0.0, 1.0, 0.0, 0.0, 0.0, "PL"))

        # Start/goal (0)
        task_locations.append(CPM.NavLocation(0, -0.4, -0.15, 0.0, 0.0, 0.0, 0.0, 1.0, "TL"))
        # task_locations.append(CPM.NavLocation(0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, "TL"))

        # shelf1 (1-10)
        task_locations.append(CPM.NavLocation(1, 0.450, 1.600, 0.0, 1.0, 0.0, 0.0, 0.0, "TL"))
        task_locations.append(CPM.NavLocation(2, 0.450, 2.575, 0.0, 1.0, 0.0, 0.0, 0.0, "TL"))
        task_locations.append(CPM.NavLocation(3, 0.450, 3.550, 0.0, 1.0, 0.0, 0.0, 0.0, "TL"))
        task_locations.append(CPM.NavLocation(4, 0.450, 4.525, 0.0, 1.0, 0.0, 0.0, 0.0, "TL"))
        task_locations.append(CPM.NavLocation(5, 0.450, 5.500, 0.0, 1.0, 0.0, 0.0, 0.0, "TL"))

        task_locations.append(CPM.NavLocation(6, 2.950, 1.600, 0.0, 0.0, 0.0, 0.0, 1.0, "TL"))
        task_locations.append(CPM.NavLocation(7, 2.950, 2.575, 0.0, 0.0, 0.0, 0.0, 1.0, "TL"))
        task_locations.append(CPM.NavLocation(8, 2.950, 3.550, 0.0, 0.0, 0.0, 0.0, 1.0, "TL"))
        task_locations.append(CPM.NavLocation(9, 2.950, 4.525, 0.0, 0.0, 0.0, 0.0, 1.0, "TL"))
        task_locations.append(CPM.NavLocation(10, 2.950, 5.500, 0.0, 0.0, 0.0, 0.0, 1.0, "TL"))

        # shelf2 (11-20)
        task_locations.append(CPM.NavLocation(11, 4.050, 1.600, 0.0, 1.0, 0.0, 0.0, 0.0, "TL"))
        task_locations.append(CPM.NavLocation(12, 4.050, 2.575, 0.0, 1.0, 0.0, 0.0, 0.0, "TL"))
        task_locations.append(CPM.NavLocation(13, 4.050, 3.550, 0.0, 1.0, 0.0, 0.0, 0.0, "TL"))
        task_locations.append(CPM.NavLocation(14, 4.050, 4.525, 0.0, 1.0, 0.0, 0.0, 0.0, "TL"))
        task_locations.append(CPM.NavLocation(15, 4.050, 5.500, 0.0, 1.0, 0.0, 0.0, 0.0, "TL"))

        task_locations.append(CPM.NavLocation(16, 6.350, 1.600, 0.0, 0.0, 0.0, 0.0, 1.0, "TL"))
        task_locations.append(CPM.NavLocation(17, 6.350, 2.575, 0.0, 0.0, 0.0, 0.0, 1.0, "TL"))
        task_locations.append(CPM.NavLocation(18, 6.350, 3.550, 0.0, 0.0, 0.0, 0.0, 1.0, "TL"))
        task_locations.append(CPM.NavLocation(19, 6.350, 4.525, 0.0, 0.0, 0.0, 0.0, 1.0, "TL"))
        task_locations.append(CPM.NavLocation(20, 6.350, 5.500, 0.0, 0.0, 0.0, 0.0, 1.0, "TL"))

        # shelf3 (21-30)
        task_locations.append(CPM.NavLocation(21, 7.550, 1.600, 0.0, 1.0, 0.0, 0.0, 0.0, "TL"))
        task_locations.append(CPM.NavLocation(22, 7.550, 2.575, 0.0, 1.0, 0.0, 0.0, 0.0, "TL"))
        task_locations.append(CPM.NavLocation(23, 7.550, 3.550, 0.0, 1.0, 0.0, 0.0, 0.0, "TL"))
        task_locations.append(CPM.NavLocation(24, 7.550, 4.525, 0.0, 1.0, 0.0, 0.0, 0.0, "TL"))
        task_locations.append(CPM.NavLocation(25, 7.550, 5.500, 0.0, 1.0, 0.0, 0.0, 0.0, "TL"))

        task_locations.append(CPM.NavLocation(26, 9.850, 1.600, 0.0, 0.0, 0.0, 0.0, 1.0, "TL"))
        task_locations.append(CPM.NavLocation(27, 9.850, 2.575, 0.0, 0.0, 0.0, 0.0, 1.0, "TL"))
        task_locations.append(CPM.NavLocation(28, 9.850, 3.550, 0.0, 0.0, 0.0, 0.0, 1.0, "TL"))
        task_locations.append(CPM.NavLocation(29, 9.850, 4.525, 0.0, 0.0, 0.0, 0.0, 1.0, "TL"))
        task_locations.append(CPM.NavLocation(30, 9.850, 5.500, 0.0, 0.0, 0.0, 0.0, 1.0, "TL"))

        # shelf4 (31-34)
        task_locations.append(CPM.NavLocation(31, 12.70, 6.3, 0.0, 0.707, 0.0, 0.0, -0.707, "TL"))
        task_locations.append(CPM.NavLocation(32, 13.55, 6.3, 0.0, 0.707, 0.0, 0.0, -0.707, "TL"))

        task_locations.append(CPM.NavLocation(33, 12.70, 4.0, 0.0, 0.707, 0.0, 0.0, 0.707, "TL"))
        task_locations.append(CPM.NavLocation(34, 13.55, 4.0, 0.0, 0.707, 0.0, 0.0, 0.707, "TL"))

        # shelf5 (35-38)
        task_locations.append(CPM.NavLocation(35, 12.70, 2.8, 0.0, 0.707, 0.0, 0.0, -0.707, "TL"))
        task_locations.append(CPM.NavLocation(36, 13.55, 2.8, 0.0, 0.707, 0.0, 0.0, -0.707, "TL"))

        task_locations.append(CPM.NavLocation(37, 12.70, 0.4, 0.0, 0.707, 0.0, 0.0, 0.707, "TL"))
        task_locations.append(CPM.NavLocation(38, 13.55, 0.4, 0.0, 0.707, 0.0, 0.0, 0.707, "TL"))

        # shelf6 (39-48)
        task_locations.append(CPM.NavLocation(39, 11.95, -1.90, 0.0, 0.0, 0.0, 0.0, 1.0, "TL"))
        task_locations.append(CPM.NavLocation(40, 11.95, -2.85, 0.0, 0.0, 0.0, 0.0, 1.0, "TL"))

        task_locations.append(CPM.NavLocation(41, 6.450, -1.90, 0.0, 1.0, 0.0, 0.0, 0.0, "TL"))
        task_locations.append(CPM.NavLocation(42, 6.450, -2.85, 0.0, 1.0, 0.0, 0.0, 0.0, "TL"))

        task_locations.append(CPM.NavLocation(43, 10.6, -4.1, 0.0, 0.707, 0.0, 0.0, 0.707, "TL"))
        task_locations.append(CPM.NavLocation(44, 9.15, -4.1, 0.0, 0.707, 0.0, 0.0, 0.707, "TL"))
        task_locations.append(CPM.NavLocation(45, 7.70, -4.1, 0.0, 0.707, 0.0, 0.0, 0.707, "TL"))

        task_locations.append(CPM.NavLocation(46, 10.6, -0.6, 0.0, 0.707, 0.0, 0.0, -0.707, "TL"))
        task_locations.append(CPM.NavLocation(47, 9.15, -0.6, 0.0, 0.707, 0.0, 0.0, -0.707, "TL"))
        task_locations.append(CPM.NavLocation(48, 7.70, -0.6, 0.0, 0.707, 0.0, 0.0, -0.707, "TL"))

        # shelfcylinder (49-52)
        task_locations.append(CPM.NavLocation(49, 3.75, -1.6, 0.0, -0.393, 0.0, 0.0, 0.92, "TL"))
        task_locations.append(CPM.NavLocation(50, 3.80, -4.2, 0.0, 0.374, 0.0, 0.0, 0.927, "TL"))
        task_locations.append(CPM.NavLocation(51, 1.30, -4.2, 0.0, 0.913, 0.0, 0.0, 0.4076, "TL"))
        task_locations.append(CPM.NavLocation(52, 1.40, -1.6, 0.0, 0.909, 0.0, 0.0, -0.418, "TL"))

    x_index = [0,1,0,-1,-1,0,0,1,1,1,0,0,0,-1,-1,-1,-1,0,0,0,0]
    y_index = [0,0,-1,0,0,1,1,0,0,0,-1,-1,-1,0,0,0,0,1,1,1,1]
    x_offset = 0.0
    y_offset = 0.0
    for i in range(0, no_of_regions):
        x_offset += x_index[i] * 16.0
        y_offset += y_index[i] * 12.0
        for j in range(i*53,i*53+53):
            task_locations[j].x += x_offset
            task_locations[j].y += y_offset
        for j in range(i*no_of_robot_locations,i*no_of_robot_locations+no_of_robot_locations):
            robot_locations[j].x += x_offset
            robot_locations[j].y += y_offset

    return task_locations, robot_locations, process_locations

################################################################################################
# Multi robot/human UseCases with flexible allocation tasks for industrial process application
#
def SetupUseCase(use_case, process_locations):
    if use_case[0] == "1":
        return CreateCollaborativeProcessModel(process_locations, 4)
    
    if use_case[0] == "2":
        return CreateCollaborativeProcessModel(process_locations, 8)

    if use_case[0] == "3":
        return CreateCollaborativeProcessModel(process_locations, 12)

    if use_case[0] == "4":
        return CreateCollaborativeProcessModel(process_locations, 16)

    if use_case[0] == "5":
        return CreateCollaborativeProcessModel(process_locations, 20)

def CreateCollaborativeProcessModel(process_locations, order_stock = 8):

    ######################################## 
    # 1) Create a Deliver process.

    # A concurrent process can run with concurrent processing of sequential items.
    # E.g., one agent can start with item 2 before another agent has completed item 1.
    concurrent = True
    # The deliver process has a fetch task and a delivery task
    no_of_subtasks = 2
    # The location of the process
    location = process_locations[0]
    proc1 = CPM.Process("Deliver", concurrent, no_of_subtasks, location)

    # Tasks performed by robot or human
    proc1.AddTeamFormation(1, 0, 0)
    proc1.AddTeamFormation(0, 1, 0)
    proc1.AddTeamFormation(1, 0, 1)

    # Note: Accessing a buffer requires a specific agent location, this does not apply for an order buffer.
    # An order buffer contains items that needs to be processed in order to complete orders.
    # An order may consist of several items that will be combined by the processes into a single output.
    buffer_is_order = True
    buffer_size = order_stock
    input_buffer = CPM.Buffer(buffer_size, buffer_is_order)

    # Input pop condition
    input_cond = ">0"
    # Number of items to pop
    input_no_of_items = 1
    proc1.AddInputBufferInterface(input_buffer, input_cond, input_no_of_items)

    buffer_size = 2
    buf1 = CPM.Buffer(buffer_size)

    # Output push condition
    output_cond = "<capacity"
    # Number of items to push
    output_no_of_items = 1
    proc1.AddOutputBufferInterface(buf1, output_cond, output_no_of_items)

    ######################################## 
    # 2) Create an assembly process
    concurrent = False
    no_of_subtasks = 1
    location = process_locations[1]
    proc2 = CPM.Process("Assemble", concurrent, no_of_subtasks, location)

    # Tasks performed by robot, human, robot + human or robot + overseer
    proc2.AddTeamFormation(1, 0, 0)
    proc2.AddTeamFormation(0, 1, 0)
    proc2.AddTeamFormation(1, 0, 1)
    proc2.AddTeamFormation(1, 1, 0)

    input_cond = ">=2"
    input_no_of_items = 2
    proc2.AddInputBufferInterface(buf1, input_cond, input_no_of_items)

    buffer_size = 2
    buf2 = CPM.Buffer(buffer_size)

    output_cond = "<capacity"
    output_no_of_items = 1
    proc2.AddOutputBufferInterface(buf2, output_cond, output_no_of_items)

    ######################################## 
    # 3) Create an inspection process
    concurrent = False
    no_of_subtasks = 1
    location = process_locations[2]
    proc3 = CPM.Process("Inspect", concurrent, no_of_subtasks, process_locations[2])

    # Tasks performed by robot, human or robot + overseer
    proc3.AddTeamFormation(1, 0, 0)
    proc3.AddTeamFormation(0, 1, 0)
    proc3.AddTeamFormation(1, 0, 1)

    input_cond = ">0"
    input_no_of_items = 1
    proc3.AddInputBufferInterface(buf2, input_cond, input_no_of_items)

    buffer_size = 2
    buf3 = CPM.Buffer(buffer_size)

    output_cond = "<capacity"
    output_no_of_items = 1
    proc3.AddOutputBufferInterface(buf3, output_cond, output_no_of_items)

    ######################################## 
    # 4) Create a transport process
    concurrent = True
    no_of_subtasks = 2
    location = process_locations[4]
    first_location = process_locations[2]
    proc4 = CPM.Process("Transport", concurrent, no_of_subtasks, location, first_location)

    # Tasks performed by robot or human
    proc4.AddTeamFormation(1, 0, 0)
    proc4.AddTeamFormation(0, 1, 0)
    proc4.AddTeamFormation(1, 0, 1)

    input_cond = ">=2"
    input_no_of_items = 2
    proc4.AddInputBufferInterface(buf3, input_cond, input_no_of_items)

    buffer_size = 4
    buf4 = CPM.Buffer(buffer_size)

    output_cond = "<capacity"
    output_no_of_items = 2
    proc4.AddOutputBufferInterface(buf4, output_cond, output_no_of_items)

    ######################################## 
    # 5) Create a palletizing process
    concurrent = False
    no_of_subtasks = 1
    location = process_locations[4]
    proc5 = CPM.Process("Palletize", concurrent, no_of_subtasks, location)

    # Tasks performed by robot, human or robot + overseer
    proc5.AddTeamFormation(1, 0, 0)
    proc5.AddTeamFormation(0, 1, 0)
    proc5.AddTeamFormation(1, 0, 1)

    input_cond = ">0"
    input_no_of_items = 1
    proc5.AddInputBufferInterface(buf4, input_cond, input_no_of_items)

    # Color code for plotting
    proc1.color = colors[0]
    proc2.color = colors[1]
    proc3.color = colors[2]
    proc4.color = colors[3]
    proc5.color = colors[4]

    return proc1

def RandomizeTaskLocations(first_process, task_locations, agent_locations, rand_gen):
    used_task_locations = []
    # Randomizing only for the first subtask of the first process
    # Select task locations
    index = rand_gen.randomUC.sample(range(0,len(task_locations)), len(task_locations))
    counter = 0
    cur = first_process
    while cur != None:
        for task in cur.tasks:
            if len(task.subtasks)>1:
                for i in range(0, len(task.subtasks)-1):
                    if i == 0 and cur.first_location != None:
                        # The very first sub task is first process location
                        task.subtasks[i].location = cur.first_location
                    else:
                        # First subtasks are task locations
                        task.subtasks[i].location = task_locations[index[counter]]
                        used_task_locations.append(task_locations[index[counter]])
                    counter+= 1
            # Last subtask is the process location
            task.subtasks[-1].location = cur.location
        # The single process location is used by all process tasks
        used_task_locations.append(cur.location)
        cur = cur.NextProcess()
    if counter >= len(task_locations):
        print("WARNING, out of task locations")
    used_task_locations.extend(agent_locations)

    return used_task_locations

def ComputeRandomizedTransitionDurations(locations):
    v_rob = 0.5
    cost_dict = {}

    for loc1 in locations:
        for loc2 in locations:
            if loc1.name == loc2.name:
                # Transition between the same location
                zero_dur = SDUR.StochasticDuration()
                zero_dur.Lock()
                cost_dict[(loc1.name, loc2.name)] = zero_dur
            else:
                # Estimate the transition cost between the two locations
				# Using cartesian distance between locations:
                tran_dist = math.sqrt(math.pow(loc1.x-loc2.x,2)+math.pow(loc1.y-loc2.y,2))
                transition_cost = tran_dist/v_rob
                # print("trans cost", transition_cost)

                # Store transition cost
                dur = SDUR.StochasticDuration()
                dur.SetUniform(0.9*transition_cost, 0.2*transition_cost)

                dur.Lock()
                # RTSGSup.PlotDuration(dur)
                cost_dict[(loc1.name, loc2.name)] = dur
    return cost_dict

def RandomizeTaskDurations(first_process, rand_gen):
    activity = first_process
    while activity != None:
        for task in activity.tasks:
            for subtask in task.subtasks:
                for i in range(0, len(activity.team_formations)):
                    tf = activity.team_formations[i]
                    subtask.duration.append(SDUR.StochasticDuration())
                    if activity.name == "Assemble":
                        if tf.no_of_humans == 1 and tf.no_of_robots == 0 and tf.no_of_overseers == 0:
                            # Single human
                            min_dur_low = 10
                            min_dur_high = 20
                            dev_min = 1.0
                            dev_max = 3.0
                        elif tf.no_of_humans == 0 and tf.no_of_robots == 1 and tf.no_of_overseers == 0:
                            # Single robot
                            min_dur_low = 30
                            min_dur_high = 40
                            dev_min = 0.5
                            dev_max = 1.0
                        elif tf.no_of_humans == 0 and tf.no_of_robots == 1 and tf.no_of_overseers == 1:
                            # Robot with human overseer
                            min_dur_low = 30
                            min_dur_high = 40
                            dev_min = 0.1
                            dev_max = 0.2
                        elif tf.no_of_humans == 1 and tf.no_of_robots == 1 and tf.no_of_overseers == 0:
                            # Human and robot collaborative
                            min_dur_low = 7.5
                            min_dur_high = 15
                            dev_min = 0.5
                            dev_max = 1.0
                        else:
                            raise Exception("Unexpected team formation")
                    elif activity.name == "Deliver" or activity.name == "Transport":
                        # Durations for loading and unloading
                        # Routing between loading and unloading is not included in subtask durations
                        if tf.no_of_humans == 1 and tf.no_of_robots == 0 and tf.no_of_overseers == 0:
                            # Single human
                            min_dur_low = 1
                            min_dur_high = 2
                            dev_min = 1.0
                            dev_max = 3.0
                        elif tf.no_of_humans == 0 and tf.no_of_robots == 1 and tf.no_of_overseers == 0:
                            # Single robot
                            min_dur_low = 3
                            min_dur_high = 4
                            dev_min = 0.5
                            dev_max = 1.0
                        elif tf.no_of_humans == 0 and tf.no_of_robots == 1 and tf.no_of_overseers == 1:
                            # Robot with human overseer
                            min_dur_low = 4
                            min_dur_high = 4
                            dev_min = 0.1
                            dev_max = 0.2
                        else:
                            raise Exception("Unexpected team formation")                        
                    else:
                        if tf.no_of_humans == 1 and tf.no_of_robots == 0 and tf.no_of_overseers == 0:
                            # Single human
                            min_dur_low = 10
                            min_dur_high = 20
                            dev_min = 1.0
                            dev_max = 3.0
                        elif tf.no_of_humans == 0 and tf.no_of_robots == 1 and tf.no_of_overseers == 0:
                            # Single robot
                            min_dur_low = 30
                            min_dur_high = 40
                            dev_min = 0.5
                            dev_max = 1.0
                        elif tf.no_of_humans == 0 and tf.no_of_robots == 1 and tf.no_of_overseers == 1:
                            # Robot with human overseer
                            min_dur_low = 40
                            min_dur_high = 40
                            dev_min = 0.1
                            dev_max = 0.2
                        else:
                            raise Exception("Unexpected team formation")                        

                    min_dur_random = min_dur_low + (min_dur_high - min_dur_low) * rand_gen.randomUC.random()
                    dev_dur_random = min_dur_random * (dev_min + (dev_max - dev_min) * rand_gen.randomUC.random())
                    subtask.duration[-1].SetUniform(min_dur_random, dev_dur_random)
                    subtask.duration[-1].Lock()
        activity = activity.NextProcess()

def CreateAgents(first_process, no_of_robots, no_of_humans, idle_min, preference_min):
    agents = []
    for i in range(0, no_of_robots):
        agents.append(CPM.Agent("R"+str(i+1), len(agents)))
    for i in range(0, no_of_humans):
        agents.append(CPM.Agent("H"+str(i+1), len(agents), True, idle_min, preference_min))

    # Remove invalid team formations caused by insufficient no of available agents
    cur_proc = first_process
    while cur_proc != None:
        remove = []
        for tf in cur_proc.team_formations:
            if (tf.no_of_humans + tf.no_of_overseers > no_of_humans) or (tf.no_of_robots > no_of_robots):
                remove.append(tf)
        for tf in remove:
            cur_proc.team_formations.remove(tf)
        if len(remove) > 0:
            # Update the team formation index
            for i in range(0, len(cur_proc.team_formations)):
                cur_proc.team_formations[i].index = i

        cur_proc = cur_proc.NextProcess()

    # Select preferred team formations
    for agent in agents:
        if agent.human:
            agent.SetHumanPreferredTeamFormations(first_process)

    return agents

def RandomizeAgentLocations(agents, agent_locations, rand_gen):
    agent_index = rand_gen.randomUC.sample(range(0,len(agent_locations)), len(agents))
    for i in range (0, len(agents)):
        agents[i].tasks[0].subtasks[0].location = agent_locations[agent_index[i]]

def RandomizeTaskAllocations(first_process, agents, rand_gen):
    # Allocate all tasks to agents.
    # Make a completely random allocation (for simplicity)
    cur = first_process
    while cur != None:
        for task in cur.tasks:
            # Random selection of agent
            if rand_gen.randomUC.randint(0,9) > 5:
                agent_index = []
                agent_index.append(rand_gen.randomUC.randrange(0,len(agents)))
            else:
                agent_index = rand_gen.randomUC.sample(range(-1,len(agents)),2)
            for a_index in agent_index:
                if a_index >= 0:
                    agents[a_index].tasks.append(task)
                    task.team.AllocateAgent(agents[a_index])
        cur = cur.NextProcess()

def ApplyExperiment(use_case, seqs, over, coll):
    # Apply the best solution found to the given use case.
    # The best solutions found are stored here in a hard coded representation.
    if use_case == "1A":
        # All
        # 1A 0 279,66  R1: R1-D1-A1,2c-I12- R2: R2-D3-D4-A3,4c-I34o-P12- H1: H1-D2-A1,2c-A3,4c-I34o-T12,34-P34
        seqs.append(['R1','D1','A1,2','I12'])
        seqs.append(['R2','D3','D4','A3,4','I34','P12'])
        seqs.append(['H1','D2','A1,2','A3,4','I34','T12,34','P34'])
        over.extend(['I34'])
        coll.extend(['A1,2','A3,4'])
    elif use_case == "1B":
        # All
        # 1B 0 235,97  R1: R1-D2-T12,34o- R2: R2-D1-A1,2c-A3,4c- R3: R3-D3-I12o- H1: H1-D4-I12o-I34- H2: H2-A1,2c-A3,4c-T12,34o-P12-P34
        seqs.append(['R1','D2','T12,34'])
        seqs.append(['R2','D1','A1,2','A3,4'])
        seqs.append(['R3','D3','I12'])
        seqs.append(['H1','D4','I12','I34'])
        seqs.append(['H2','A1,2','A3,4','T12,34','P12','P34'])
        over.extend(['T12,34','I12'])
        coll.extend(['A1,2','A3,4'])
    elif use_case == "2A":
        # GAW
        # 2A 8 431,50  R1: R1-D3-D5-D7-I34-T12,34-P12-P34- R2: R2-D2-D4-D6-A5,6c-D8-I56-I78-T56,78o- H1: H1-D1-A1,2-I12-A3,4-A5,6c-A7,8-T56,78o-P56-P78        
        seqs.append(['R1','D3','D5','D7','I34','T12,34','P12','P34'])
        seqs.append(['R2','D2','D4','D6','A5,6','D8','I56','I78','T56,78'])
        seqs.append(['H1','D1','A1,2','I12','A3,4','A5,6','A7,8','T56,78','P56','P78'])
        over.extend(['T56,78'])
        coll.extend(['A5,6'])
    elif use_case == "2B":
        # GAW
        # 2B 2 299,46  R1: R1-D6-A5,6c-I56-I78-T56,78o-P56o- R2: R2-D3-D5-D7-P12- R3: R3-D2-D4-A3,4c-D8-A7,8c- H1: H1-A3,4c-A5,6c-T12,34-P34-P56o-P78- H2: H2-D1-A1,2-I12-I34-A7,8c-T56,78o
        seqs.append(['R1','D6','A5,6','I56','I78','T56,78','P56'])
        seqs.append(['R2','D3','D5','D7','P12'])
        seqs.append(['R3','D2','D4','A3,4','D8','A7,8'])
        seqs.append(['H1','A3,4','A5,6','T12,34','P34','P56','P78'])
        seqs.append(['H2','D1','A1,2','I12','I34','A7,8','T56,78'])
        over.extend(['T56,78','P56'])
        coll.extend(['A5,6','A3,4','A7,8'])
    elif use_case == "3A":
        # GAW
        # 3A 6 635,03  R1: R1-D1-A1,2c-D4-D6-A5,6c-I34-I56-A7,8-D11-D12-T56,78o-P56-P910- R2: R2-D2-D3-D5-D7-D8-D9-D10-I78-I910-I1112-T910,1112- H1: H1-A1,2c-I12-A3,4-A5,6c-T12,34-P12-P34-A9,10-A11,12-T56,78o-P78-P1112
        seqs.append(['R1','D1','A1,2','D4','D6','A5,6','I34','I56','A7,8','D11','D12','T56,78','P56','P910'])
        seqs.append(['R2','D2','D3','D5','D7','D8','D9','D10','I78','I910','I1112','T910,1112'])
        seqs.append(['H1','A1,2','I12','A3,4','A5,6','T12,34','P12','P34','A9,10','A11,12','T56,78','P78','P1112'])
        over.extend(['T56,78'])
        coll.extend(['A1,2','A5,6'])
    elif use_case == "3B":
        # GAW
        # 3B 11 445,13  R1: R1-D2-A1,2c-A3,4c-D7-T12,34-P12-P34-P56-P78o- R2: R2-D3-D6-D9-D11-T56,78-P1112o- R3: R3-D1-D5-D8-D10-A9,10c-D12-A11,12c-I910o-T910,1112- H1: H1-A1,2c-A3,4c-I12-I34-A7,8-A9,10c-I78-A11,12c-I910o-I1112- H2: H2-D4-A5,6-I56-P78o-P910-P1112o
        seqs.append(['R1','D2','A1,2','A3,4','D7','T12,34','P12','P34','P56','P78'])
        seqs.append(['R2','D3','D6','D9','D11','T56,78','P1112'])
        seqs.append(['R3','D1','D5','D8','D10','A9,10','D12','A11,12','I910','T910,1112'])
        seqs.append(['H1','A1,2','A3,4','I12','I34','A7,8','A9,10','I78','A11,12','I910','I1112'])
        seqs.append(['H2','D4','A5,6','I56','P78','P910','P1112'])
        over.extend(['P78','P1112','I910'])
        coll.extend(['A1,2','A3,4','A9,10','A11,12'])
    elif use_case == "4A":
        # GAW
        # 4A 17 827,69  R1: R1-D2-A1,2c-D3-D6-A5,6c-T12,34-P12-P34-D10-D12-D13-A13,14c-D16-A15,16c-I1314-I1516-T1314,1516-P1516- R2: R2-D1-D4-D5-D7-D8-D9-D11-T56,78-D14-D15-I1112-T910,1112-P910-P1112o- H1: H1-A1,2c-I12-A3,4-I34-A5,6c-A7,8-I56-I78-A9,10-A11,12-I910-A13,14c-A15,16c-P56-P78-P1112o-P1314
        seqs.append(['R1','D2','A1,2','D3','D6','A5,6','T12,34','P12','P34','D10','D12','D13','A13,14','D16','A15,16','I1314','I1516','T1314,1516','P1516'])
        seqs.append(['R2','D1','D4','D5','D7','D8','D9','D11','T56,78','D14','D15','I1112','T910,1112','P910','P1112'])
        seqs.append(['H1','A1,2','I12','A3,4','I34','A5,6','A7,8','I56','I78','A9,10','A11,12','I910','A13,14','A15,16','P56','P78','P1112','P1314'])
        over.extend(['P1112'])
        coll.extend(['A1,2','A5,6','A13,14','A15,16'])
    elif use_case == "4B":
        # GAW
        #  4B 5 613,92  R1: R1-D1-D6-D7-A7,8c-I56o-A9,10c-I78o-D14-I1112-I1314-T1314,1516o- R2: R2-A1,2c-I34o-T12,34-P12-P34-D13-D15-A15,16c- R3: R3-D2-D3-A3,4c-D5-D8-D9-D11-D12-A11,12c-D16-T910,1112-P1516- H1: H1-A1,2c-I12-I34o-D10-I910-A11,12c-A13,14-A15,16c-I1516-T1314,1516o-P1314- H2: H2-D4-A3,4c-A5,6-A7,8c-I56o-A9,10c-I78o-T56,78-P56-P78-P910-P1112
        seqs.append(['R1','D1','D6','D7','A7,8','I56','A9,10','I78','D14','I1112','I1314','T1314,1516'])
        seqs.append(['R2','A1,2','I34','T12,34','P12','P34','D13','D15','A15,16'])
        seqs.append(['R3','D2','D3','A3,4','D5','D8','D9','D11','D12','A11,12','D16','T910,1112','P1516'])
        seqs.append(['H1','A1,2','I12','I34','D10','I910','A11,12','A13,14','A15,16','I1516','T1314,1516','P1314'])
        seqs.append(['H2','D4','A3,4','A5,6','A7,8','I56','A9,10','I78','T56,78','P56','P78','P910','P1112'])
        over.extend(['I56','I78','T1314,1516','I34'])
        coll.extend(['A7,8','A9,10','A1,2','A15,16','A3,4','A11,12'])
    elif use_case == "5A":
        # GRASP
        # 5A 11 1113,60  R1: R1-D1-A1,2c-I12-D5-D7-D8-P12-P34-P56-P78-D13-D14-D15-A15,16c-D18-T1314,1516-P910-P1112-P1314-P1516-P1920- R2: R2-D3-D4-D6-A5,6c-T12,34-D9-D10-T56,78-D11-D12-A11,12c-I1112o-A13,14c-I1314-D17-D19-D20-A19,20c-T1718,1920-P1718o- H1: H1-D2-A1,2c-A3,4-A5,6c-I34-A7,8-I56-I78-A9,10-I910-A11,12c-I1112o-T910,1112-A13,14c-D16-A15,16c-I1516-A17,18-I1718-A19,20c-I1920-P1718o
        seqs.append(['R1','D1','A1,2','I12','D5','D7','D8','P12','P34','P56','P78','D13','D14','D15','A15,16','D18','T1314,1516','P910','P1112','P1314','P1516','P1920'])
        seqs.append(['R2','D3','D4','D6','A5,6','T12,34','D9','D10','T56,78','D11','D12','A11,12','I1112','A13,14','I1314','D17','D19','D20','A19,20','T1718,1920','P1718'])
        seqs.append(['H1','D2','A1,2','A3,4','A5,6','I34','A7,8','I56','I78','A9,10','I910','A11,12','I1112','T910,1112','A13,14','D16','A15,16','I1516','A17,18','I1718','A19,20','I1920','P1718'])
        over.extend(['I1112','P1718'])
        coll.extend(['A1,2','A15,16','A5,6','A11,12','A13,14','A19,20'])
    elif use_case == "5B":
        # GRASP
        # 5B 14 754,63  R1: R1-A1,2c-A3,4c-D8-D9-A9,10c-T56,78-D13-D15-D17-D19-A19,20c-P1516- R2: R2-D1-D4-D6-A5,6c-T12,34o-P12-P34-I1112-T910,1112o-P56o-P78o-P910-P1112-P1314- R3: R3-D2-D5-D7-A7,8c-D10-D11-D12-A11,12c-D14-D16-D18-A17,18c-D20-T1718,1920-P1920o- H1: H1-A1,2c-A3,4c-I12-I34-A7,8c-I56-A11,12c-T910,1112o-P56o-P78o-T1314,1516-P1718-P1920o- H2: H2-D3-A5,6c-T12,34o-A9,10c-I78-I910-A13,14-A15,16-I1314-A17,18c-I1516-A19,20c-I1718-I1920
        seqs.append(['R1','A1,2','A3,4','D8','D9','A9,10','T56,78','D13','D15','D17','D19','A19,20','P1516'])
        seqs.append(['R2','D1','D4','D6','A5,6','T12,34','P12','P34','I1112','T910,1112','P56','P78','P910','P1112','P1314'])
        seqs.append(['R3','D2','D5','D7','A7,8','D10','D11','D12','A11,12','D14','D16','D18','A17,18','D20','T1718,1920','P1920'])
        seqs.append(['H1','A1,2','A3,4','I12','I34','A7,8','I56','A11,12','T910,1112','P56','P78','T1314,1516','P1718','P1920'])
        seqs.append(['H2','D3','A5,6','T12,34','A9,10','I78','I910','A13,14','A15,16','I1314','A17,18','I1516','A19,20','I1718','I1920'])
        over.extend(['T12,34','T910,1112','P56','P78','P1920'])
        coll.extend(['A1,2','A3,4','A9,10','A19,20','A5,6','A7,8','A11,12','A17,18'])
    else:
        raise Exception("Unknown use case")

def ApplySolution(proc1, agents, use_case):
    sequences = []
    overseer = []
    collaborative = []

    ApplyExperiment(use_case, sequences, overseer, collaborative)

    # Collect all agents and tasks
    all_agents = {}
    for a in agents:
        all_agents[a.name] = a
    all_tasks = {}
    proc = proc1
    while proc != None:
        for t in proc.tasks:
            all_tasks[t.name] = t
        proc = proc.NextProcess()

    # Apply the solution
    CPM.PrepareForNewSolution(proc1, agents)
    for seq in sequences:
        for i in range(1, len(seq)):
            all_agents[seq[0]].tasks.append(all_tasks[seq[i]])
            all_tasks[seq[i]].team.AllocateAgent(all_agents[seq[0]])
            if seq[i] in overseer:
                all_tasks[seq[i]].team_formation_index = 2
            elif seq[i] in collaborative:
                all_tasks[seq[i]].team_formation_index = 3
            elif all_agents[seq[0]].human:
                all_tasks[seq[i]].team_formation_index = 1
            else:
                all_tasks[seq[i]].team_formation_index = 0
