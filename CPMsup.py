import networkx as nx
import numpy as np
import random
import pickle
import SDUR
import CPM

def CreateConvergenceLogFile(file_name):
    fd=open("logs//" + file_name + ".log",'w')
    fd.close()
    fd_seq=open("logs//" + file_name + "_seq.log",'w')
    print("Run GenIt Iteration Cost Makespan MakespanMedian MakespanLow Time", file=fd_seq)
    fd_seq.close()

def CreateLogFile(file_name, agents):
    fd=open("logs//" + file_name + ".log",'w')
    print("UseCase #Tasks Algorithm Run Cost Makespan MSMedian MSLowMedian StatDist Feasible ", end="", file=fd)
    for agent in agents:
        print("%s IdleQuota PrefQuota "%(agent.name), end="", file=fd)
    print("CostCount CostTime TotTime", file=fd)
    fd.close()
    fd_seq=open("logs//" + file_name + "_seq.log",'w')
    print("UseCase Run Cost Sequences", file=fd_seq)
    fd_seq.close()

def AppendSolutionToLogFiles(use_case, no_of_tasks, r, alg_name, makespan, makespan_low, agents, percentile, cost_count, cost_time, exec_time, file_name):
    fd=open("logs//" + file_name + ".log",'a')
    feasible, cost, ms_cost, ms_median, ms_low_median, idle_quota, pref_quota = CPM.SolutionCost(makespan, makespan_low, agents, percentile)
    stat_dist = makespan.StatDist(makespan_low)
    print(("%s %d %s %d %.2f %.2f %.2f %.2f %.6f %s "%(use_case, no_of_tasks, alg_name, r, cost, ms_cost, ms_median, ms_low_median, stat_dist, str(feasible))).replace(".",","), file=fd, end='')
    for agent in agents:
        iq = str(round(idle_quota[agent.name],4))
        if agent.human:
            pq = str(round(pref_quota[agent.name],4))
        else:
            pq = "N/A"
        print(("%s %s %s "%(agent.name, iq, pq)).replace(".",","), file=fd, end='')
    print(("%d %.2f %.2f"%(cost_count, cost_time, exec_time)).replace(".",","), file=fd)
    fd.close()

    fd_seq=open("logs//" + file_name + "_seq.log",'a')
    print(("%s %d %.2f "%(use_case, r, cost)).replace(".",","), file=fd_seq, end='')
    seqs = ""
    for agent in agents:
        seqs += " " + agent.name + ": "
        for t in range(0, len(agent.tasks)):
            seqs += agent.tasks[t].name
            if agent.tasks[t].process != None:
                tf = agent.tasks[t].process.team_formations[agent.tasks[t].team_formation_index]
                if tf.no_of_overseers > 0:
                    seqs += "o"
                elif tf.no_of_robots > 0 and tf.no_of_humans > 0:
                    seqs += "c"
            if t < len(agent.tasks)-1 or agent != agents[-1]:
                seqs+="-"
    print(seqs, file=fd_seq)
    fd_seq.close()

class RandomGenerators(object):
    def __init__(self, repeatable, seed):
        if repeatable:
            self.randomUC = random.Random(seed)     # Generate use cases
            np.random.seed(seed)
        else:
            self.randomUC = random.Random()

    def SaveState(self, file_name):
        with open(file_name, 'w') as fh:
            fh.write(str(self.randomUC.getstate())+'\n')
            np_ran = np.random.get_state()
            with open(file_name+"2", 'wb') as handle:
                pickle.dump(np_ran, handle, protocol=pickle.HIGHEST_PROTOCOL)

    def LoadState(self, file_name):
        with open(file_name, 'r') as fh:
            self.randomUC.setstate(eval(fh.readline()))
            with open(file_name+"2", 'rb') as handle:
                np_ran = pickle.load(handle)
            np.random.set_state(np_ran)


def PrintAgentSequences(agents):
    print("Agent sequences")
    for agent in agents:
        info = "Name=" + agent.name + ", tasks="
        for t in range(0, len(agent.tasks)):
            info += "(" + agent.tasks[t].name + ")"
            if t < len(agent.tasks)-1:
                info+=","
        print(info)

def PrintTaskNetwork(first_process):
    current_process = first_process
    while current_process != None:
        for task in current_process.tasks:
            task_info = task.name
            task_info += ", prec="
            for t in range(0,len(task.preceding_tasks)):
                task_info += "("+task.preceding_tasks[t].name+")"
                if t < len(task.preceding_tasks)-1:
                    task_info+=","

            task_info += ", succ="
            for t in range(0,len(task.succeeding_tasks)):
                task_info += "("+task.succeeding_tasks[t].name+")"
                if t < len(task.succeeding_tasks)-1:
                    task_info+=","

            task_info += ", task-start-prec-compl="
            if task.task_start_preceding_completion != None:
                task_info+="("+task.task_start_preceding_completion.name+")"

            task_info += ", task-compl-succ-start="
            if task.task_completion_succeeding_start != None:
                task_info+="("+task.task_completion_succeeding_start.name+")"

            task_info += ", task-start-succ-start="
            if task.task_start_succeeding_start != None:
                task_info+="("+task.task_start_succeeding_start.name+")"

            task_info += ", task-compl-succ-compl="
            if task.task_start_succeeding_start != None:
                task_info+="("+task.task_start_succeeding_start.name+")"

            if task.agent != None:
                task_info+=", agent="+task.agent.name
            print(task_info+"\n")
        current_process = current_process.NextProcess()

class Edge(object):
    def __init__(self, from_node, to_node):
        self.from_node = from_node
        self.to_node = to_node

def GetAgentTaskNodes(first_process, G, agent):
    node_color = {}
    node_size = {}
    node_pos = {}
    x_pos = 0.0

    cur = first_process
    while cur != None:
        y_pos = 0.0
        for task in cur.tasks:
            G.add_node(task.name)
            node_size[task.name] = 1000
            if agent == None or not agent.InListOf(task.team.agents):
                # Transparent orange
                # (R,G,B,alpha)
                node_color[task.name] = (1.0,0.644,0,0.5)
            else:
                # Transparent cyan
                # (R,G,B,alpha)
                node_color[task.name] = (0,1,1,0.5)
            node_pos[task.name] = [x_pos, y_pos]
            y_pos -= 1.0
        cur = cur.NextProcess()
        x_pos += 1.0

    if agent != None:
        x_pos = -1.0
        y_pos = 0.0
        G.add_node(agent.tasks[0].name)
        # Transparent cyan
        # (R,G,B,alpha)
        node_color[agent.tasks[0].name] = (0,1,1,0.5)
        node_size[agent.tasks[0].name] = 1000
        node_pos[agent.tasks[0].name] = [x_pos, y_pos]

    return node_pos, node_color, node_size


def GetAllTaskNodes(G, agents):
    node_color = {}
    node_size = {}
    node_pos = {}
    x_pos = 0.0

    y_pos = 0.0
    multi_robot_task_name = {}
    agent_name = {}
    for ag in agents:
        x_pos = -1.0
        for task in ag.tasks:
            if task.name in G or task.name in multi_robot_task_name:
                multi_robot_task_name[task.name] = True
                if task.name in G:
                    G.remove_node(task.name)
                    G.add_node(task.name + agent_name[task.name])
                    node_color[task.name + agent_name[task.name]] = node_color[task.name]
                    node_size[task.name + agent_name[task.name]] = node_size[task.name]
                    node_pos[task.name + agent_name[task.name]] = node_pos[task.name]
                task_name = task.name + ag.name
            else:
                task_name = task.name
            G.add_node(task_name)
            agent_name[task_name] = ag.name
            node_size[task_name] = 1000
            if task == ag.tasks[0]:
                # Transparent cyan
                # (R,G,B,alpha)
                node_color[task_name] = (0,1,1,0.5)
            else:
                # Transparent orange
                # (R,G,B,alpha)
                node_color[task_name] = (1.0,0.644,0,0.5)
            node_pos[task_name] = [x_pos, y_pos]
            x_pos += 1.0
        y_pos -= 1.0

    return node_pos, node_color, node_size, multi_robot_task_name

def GetAllPrecedenceEdges(task, visited, edge_color):
    all_edges = []
    if not(task.name in visited):
        if task.succeeding_tasks != []:
            for succ in task.succeeding_tasks:
                all_edges.append(Edge(task.name, succ.name))
                edge_color[(task.name, succ.name)] = "black"
                all_edges.extend(GetAllPrecedenceEdges(succ, visited, edge_color))
        if task.task_completion_succeeding_start != None:
            all_edges.append(Edge(task.name, task.task_completion_succeeding_start.name))
            edge_color[(task.name, task.task_completion_succeeding_start.name)] = "red"
            all_edges.extend(GetAllPrecedenceEdges(task.task_completion_succeeding_start, visited, edge_color))

        if task.task_start_succeeding_start != None and task.task_start_succeeding_start == task.task_completion_succeeding_completion:
            all_edges.append(Edge(task.name, task.task_start_succeeding_start.name))
            edge_color[(task.name, task.task_start_succeeding_start.name)] = "purple"
            all_edges.extend(GetAllPrecedenceEdges(task.task_start_succeeding_start, visited, edge_color))
        else:
            if task.task_completion_succeeding_completion != None:
                all_edges.append(Edge(task.name, task.task_completion_succeeding_completion.name))
                edge_color[(task.name, task.task_completion_succeeding_completion.name)] = "green"
                all_edges.extend(GetAllPrecedenceEdges(task.task_completion_succeeding_completion, visited, edge_color))

        visited[task.name] = True
    return all_edges

def GetAgentPrecedenceEdges(edge_color, agent):
    all_edges = []
    for task in agent.tasks:
        for succ in task.intra_schedule_succeeding_tasks:
            if agent.InListOf(succ.team.agents):
                all_edges.append(Edge(task.name, succ.name))
                edge_color[(task.name, succ.name)] = "black"
    return all_edges

def GetAgentTaskSequenceEdges(agent, edge_color, multi_robot_task_name, mrt_nodes):
    all_edges = []
    if len(agent.tasks) > 1:
        for i in range(1, len(agent.tasks)):
            prev_task = agent.tasks[i-1]
            this_task = agent.tasks[i]
            if prev_task.name in multi_robot_task_name:
                prev_name = prev_task.name + agent.name
            else:
                prev_name = prev_task.name
            if this_task.name in multi_robot_task_name:
                this_name = this_task.name + agent.name
                if this_task.name not in mrt_nodes:
                    mrt_nodes[this_task.name] = []
                mrt_nodes[this_task.name].append(this_name)
            else:
                this_name = this_task.name
            all_edges.append(Edge(prev_name, this_name))
            edge_color[(prev_name, this_name)] = "black"

    return all_edges

def GetAllProcessNodes(first_process, G):
    node_color = {}
    node_size = {}
    node_pos = {}
    node_name = {}

    cur = first_process
    x_pos = 0.0
    y_pos = 0.0
    while cur != None:
        # Order buffer node
        if cur == first_process:
            buf_name = cur.name + "i" + str(cur.input_buffer.buffer.capacity)
            G.add_node(buf_name)
            node_name[buf_name] = str(cur.input_buffer.buffer.capacity)
            node_size[buf_name] = 1000
            # Transparent cyan
            # (R,G,B,alpha)
            node_color[buf_name] = (0,1,1,0.5)
            node_pos[buf_name] = [x_pos, y_pos]
            x_pos += 1.0

        # Process node
        G.add_node(cur.name)
        node_name[cur.name] = cur.name
        node_size[cur.name] = 4000
        # Transparent orange
        # (R,G,B,alpha)
        node_color[cur.name] = (1.0,0.644,0,0.5)
        node_pos[cur.name] = [x_pos, y_pos]

        if cur.output_buffer != None:
            # Buffer node
            if cur.first_location != None or cur.IsFirstProcess():
                x_pos += 1.0
            else:
                # Output buffer is closely located to the process
                x_pos += 0.5
            buf_name = cur.name + "o" + str(cur.output_buffer.buffer.capacity)
            G.add_node(buf_name)
            node_name[buf_name] = str(cur.output_buffer.buffer.capacity)
            node_size[buf_name] = 1000
            # Transparent cyan
            # (R,G,B,alpha)
            node_color[buf_name] = (0,1,1,0.5)
            node_pos[buf_name] = [x_pos, y_pos]
            if cur.NextProcess().first_location != None:
                x_pos += 1.0
            else:
                # Output buffer is closely located to the NEXT process
                x_pos += 0.5

        cur = cur.NextProcess()

    return node_pos, node_color, node_size, node_name

def GetAllBufferEdges(process, visited, edge_color):
    all_edges = []
    if not(process.name in visited):
        if process.input_buffer != None:
            if process.IsFirstProcess():
                in_buf_name = process.name + "i" + str(process.input_buffer.buffer.capacity)
            else:
                in_buf_name = process.PrevProcess().name + "o" + str(process.input_buffer.buffer.capacity)
            all_edges.append(Edge(in_buf_name, process.name))
            edge_color[(in_buf_name, process.name)] = "black"
        if process.output_buffer != None:
            out_buf_name = process.name + "o" + str(process.output_buffer.buffer.capacity)
            all_edges.append(Edge(process.name, out_buf_name))
            edge_color[(process.name, out_buf_name)] = "black"
            all_edges.extend(GetAllBufferEdges(process.NextProcess(), visited, edge_color))

        visited[process.name] = True

    return all_edges

def PlotFactory(task_locations, robot_locations, ax, process_locations = []):

    ax.set_title("Factory layout")

    for loc in robot_locations:
        ax.plot(loc.x, loc.y, color='pink', marker='o')

    for node in task_locations:
        ax.plot(node.x, node.y, color='purple', marker='o', markersize=6)

    for proc in process_locations:
        ax.plot(proc.x, proc.y, color='green', marker='o', markersize=10)

def PlotCPM(first_process):
    G = nx.DiGraph()
    edge_color = {}
    node_pos, node_color, node_size, node_name = GetAllProcessNodes(first_process, G)
    pass
    visited = {}
    all_edges = GetAllBufferEdges(first_process, visited, edge_color)
    for edge in all_edges:
        G.add_edge(edge.from_node,edge.to_node)

    edg_lst = list(G.edges())
    edge_color_lst = []
    for tup in edg_lst:
        edge_color_lst.append(edge_color[tup])
    node_color_lst = []
    node_size_lst = []
    names = {}
    for node in G:
        node_color_lst.append(node_color[node])
        node_size_lst.append(node_size[node])
    nx.draw(G, node_pos, edgelist = edg_lst, node_size = node_size_lst, 
            node_color = node_color_lst, edge_color = edge_color_lst, labels = node_name,
            style='dotted', with_labels = True, node_shape = 's')

def PlotCPMTaskConstraints(first_process, agent = None):
    G = nx.DiGraph()
    edge_color = {}
    node_pos, node_color, node_size = GetAgentTaskNodes(first_process, G, agent)

    if len(first_process.tasks) > 0:
        if agent == None:
            task = first_process.tasks[0]
            visited = {}
            all_edges = GetAllPrecedenceEdges(task, visited, edge_color)
            for edge in all_edges:
                G.add_edge(edge.from_node,edge.to_node)
        else:
            edge_color_agent = {}
            agent_edges = GetAgentPrecedenceEdges(edge_color_agent, agent)
            for edge in agent_edges:
                if G.has_edge(edge.from_node,edge.to_node):
                    edge_color[(edge.from_node,edge.to_node)] = edge_color_agent[(edge.from_node,edge.to_node)]
                else:
                    G.add_edge(edge.from_node,edge.to_node)
                    edge_color[(edge.from_node,edge.to_node)] = edge_color_agent[(edge.from_node,edge.to_node)]
        edg_lst = list(G.edges())
        edge_color_lst = []
        for tup in edg_lst:
            edge_color_lst.append(edge_color[tup])
        node_color_lst = []
        node_size_lst = []
        for node in G:
            node_color_lst.append(node_color[node])
            node_size_lst.append(node_size[node])
        if agent == None:
            nx.draw(G, node_pos, edgelist = edg_lst, node_size = node_size_lst, 
                    node_color = node_color_lst, edge_color = edge_color_lst,
                    style='dotted', with_labels = True, node_shape = 's')
        else:
            # For presentation purpose of precedence constraints:
            # Reduce the set of edges in the agent graph into a minimum, using "Transitive Reduction".
            TR = nx.transitive_reduction(G)
            edg_lst = list(TR.edges())
            nx.draw(TR, node_pos, edgelist = edg_lst, node_size = node_size_lst, 
                node_color = node_color_lst, edge_color = edge_color_lst,
                style='dotted', with_labels = True, node_shape = 's')

def PlotCPMTaskSequences(agents):
    G = nx.DiGraph()
    edge_color = {}
    node_pos, node_color, node_size, multi_robot_task_name = GetAllTaskNodes(G, agents)

    mrt_nodes = {}
    edge_color = {}
    for agent in agents:
        agent_edges = GetAgentTaskSequenceEdges(agent, edge_color, multi_robot_task_name, mrt_nodes)
        # Adding intra-schedule edges between agent tasks
        for edge in agent_edges:
            G.add_edge(edge.from_node,edge.to_node)

    # Adding cross-schedule edges for multi robot tasks
    for node_lst in mrt_nodes.values():
        prev_node = None
        for node in node_lst:
            if prev_node != None:
                G.add_edge(prev_node, node)
                edge_color[(prev_node, node)] = "red"
            prev_node = node

    edg_lst = list(G.edges())
    edge_color_lst = []
    for tup in edg_lst:
        edge_color_lst.append(edge_color[tup])
    node_color_lst = []
    node_size_lst = []
    for node in G:
        node_color_lst.append(node_color[node])
        node_size_lst.append(node_size[node])

    nx.draw(G, node_pos, edgelist = edg_lst, node_size = node_size_lst, 
            node_color = node_color_lst, edge_color = edge_color_lst,
            style='dotted', with_labels = True, node_shape = 's')
            
def PlotTaskLocations(axs, first_process, agents):
    visited = {}
    locations, names = first_process.tasks[0].GetLocations(visited)

    for i in range(0, len(locations)):
        axs.plot(locations[i].x, locations[i].y, color='pink', marker='o')
        axs.annotate(locations[i].name,(locations[i].x, locations[i].y))

    for agent in agents:
        axs.plot(agent.tasks[0].subtasks[0].location.x, agent.tasks[0].subtasks[0].location.y, color='blue', marker='o')
        axs.annotate(agent.tasks[0].subtasks[0].location.name,(agent.tasks[0].subtasks[0].location.x, agent.tasks[0].subtasks[0].location.y))

def PlotRoutes(axs, agents):
    locations = {}
    location_tasks = {}
    for agent in agents:
        if len(agent.tasks) > 0:
            x_pos = []
            y_pos = []
            # Start location
            x_pos.append(agent.tasks[0].subtasks[-1].location.x)
            y_pos.append(agent.tasks[0].subtasks[-1].location.y)
            # Start locations task names
            axs.text(x_pos[-1], y_pos[-1], agent.tasks[0].name, fontsize=12)
            for i in range(1,len(agent.tasks)):
                for subtask in agent.tasks[i].subtasks:
                    x_pos.append(subtask.location.x)
                    y_pos.append(subtask.location.y)
                    locations[subtask.location.name] = subtask.location
                    if subtask.location.name not in location_tasks:
                        location_tasks[subtask.location.name] = []
                    location_tasks[subtask.location.name].append(agent.tasks[i].name)
            axs.plot(x_pos, y_pos, linestyle='--', label=agent.name)
    for loc_name in location_tasks:
        task_names = ""
        for task_name in location_tasks[loc_name]:
            task_names += task_name + "-"
        axs.text(locations[loc_name].x, locations[loc_name].y, task_names, fontsize=12)


def PlotStartAndCompletionDurations(agent, ax, cumulative, percentile, min_x = None, max_x = None, min_y = None, max_y = None):
    plot_max_time = 0.0
    plot_min_time = 9E9

    # Size of plot
    if min_x != None:
        plot_min_time = min_x
    else:
        for task in agent.tasks:
            dur = SDUR.StochasticDuration()
            dur.MultiMax([o.duration for o in task.completion_durations])
            plot_min_time = min(plot_min_time, dur.MinEstimate())
    if max_x != None:
        plot_max_time = max_x
    else:
        for task in agent.tasks:
            dur = SDUR.StochasticDuration()
            dur.MultiMax([o.duration for o in task.completion_durations])
            plot_max_time = max(plot_max_time, dur.MaxEstimate())
    ax.set_xlim([plot_min_time, plot_max_time])
    if min_y != None and max_y != None:
        ax.set_ylim([min_y, max_y])
    ax.grid(linestyle = 'dotted')

    start = int((plot_min_time - SDUR.min_dur)/(SDUR.max_dur - SDUR.min_dur) * SDUR.pmf_size)
    end = int((plot_max_time - SDUR.min_dur)/(SDUR.max_dur - SDUR.min_dur) * SDUR.pmf_size)

    # Plot all Monte Carlo durations
    for i in range(0, len(agent.tasks)):
        completion_time = SDUR.StochasticDuration()
        completion_time.MultiMax([o.duration for o in agent.tasks[i].completion_durations])
        if not completion_time.zero and hasattr(agent, 'normalized_ic_completion_time'):
            mcdur = SDUR.StochasticDuration()
            mcdur.SetGeneralDistribution(agent.normalized_ic_completion_time[i])
            if cumulative:
                ax.plot(SDUR.big_grid[start:end], mcdur.GetCumulative()[start:end], color='black') 
            else:
                ax.plot(SDUR.big_grid[start:end], mcdur.pmf[start:end], color='black')

        start_time = SDUR.StochasticDuration()
        start_time.MultiMax([o.duration for o in agent.tasks[i].start_durations])
        if start_time != None and not start_time.zero and hasattr(agent, 'normalized_ic_completion_time'):
            mcdur = SDUR.StochasticDuration()
            mcdur.SetGeneralDistribution(agent.normalized_ic_start_time[i])
            if cumulative:
                ax.plot(SDUR.big_grid[start:end], mcdur.GetCumulative()[start:end], linestyle='dashed', color='black')
            else:
                ax.plot(SDUR.big_grid[start:end], mcdur.pmf[start:end], linestyle='dashed', color='black')

    # Plot all explored durations
    for task in agent.tasks:
        if task.process != None:
            clr = task.process.color
        else:
            clr = 'black'
        # Plot start time
        if len(task.start_durations) > 0:
            dur = SDUR.StochasticDuration()
            dur.MultiMax([o.duration for o in task.start_durations])
            durlow = SDUR.StochasticDuration()
            durlow.MultiMaxLow([o.duration for o in task.start_durations])
            if dur != None and not dur.zero:
                if cumulative:
                    ax.plot(SDUR.big_grid[start:end], dur.GetCumulative()[start:end], linestyle='dashed', color=clr)
                    ax.plot(SDUR.big_grid[start:end], durlow.GetCumulative()[start:end], linestyle='dashed', color=clr)
                else:
                    ax.plot(SDUR.big_grid[start:end], dur.pmf[start:end], linestyle='dashed', color=clr)
                    ax.plot(SDUR.big_grid[start:end], durlow.pmf[start:end], linestyle='dashed', color=clr)

        # Plot completion time
        dur = SDUR.StochasticDuration()
        dur.MultiMax([o.duration for o in task.completion_durations])
        durlow = SDUR.StochasticDuration()
        durlow.MultiMaxLow([o.duration for o in task.completion_durations])
        if not dur.zero:
            if cumulative:
                ax.plot(SDUR.big_grid[start:end], dur.GetCumulative()[start:end], color=clr)
                ax.plot(SDUR.big_grid[start:end], durlow.GetCumulative()[start:end], color=clr)
            else:
                ax.plot(SDUR.big_grid[start:end], dur.pmf[start:end], color=clr)
                ax.plot(SDUR.big_grid[start:end], durlow.pmf[start:end], color=clr)
        ax.axvline(x=dur.AtPercentile(percentile), color=clr)

    # Set vertical limits
    if cumulative:
        ax.set_ylim([0, 1])
    else:
        ax.set_ylim([0, 0.02])

    # Plot all task names separated with a vertical offset (y_step)
    y_lim = ax.get_ylim()
    y_step = (y_lim[1] - y_lim[0]) / (len(agent.tasks)+1)
    y_pos = y_lim[0]
    for task in agent.tasks:
        if task.process != None:
            clr = task.process.color
        else:
            clr = 'black'

        task_name = task.name
        # Mark a non-exact solution with '*'
        if len(task.completion_durations) > 1:
            task_name += '*'
        # Mark human overseer task with 'o'
        if agent.human and task.process != None and task.process.team_formations[task.team_formation_index].no_of_overseers > 0:
            task_name += 'o'
        dur = SDUR.StochasticDuration()
        dur.MultiMax([o.duration for o in task.completion_durations])
        ax.text(dur.AtPercentile(percentile), y_pos, task_name, fontsize=12, color=clr)
        y_pos += y_step

    ax.axhline(y=0, color='black')

    return plot_min_time, plot_max_time, ax.get_ylim()

def PlotOtherDurations(agent, ax, cumulative, plot_task_activity_durations, makespan_low, min_x = None, max_x = None, min_y = None, max_y = None):
    plot_max_time = 0.0
    plot_min_time = 9E9

    # Size of plot
    if min_x != None:
        plot_min_time = min_x
    else:
        for task in agent.tasks:
            plot_min_time = min(plot_min_time, task.completion_time.MinEstimate())
    if max_x != None:
        plot_max_time = max_x
    else:
        for task in agent.tasks:
            plot_max_time = max(plot_max_time, task.completion_time.MaxEstimate())
    ax.set_xlim([plot_min_time, plot_max_time])
    if min_y != None and max_y != None:
        ax.set_ylim([min_y, max_y])
    else:
        if cumulative:
            ax.set_ylim([0, 1])
        else:
            ax.set_ylim([0, 0.003])

    ax.grid(linestyle = 'dotted')

    start = int((plot_min_time - SDUR.min_dur)/(SDUR.max_dur - SDUR.min_dur) * SDUR.pmf_size)
    end = int((plot_max_time - SDUR.min_dur)/(SDUR.max_dur - SDUR.min_dur) * SDUR.pmf_size)

    if plot_task_activity_durations:
        # Activity times
        y_lim = ax.get_ylim()
        y_pos = y_lim[0]
        y_step = (y_lim[1] - y_lim[0]) / (len(agent.task_accumulated_activity_time) + 1)
        # Plot task name
        for task in agent.task_accumulated_activity_time:
            ax.text(agent.task_accumulated_activity_time[task].Median(), y_pos, task, fontsize=12)
            y_pos += y_step

        for task in agent.tasks:
            # Monte Carlo activity time
            if hasattr(agent, 'normalized_ic_activity_time'):
                clr = 'black'
                mcact = SDUR.StochasticDuration()
                mcact.SetGeneralDistribution(agent.normalized_ic_activity_time[agent.tasks.index(task)])
                if cumulative:
                    ax.plot(SDUR.big_grid[start:end], mcact.GetCumulative()[start:end], color='black') 
                else:
                    ax.plot(SDUR.big_grid[start:end], mcact.pmf[start:end], color='black')

            # Computed activity time
            if task.name in agent.task_accumulated_activity_time:
                if task.process != None:
                    clr = task.process.color
                else:
                    clr = 'black'
                if not agent.task_accumulated_activity_time[task.name].zero:
                    if cumulative:
                        ax.plot(SDUR.big_grid[start:end], agent.task_accumulated_activity_time[task.name].GetCumulative()[start:end], linestyle='dotted', color=clr, zorder=10)
                    else:
                        ax.plot(SDUR.big_grid[start:end], agent.task_accumulated_activity_time[task.name].pmf[start:end], linestyle='dotted', color=clr, zorder=10)
                ax.axvline(x=agent.task_accumulated_activity_time[task.name].Median(), color=clr)
    
        # Plot total preferred activity time
        if not agent.total_preferred_activity_time.zero:
            if cumulative:
                ax.plot(SDUR.big_grid[start:end], agent.total_preferred_activity_time.GetCumulative()[start:end], linestyle='dotted', color='black', zorder=10)
            else:
                ax.plot(SDUR.big_grid[start:end], agent.total_preferred_activity_time.pmf[start:end], linestyle='dotted', color='black', zorder=10)
            ax.axvline(x=agent.total_preferred_activity_time.Median(), color='black')
            ax.text(agent.total_preferred_activity_time.Median(), y_lim[0], "total preferred activity time", fontsize=12)
        # Plot total activity time
        if not agent.total_activity_time.zero:
            # Just the text.
            # No need to plot, this is already covered by the accumulated activity time of the last task
            ax.text(agent.total_activity_time.Median(), y_lim[0], "total activity time", fontsize=12)
    else:
        # Idle times
        y_lim = ax.get_ylim()
        if hasattr(agent, 'task_accumulated_idle_time_sim'):
            y_pos = y_lim[0]
            y_step = (y_lim[1] - y_lim[0]) / (len(agent.task_accumulated_idle_time_sim) + 1)
            for task in agent.tasks:
                # Monte Carlo idle time
                if task.name in agent.task_accumulated_idle_time_sim:
                    mcidle = SDUR.StochasticDuration()
                    mcidle.SetGeneralDistribution(agent.normalized_ic_idle_time[agent.tasks.index(task)])
                    if cumulative:
                        ax.plot(SDUR.big_grid[start:end], mcidle.GetCumulative()[start:end], color='black') 
                    else:
                        ax.plot(SDUR.big_grid[start:end], mcidle.pmf[start:end], color='black')

                    if task.process != None:
                        clr = task.process.color
                    else:
                        clr = 'black'
                    ax.text(mcidle.Median(), y_pos, task.name, color=clr, fontsize=12)
                    y_pos += y_step
        # Total Monte Carlo idle time
        if hasattr(agent, 'normalized_ic_total_idle_time'):
            mctotidle = SDUR.StochasticDuration()
            mctotidle.SetGeneralDistribution(agent.normalized_ic_total_idle_time)
            if cumulative:
                ax.plot(SDUR.big_grid[start:end], mctotidle.GetCumulative()[start:end], color='black') 
            else:
                ax.plot(SDUR.big_grid[start:end], mctotidle.pmf[start:end], color='black')
            ax.axvline(x=mctotidle.Median(), color='black')

        # Total computed idle time
        total_idle_time = agent.total_activity_time.Clone()
        total_idle_time.SetNegative()
        total_idle_time.Conv(makespan_low)
        if not total_idle_time.zero:
            if cumulative:
                ax.plot(SDUR.big_grid[start:end], total_idle_time.GetCumulative()[start:end], color='pink') 
            else:
                ax.plot(SDUR.big_grid[start:end], total_idle_time.pmf[start:end], color='pink')
        ax.axvline(x=makespan_low.Median() - agent.total_activity_time.Median(), color='pink')
        ax.text(total_idle_time.Median(), y_lim[0], "total idle time", fontsize=12)


    return plot_min_time, plot_max_time, ax.get_ylim()

def PlotDurationList(axs, durations, cumulative, labels, legend = False, line_style = '-', title_str = None):
    plot_min_time = 9E9
    plot_max_time = -9E9
    if title_str != None:
        axs.set_title(title_str, fontsize=14)
    for i in range(0, len(durations)):
        if cumulative:
            axs.plot(SDUR.big_grid, durations[i].GetCumulative(), label=labels[i], linestyle = line_style)
        else:
            axs.plot(SDUR.big_grid, durations[i].pmf, label=labels[i], linestyle = line_style)
        offset = 0.2*(durations[i].MaxEstimate() - durations[i].MinEstimate())
        plot_min_time = min(plot_min_time, durations[i].MinEstimate() - offset)
        plot_max_time = max(plot_max_time, durations[i].MaxEstimate() + offset)
    if legend:
        axs.legend(fontsize=16)
    axs.set_xlim([plot_min_time, plot_max_time])

def PlotTask(last_task, dur, durlow, dur_det, proc1, ax_cdf, x_label, y_label, legend, kld_val):
    # Size of plot
    plot_min_time = min(dur.MinEstimate(), durlow.MinEstimate())
    plot_max_time = max(dur.MaxEstimate(), durlow.MaxEstimate())

    plot_min_time -= 0.1*(plot_max_time - plot_min_time)
    plot_max_time += 0.1*(plot_max_time - plot_min_time)

    ax_cdf.set_xlim([plot_min_time, plot_max_time])
    ax_cdf.grid(linestyle = 'dotted')

    start = int((plot_min_time - SDUR.min_dur)/(SDUR.max_dur - SDUR.min_dur) * SDUR.pmf_size)
    end = int((plot_max_time - SDUR.min_dur)/(SDUR.max_dur - SDUR.min_dur) * SDUR.pmf_size)

    if hasattr(proc1, 'normalized_ic_makespan'):
        mcdur = SDUR.StochasticDuration()
        mcdur.SetGeneralDistribution(proc1.normalized_ic_makespan)

    filter_step_cdf = 4

    # Plot completion duration CDFs
    for cd in last_task.completion_durations:
        if cd == last_task.completion_durations[0]:
            # With label
            ax_cdf.plot(SDUR.big_grid[start:end][::filter_step_cdf], cd.duration.GetCumulative()[start:end][::filter_step_cdf], color='pink', label=r'$C(\tau_L)$', zorder=0)
        else:
            # No label
            ax_cdf.plot(SDUR.big_grid[start:end][::filter_step_cdf], cd.duration.GetCumulative()[start:end][::filter_step_cdf], color='pink', zorder=0)

    # Plot makespan bound CDFs
    ax_cdf.plot(SDUR.big_grid[start:end][::filter_step_cdf], dur.GetCumulative()[start:end][::filter_step_cdf], color='blue', label=r'$K_e(\tau_L)$', zorder=3)
    ax_cdf.plot(SDUR.big_grid[start:end][::filter_step_cdf], durlow.GetCumulative()[start:end][::filter_step_cdf], color='red', label=r'$K_l(\tau_L)$', zorder=3)

    ax_cdf.fill_between(SDUR.big_grid[start:end][::filter_step_cdf], durlow.GetCumulative()[start:end][::filter_step_cdf], dur.GetCumulative()[start:end][::filter_step_cdf], color='lightgrey', alpha=0.5, zorder=2)

    # Plot Monte Carlo makespan
    if hasattr(proc1, 'normalized_ic_makespan'):
        ax_cdf.plot(SDUR.big_grid[start:end][::filter_step_cdf], mcdur.GetCumulative()[start:end][::filter_step_cdf], color='black', linestyle='dashed', label='Monte Carlo', zorder=3)

    # Plot median makespans
    ax_cdf.axvline(x=durlow.Median(), color='red', zorder=1)
    ax_cdf.axvline(x=dur.Median(), color='blue', zorder=1)
    # Plot deterministic makespan
    ax_cdf.axvline(x=dur_det, color='limegreen', linestyle='dashed', label='Deterministic', zorder=1)
    if hasattr(proc1, 'normalized_ic_makespan'):
        ax_cdf.axvline(x=mcdur.Median(), color='black', linestyle='dashed', zorder=1)

    textstr = "KLD = " + str(round(kld_val,4))
    props = dict(facecolor='white', alpha=0.5)
    ax_cdf.text(plot_max_time - 0.03*(plot_max_time-plot_min_time), 0.0, textstr, color='black', horizontalalignment='right', verticalalignment='bottom', bbox=props)

    if x_label:
        ax_cdf.set_xlabel("Makespan duration (s)")
    if y_label:
        ax_cdf.set_ylabel("CDF")

    if legend:
        ax_cdf.legend(loc='upper left')

