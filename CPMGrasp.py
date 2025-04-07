import numpy as np
import CPM
from timeit import default_timer as timer

debug_mode = False

class Element(object):
    def __init__(self, task, team, cost, makespan, agent_start, agent_completion, agent_activities, agent_preferred):
        self.task = task
        self.team = team
        self.cost = cost
        self.makespan = makespan
        self.agent_start = agent_start
        self.agent_completion = agent_completion
        self.agent_activities = agent_activities
        self.agent_preferred = agent_preferred

class Team(object):
    def __init__(self, tf_index):
        self.tf_index = tf_index
        self.agents = set()

    def AddAgent(self, agent):
        self.agents.add(agent)

    def TaskSequencesOK(self, task):
        for agent in self.agents:
            for t in agent.tasks:
                if task in t.intra_schedule_preceding_tasks:
                    return False
        return True

class Solution(object):
    def __init__(self, proc1, agents, task_list, cost, makespan_cost, makespan_median, makespan_low_median):
        self.proc1 = proc1
        self.agents = agents
        self.task_list = task_list
        self.cost = cost
        self.makespan_cost = makespan_cost
        self.makespan_median = makespan_median
        self.makespan_low_median = makespan_low_median
        self.agent_tasks = {}
        self.task_tf_index = {}
        self.task_agents = {}
        for a in agents:
            self.agent_tasks[a.name] = a.tasks.copy()
        for t in task_list:
            self.task_tf_index[t.name] = t.team_formation_index
            self.task_agents[t.name] = []
            for a in t.team.agents:
                self.task_agents[t.name].append(a.agent)

    def Apply(self):
        CPM.PrepareForNewSolution(self.proc1, self.agents)
        for a in self.agents:
            a.tasks = self.agent_tasks[a.name].copy()
        for t in self.task_list:
            t.team_formation_index = self.task_tf_index[t.name]
            for a in self.task_agents[t.name]:
                t.team.AllocateAgent(a)

class GR(object):
    def __init__(self, proc1, agents, percentile, no_of_iterations, rand_gen):
        # Model
        self.proc1 = proc1
        self.agents = agents
        self.percentile = percentile
        self.NO_OF_ITERATIONS = no_of_iterations
        self.human_agents = []
        self.robot_agents = []
        for agent in self.agents:
            if agent.human:
                self.human_agents.append(agent)
            else:
                self.robot_agents.append(agent)
        self.rand_gen = rand_gen
        self.no_of_agents = len(agents)
        self.no_of_tasks = 0
        # Create an indexed list with references to all tasks
        self.task_list = []
        proc = self.proc1
        while proc != None:
            for t in proc.tasks:
                if len(self.task_list) != t.index:
                    raise Exception("WARNING: Task index mismatch")
                self.task_list.append(t)
            proc = proc.NextProcess()
        self.no_of_tasks = len(self.task_list)
        self.last_task = proc1.LastProcess().tasks[-1]

        # Identify applicable teams for each activity
        # A team is a combination of a team formation and agents
        self.teams = {}
        proc = proc1
        while proc != None:
            self.teams[proc.name] = []
            for tf_index in range(0, len(proc.team_formations)):
                tf = proc.team_formations[tf_index]
                if tf.no_of_robots == 1 and tf.no_of_humans == 0 and tf.no_of_overseers == 0:
                    for agent in self.robot_agents:
                        # Create Team
                        team = Team(tf_index)
                        team.AddAgent(agent)
                        self.teams[proc.name].append(team)
                elif tf.no_of_robots == 0 and tf.no_of_humans == 1 and tf.no_of_overseers == 0:
                    for agent in self.human_agents:
                        # Create Team
                        team = Team(tf_index)
                        team.AddAgent(agent)
                        self.teams[proc.name].append(team)
                elif tf.no_of_robots == 1 and tf.no_of_humans == 1 and tf.no_of_overseers == 0:
                    for rob in self.robot_agents:
                        for hum in self.human_agents:
                            # Create Team
                            team = Team(tf_index)
                            team.AddAgent(rob)
                            team.AddAgent(hum)
                            self.teams[proc.name].append(team)
                elif tf.no_of_robots == 1 and tf.no_of_humans == 0 and tf.no_of_overseers == 1:
                    for rob in self.robot_agents:
                        for hum in self.human_agents:
                            # Create Team
                            team = Team(tf_index)
                            team.AddAgent(rob)
                            team.AddAgent(hum)
                            self.teams[proc.name].append(team)
            proc = proc.NextProcess()


    def SetTransitionCosts(self, transition_costs, deterministic_costs):
        self.transition_costs = transition_costs
        self.deterministic_costs = deterministic_costs

    def RunAlgorithm(self, run_index, timeout, log_file = None):
        if log_file != None:
            try:
                fd=open("logs//" + log_file + ".log",'a')
            except:
                fd = None
        else:
            fd = None
        best_solutions = []
        no_of_feasible = 0
        iter_time = timer()
        start_time = iter_time
        cost_time = 0
        cost_count = 0
        for i in range(0, self.NO_OF_ITERATIONS):
            available_tasks = set(self.task_list)
            agent_start = {}
            agent_completion = {}
            agent_activities = {}
            agent_preferred = {}
            for agent in self.agents:
                agent_start[agent.name] = 0.0
                agent_completion[agent.name] = 0.0
                agent_activities[agent.name] = 0.0
                if agent.human:
                    agent_preferred[agent.name] = 0.0
            makespan = 0.0
            # Add first task
            available_tasks.add(self.proc1.tasks[0])
            allocated_tasks = set()
            CPM.PrepareForNewSolution(self.proc1, self.agents)
            while len(available_tasks) != 0:
                # Filter available tasks
                available2 = available_tasks.copy()
                for task1 in available2:
                    remove = False
                    for task2 in available_tasks:
                        if task1 in task2.intra_schedule_succeeding_tasks:
                            remove = True
                            break
                    if remove:
                        available_tasks.remove(task1)

                elements = []
                for task in available_tasks:
                    # Create an element for each task/team combination
                    for team in self.teams[task.process.name]:
                        # if team.TaskSequencesOK(task):
                        cost, new_makespan, new_agent_start, new_agent_completion, new_agent_activities, new_agent_preferred = \
                            self.ComputeHeuristic(task, team, agent_start, agent_completion, agent_activities, agent_preferred, makespan)
                        elements.append(Element(task, team, cost, new_makespan, new_agent_start, new_agent_completion, new_agent_activities, new_agent_preferred))

                # Sort the elements
                elements.sort(key=lambda x: x.cost)

                index = np.random.choice([0,1,2,3], p=[8/15, 4/15, 2/15, 1/15])

                # Randomly select one of the best elements
                selected_element = elements[index]

                # Apply the element
                makespan = selected_element.makespan
                for agent_name in selected_element.agent_start:
                    agent_start[agent_name] = selected_element.agent_start[agent_name]
                for agent_name in selected_element.agent_completion:
                    agent_completion[agent_name] = selected_element.agent_completion[agent_name]
                for agent_name in selected_element.agent_activities:
                    agent_activities[agent_name] = selected_element.agent_activities[agent_name]
                for agent_name in selected_element.agent_preferred:
                    agent_preferred[agent_name] = selected_element.agent_preferred[agent_name]
                
                # Insert the element into agent schedules
                selected_element.task.team_formation_index = selected_element.team.tf_index
                for agent in selected_element.team.agents:
                    selected_element.task.team.AllocateAgent(agent)
                    agent.tasks.append(selected_element.task)
                allocated_tasks.add(selected_element.task)

                available_tasks = set(self.task_list) - allocated_tasks

            # Evaluate solution cost
            dependency_order,_ = CPM.RepairInterScheduleDeadLocks(self.proc1, self.agents)
            cost_time -= timer()
            makespan, makespan_low = self.last_task.ExploreMakespan(self.transition_costs, self.deterministic_costs, dependency_order, self.agents)
            cost_time += timer()
            cost_count += 1
            feasible, cost, makespan_cost, makespan_median, makespan_low_median, _, _ = CPM.SolutionCost(makespan, makespan_low, self.agents, self.percentile)
            if feasible and (best_solutions == [] or cost < best_solutions[0].cost):
                try:
                    fd_seq=open("logs//" + log_file + "_seq.log",'a')
                except:
                    fd_seq = None
                print(("%d %d %.2f %.2f %.2f %.2f %.2f"%(run_index, i+1, cost, makespan_cost, makespan_median, makespan_low_median, timer() - start_time)).replace(".",","), file=fd_seq)
                if fd_seq != None:
                    fd_seq.close()

            if feasible or self.NO_OF_ITERATIONS == 1:
                no_of_feasible += 1
                if len(best_solutions) < 10:
                    best_solutions.append(Solution(self.proc1, self.agents, self.task_list, cost, makespan_cost, makespan_median, makespan_low_median))
                    best_solutions.sort(key=lambda x: x.cost)
                else:
                    if cost < best_solutions[-1].cost:
                        best_solutions.remove(best_solutions[-1])
                        best_solutions.append(Solution(self.proc1, self.agents, self.task_list, cost, makespan_cost, makespan_median, makespan_low_median))
                    best_solutions.sort(key=lambda x: x.cost)

            if max(i,1)/100 == max(i,1)//100:
                print("Run %d, iter %d, itertime %.2f, cost %.2f, makespan %.2f, msmedian %.2f, mslowmedian %.2f, #feasible %d"%(run_index, i, timer() - iter_time, best_solutions[0].cost, best_solutions[0].makespan_cost, best_solutions[0].makespan_median, best_solutions[0].makespan_low_median, no_of_feasible), file=fd)
                print("Run %d, iter %d, itertime %.2f, cost %.2f, makespan %.2f, msmedian %.2f, mslowmedian %.2f, #feasible %d"%(run_index, i, timer() - iter_time, best_solutions[0].cost, best_solutions[0].makespan_cost, best_solutions[0].makespan_median, best_solutions[0].makespan_low_median, no_of_feasible))
                iter_time = timer()

            if timer() - start_time >= timeout:
                if fd != None:
                    fd.close()
                break

        # Apply the best solution
        best_solutions[0].Apply()
        return cost_count, cost_time

    def ComputeHeuristic(self, task, team, agent_start, agent_completion, agent_activities, agent_preferred, makespan):
        # Agent time, since the start of the plan
        agent_arrival = {}
        new_agent_start = {}
        new_agent_completion = {}
        new_agent_activities = {}
        new_agent_preferred = {}

        # Task duration
        team_task_duration = task.subtasks[0].duration[team.tf_index].median
        prev_team_location = task.subtasks[0].location
        for i in range(1, len(task.subtasks)):
            if prev_team_location.name != task.subtasks[i].location.name:
                team_task_duration += self.transition_costs[prev_team_location.name, task.subtasks[i].location.name].median
            prev_team_location = task.subtasks[i].location
            team_task_duration += task.subtasks[i].duration[team.tf_index].median

        # Consider agent's previous completion time
        for agent in team.agents:
            agent_arrival[agent.name] = agent_completion[agent.name]
            new_agent_activities[agent.name] = agent_activities[agent.name]
            if agent.human:
                new_agent_preferred[agent.name] = agent_preferred[agent.name]

        # Consider routing durations
        for agent in team.agents:
            agent_routing_duration = self.transition_costs[agent.tasks[-1].subtasks[-1].location.name, task.subtasks[0].location.name].median
            agent_arrival[agent.name] += agent_routing_duration
            new_agent_activities[agent.name] += agent_routing_duration
            if agent.human:
                if agent.IsPreferredTeamFormation(task, team.tf_index):
                    new_agent_preferred[agent.name] += agent_routing_duration

        # Consider arrival of all team agents
        team_start = max(agent_arrival.values())

        # Consider task completion preceding start
        for cps in task.preceding_tasks:
            cps_agent = cps.team.agents[0]
            team_start = max(team_start, agent_completion[cps_agent.agent.name])

        # Consider task start preceding start
        if task.task_start_preceding_start != None:
            sps_agent = task.task_start_preceding_start.team.agents[0]
            team_start = max(team_start, agent_start[sps_agent.agent.name])

        # Agent task start
        for agent in team.agents:
            new_agent_start[agent.name] = team_start

        # Consider task duration
        team_completion = team_start + team_task_duration
        for agent in team.agents:
            new_agent_activities[agent.name] += team_task_duration
            if agent.human and agent.IsPreferredTeamFormation(task, team.tf_index):
                new_agent_preferred[agent.name] += team_task_duration

        # Consider task start preceding completion
        if task.task_start_preceding_completion != None:
            spc_agent = task.task_start_preceding_completion.team.agents[0]
            team_completion = max(team_completion, agent_start[spc_agent.agent.name])

        # Consider task completion preceding completion
        if task.task_completion_preceding_completion != None:
            cpc_agent = task.task_completion_preceding_completion.team.agents[0]
            team_completion = max(team_completion, agent_completion[cpc_agent.agent.name])

        # Agent task completion
        for agent in team.agents:
            new_agent_completion[agent.name] = team_completion 

        # Makespan
        makespan_increase = 0
        if team_completion - makespan > 0:
            # Increased makespan
            makespan_increase = team_completion - makespan
            new_makespan = team_completion
        else:
            # Unchanged makespan
            new_makespan = makespan

        # Balancing idle time and preferred activities
        idle_time_penalty = 0
        preferred_penalty = 0
        for agent in team.agents:
            if agent.human:
                idle_time_penalty += max(0, new_agent_activities[agent.name] - new_makespan * (1 - agent.idle_min))
                preferred_penalty += max(0, agent.preference_min * new_agent_activities[agent.name] - new_agent_preferred[agent.name])

        cost = makespan_increase + idle_time_penalty + preferred_penalty

        return cost, new_makespan, new_agent_start, new_agent_completion, new_agent_activities, new_agent_preferred
