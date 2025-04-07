import numpy as np
import CPM
import CPMGrasp
from timeit import default_timer as timer

debug_mode = False

class TaskData(object):
    def _init_(self):
        pass

class CHROMOSOME(object):
    def __init__(self, ancestor, no_of_agents = None, no_of_tasks = None):
        self.total_cost = 0.0
        self.makespan_cost = 0.0
        self.makespan_median = 0.0
        self.makespan_low_median = 0.0
        self.idle_time_median = []
        self.total_activity_time_median = []
        self.total_preferred_activity_time_median = []
        self.individual = []
        self.indTaskData = []
        self.numTasks = []
        self.ancestor = ancestor
        self.feasible = True
        self.idle_quota = {}
        self.pref_quota = {}
        if no_of_agents != None and no_of_tasks != None:
            # Memory allocations
            self.numTasks.extend([0]*no_of_agents)
            self.idle_time_median.extend([0.0]*no_of_agents)
            self.total_activity_time_median.extend([0.0]*no_of_agents)
            self.total_preferred_activity_time_median.extend([0.0]*no_of_agents)
            for i in range(0, no_of_agents):
                self.individual.append(np.array([-1]*(no_of_agents + no_of_tasks)))
            for i in range(0, no_of_tasks):
                self.indTaskData.append(TaskData())

    def Clone(self):
        ret = CHROMOSOME(self.ancestor)
        ret.total_cost = self.total_cost
        ret.makespan_cost = self.makespan_cost
        ret.makespan_median = self.makespan_median
        ret.makespan_low_median = self.makespan_low_median
        ret.idle_time_median = self.idle_time_median.copy()
        ret.total_activity_time_median = self.total_activity_time_median.copy()
        ret.total_preferred_activity_time_median = self.total_preferred_activity_time_median.copy()
        for i in range(0, len(self.individual)):
            ret.individual.append(np.copy(self.individual[i]))
        for i in range(0, len(self.indTaskData)):
            ret.indTaskData.append(TaskData())
            ret.indTaskData[i].process = self.indTaskData[i].process
            ret.indTaskData[i].team_formation = self.indTaskData[i].team_formation
        ret.numTasks = self.numTasks.copy()
        ret.feasible = self.feasible
        ret.idle_quota = self.idle_quota.copy()
        ret.pref_quota = self.pref_quota.copy()
        return ret

class GA(object):
    def __init__(self, proc1, agents, percentile, population, generations, rand_gen, warm_start):
        # Model
        self.proc1 = proc1
        self.agents = agents
        self.percentile = percentile
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
        self.last_task = proc1.LastProcess().tasks[-1]
        self.no_of_tasks = len(self.task_list)
        self.POPULATION_SIZE = population
        self.GENERATIONS = generations
        self.pMUTATION = 0.3
        self.pELITE = 0.05
        self.ELITE_SIZE = max(1, int(self.pELITE*self.POPULATION_SIZE))
        self.warm_start = warm_start

    def SetTransitionCosts(self, transition_costs, deterministic_costs):
        self.transition_costs = transition_costs
        self.deterministic_costs = deterministic_costs

    def RunAlgorithm(self, run_index, timeout, log_file):
        self.run_index = run_index
        self.population = []
        self.elitePopulation = []
        self.cost_time = 0
        self.cost_count = 0
        start_time = timer()

        if self.warm_start:
            Grasp = CPMGrasp.GR(self.proc1, self.agents, self.percentile, 1, self.rand_gen)
            Grasp.SetTransitionCosts(self.transition_costs, self.deterministic_costs)

        # Creation of initial schedules
        for i in range(0, self.POPULATION_SIZE):
            self.population.append(CHROMOSOME(i, self.no_of_agents, self.no_of_tasks))

            if self.warm_start:
                # Generate a randomized solution with GRASP heuristics
                Grasp.RunAlgorithm(0, 0)

                self.CpmToChromo(self.population[-1])
                # Evaluate solution
                self.evaluate(self.population[-1])
            else:
                # Random solution
                self.randomizeSolution(self.population[-1])

                # Repair solution
                self.ChromoToCpm(self.population[-1])
                CPM.RepairAgentTaskSequences(self.agents)
                dependency_order, _ = CPM.RepairInterScheduleDeadLocks(self.proc1, self.agents)

                # Evaluate solution
                self.evaluate(self.population[-1], dependency_order)

                # Update solution after repair
                self.CpmToChromo(self.population[-1])
                if debug_mode:
                    print("Solution generated " + str(i) + "\n")

        # Sort the population (best first)
        self.population.sort(key = lambda x: x.total_cost)

        # Find the best feasible solution
        best_feasible_solution = None
        for f in range(0, self.POPULATION_SIZE):
            if self.population[f].feasible:
                best_feasible_solution = self.population[f].Clone()
                break
            elif self.POPULATION_SIZE == 1:
                best_feasible_solution = self.population[f].Clone()

        total_updated = self.POPULATION_SIZE
        if best_feasible_solution != None:
            self.LogNewSolution(best_feasible_solution, run_index, 1, total_updated, timer()-start_time, log_file)

        self.LogPopulation(1, self.POPULATION_SIZE, total_updated, best_feasible_solution, timer() - start_time, log_file)
        self.LogPopulation(1, self.POPULATION_SIZE, total_updated, best_feasible_solution, timer() - start_time)

        # Optimize over a number of generations
        for i in range(0, self.GENERATIONS - 1):
            no_of_updated = 0
            exec_time = timer()
            for j in range(0, self.POPULATION_SIZE):
                individual_cloned = False    # Elite individuals are cloned to remain after mutation
                chromo_updated = False       # Avoid recomputing the cost if the chromosome does not change
 
                # Perform mutations

                # The idle time mutation should be done first since it relies on (unmutated) idle time data
                if self.rand_gen.randomUC.random() < self.pMUTATION:
                    if j < self.ELITE_SIZE and not individual_cloned:
                        self.elitePopulation.append(self.population[j].Clone())
                        if self.TaskToIdleMutation(self.population[j]):
                            individual_cloned = True
                            chromo_updated = True
                        else:
                            del self.elitePopulation[-1]
                    elif self.TaskToIdleMutation(self.population[j]):
                        chromo_updated = True

                # Team formation mutation
                if self.rand_gen.randomUC.random() < self.pMUTATION:
                    if j < self.ELITE_SIZE and not individual_cloned:
                        self.elitePopulation.append(self.population[j].Clone())
                        individual_cloned = True
                    self.TeamFormationMutation(self.population[j])
                    chromo_updated = True

                # Insert task mutation
                if self.rand_gen.randomUC.random() < self.pMUTATION:
                    if j < self.ELITE_SIZE and not individual_cloned:
                        self.elitePopulation.append(self.population[j].Clone())
                        if self.InsertTaskMutation(self.population[j]):
                            individual_cloned = True
                            chromo_updated = True
                        else:
                            del self.elitePopulation[-1]
                    elif self.InsertTaskMutation(self.population[j]):
                        chromo_updated = True

                # Swap task mutation
                if self.rand_gen.randomUC.random() < self.pMUTATION:
                    if j < self.ELITE_SIZE and not individual_cloned:
                        self.elitePopulation.append(self.population[j].Clone())
                        if self.SwapTaskMutation(self.population[j]):
                            individual_cloned = True
                            chromo_updated = True
                        else:
                            del self.elitePopulation[-1]
                    elif self.SwapTaskMutation(self.population[j]):
                        chromo_updated = True

                if individual_cloned:
                    # Did it really change (after multiple mutations) ?
                    if str(self.population[j].individual) == str(self.elitePopulation[-1].individual):
                        del self.elitePopulation[-1]
                        chromo_updated = False

                if chromo_updated:
                    no_of_updated += 1
                    # A check to find bugs in the update of chromosomes (to be removed)
                    self.CheckChromoConsistency(self.population[j])

                    # Apply the solution
                    self.ChromoToCpm(self.population[j])

                    # A check to find if mutations violate the precedence constraints (to be removed)
                    if CPM.RepairAgentTaskSequences(self.agents):
                        raise Exception("Task sequence order violated by mutations")

                    # Detect and repair deadlocks
                    dependency_order, moved_tasks = CPM.RepairInterScheduleDeadLocks(self.proc1, self.agents)

                    # Adjust solution for performed deadlock repairs
                    self.AdjustChromoFromDeadlockRepair(self.population[j], moved_tasks)

                    # A check to find mismatch bug between the representations (to be removed)
                    self.CompareChromoWithCPM(self.population[j])

                    # Evaluate solution cost
                    self.evaluate(self.population[j], dependency_order)
                    
            # Sort the population (best first)
            self.population.sort(key = lambda x: x.total_cost)

            # Insert the mutated part of the elite into the population, while keeping the sorted order
            cur_index = 0
            for e in range(0, len(self.elitePopulation)):
                while self.population[cur_index].total_cost < self.elitePopulation[e].total_cost:
                    cur_index += 1
                    if cur_index == len(self.population):
                        break
                if cur_index == len(self.population):
                    print("WARNING: Remaining elite are worse than any individual in the population")
                    break
                if str(self.elitePopulation[e].individual) != str(self.population[cur_index].individual):
                    self.population.insert(cur_index, self.elitePopulation[e])

            # Cut of the surplus population and reset the elite
            self.population = self.population[0:self.POPULATION_SIZE]

            # Find the best feasible solution
            new_sol = False
            for f in range(0, self.POPULATION_SIZE):
                if self.population[f].feasible:
                    if best_feasible_solution != None:
                        if self.population[f].total_cost < best_feasible_solution.total_cost:
                            best_feasible_solution = self.population[f].Clone()
                            new_sol = True
                    else:
                        best_feasible_solution = self.population[f].Clone()
                        new_sol = True

            # Prepare to identify the elite that shall remain in the next generation
            self.elitePopulation = []

            total_updated += no_of_updated
            if new_sol:
                self.LogNewSolution(best_feasible_solution, run_index, i+2, total_updated, timer() - start_time, log_file)
            self.LogPopulation(i+2, no_of_updated, total_updated, best_feasible_solution, timer() - exec_time, log_file)
            self.LogPopulation(i+2, no_of_updated, total_updated, best_feasible_solution, timer() - exec_time)

            if timer() - start_time >= timeout:
                break

        if best_feasible_solution != None:
            # Apply the best feasible solution
            self.ChromoToCpm(best_feasible_solution)
        else:
            # Apply the best non-feasible solution
            self.ChromoToCpm(self.population[0])
            fd=open("logs//" + log_file + ".log",'a')
            print("No feasible solution found!", file=fd)
            fd.close()
            print("No feasible solution found!")

        return self.cost_count, self.cost_time

    def LogNewSolution(self, sol, run_index, generation, total_updated, time, log_file):
        fd=open("logs//" + log_file + "_seq.log",'a')
        print(("%d %d %d %.2f %.2f %.2f %.2f %.2f"%(run_index, generation, total_updated, sol.total_cost, sol.makespan_cost, sol.makespan_median, sol.makespan_low_median, time)).replace(".",","), file=fd)
        fd.close()
        

    def LogPopulation(self, generation, no_of_updated, total_updated, best_feasible_solution = None, exec_time = "N/A", log_file = None):
        if log_file != None:
            try:
                fd=open("logs//" + log_file + ".log",'a')
            except:
                fd = None
        else:
            fd = None
        chromo_set = set()
        ancestor_set = set()
        feasible = 0
        for c in self.population:
            chromo_set.add(str(c.individual))
            ancestor_set.add(c.ancestor)
            feasible += c.feasible
        print("\nRun " + str(self.run_index) +
              ", gen " + str(generation) + 
              ", exectime = " + str(round(exec_time,2)) +
              ", unique = " + str(len(chromo_set)) + " (" + str(self.POPULATION_SIZE) + ")" + \
              ", feasible = " + str(feasible) + \
              ", ancestors = " + str(len(ancestor_set)) + \
              ", updated = " + str(no_of_updated) + \
              ", total updated = " + str(total_updated), file=fd)
        if best_feasible_solution != None:
            self.PrintChromo(best_feasible_solution, fd)
        for k in range(0,self.ELITE_SIZE*3//4):
            self.PrintChromo(self.population[k], fd)
        for k in range(self.POPULATION_SIZE-self.ELITE_SIZE//4, self.POPULATION_SIZE):
            self.PrintChromo(self.population[k], fd)
        if log_file != None:
            fd.close()

    def PrintChromo(self, chromo, fd = None):
        text = "{:.2f}".format(chromo.total_cost) + ", {:.2f}".format(chromo.makespan_cost) + ", {:.2f}".format(chromo.makespan_median) + ", {:.2f}, ".format(chromo.makespan_low_median) + str(chromo.feasible) + " (" + str(chromo.ancestor) + ") :: "
        for i in range(0, self.no_of_agents):
            idle_quota = str(round(chromo.idle_quota[self.agents[i].name],3))
            if self.agents[i].human:
                pref_quota = str(round(chromo.pref_quota[self.agents[i].name],3))
            else:
                pref_quota = "-"
            text += self.agents[i].name + "(" + idle_quota + "," + pref_quota + ")"
            task = chromo.individual[i][i]
            while task > 0:
                text += "-" + self.task_list[task - self.no_of_agents].name
                tf = chromo.indTaskData[task - self.no_of_agents].team_formation
                if tf.no_of_overseers > 0:
                    text += "o"
                elif tf.no_of_robots > 0 and tf.no_of_humans > 0:
                    text += "c"
                task = chromo.individual[i][task]
            text += " :: "
        print(text, file=fd)

    def CheckChromoConsistency(self, chromo):
        # Check team formation vs chromosome
        for task_id in range(0, len(self.task_list)):
            tc = chromo.indTaskData[task_id].team_formation
            no_of_humans = 0
            no_of_robots = 0
            for a_id in range(0, len(self.agents)):
                idx = np.argwhere(chromo.individual[a_id] == task_id + self.no_of_agents)
                if len(idx) == 1:
                    # Agent has task
                    if self.agents[a_id].human:
                        no_of_humans += 1
                    else:
                        no_of_robots += 1
            if not (no_of_humans == tc.no_of_humans + tc.no_of_overseers and no_of_robots == tc.no_of_robots):
                raise Exception("Task allocation does not match with task formation")
        # Check numTasks vs chromosome
        for a_id in range(0, len(self.agents)):
            if chromo.numTasks[a_id] != np.count_nonzero(np.add(chromo.individual[a_id], 1)):
                raise Exception("numTasks does match number of tasks in chromosome!")

    def CompareChromoWithCPM(self, chromo):
        for a_id in range(0, len(self.agents)):
            check_id = chromo.individual[a_id][a_id]
            list_index = 1
            while check_id > 0 and len(self.agents[a_id].tasks) > list_index:
                if check_id != self.agents[a_id].tasks[list_index].index + self.no_of_agents:
                    raise Exception("Chromosome does not match CPM for task " + str(list_index))
                list_index += 1
                check_id = chromo.individual[a_id][check_id]
        
    def TeamFormationMutation(self, chromo):
        # Select a random task
        rndT = self.rand_gen.randomUC.randrange(0, self.no_of_tasks)

        # Switch to a random team formation
        tc = chromo.indTaskData[rndT].team_formation
        tc_index = tc.index
        rndC = self.rand_gen.randomUC.randrange(0, len(self.task_list[rndT].process.team_formations) - 1)
        # Exclude the current team formation (if there are alternatives)
        if rndC <= tc_index:
            # Index is allowed to wrap around
            rndC -= 1
        tc_new = self.task_list[rndT].process.team_formations[rndC]
        chromo.indTaskData[rndT].team_formation = tc_new

        # Keep the agents if possible within the new team formation.
        # Identify agents to add/remove
        add_to_agents = []
        remove_from_agents = []
        assigned_robots = []
        for rob in self.robot_agents:
            idx = np.argwhere(chromo.individual[rob.index] == rndT + self.no_of_agents)
            if len(idx) == 1:
                # Robot has task
                assigned_robots.append((rob.index, idx[0][0]))
        if tc_new.no_of_robots > len(assigned_robots):
            # Add a random robot, assuming max 1 robot in team formation
            add_to_agents.append(self.robot_agents[self.rand_gen.randomUC.randrange(0,len(self.robot_agents))].index)
        elif tc_new.no_of_robots < len(assigned_robots):
            # Remove the task from the robot, assuming max 1 robot in team formation
            remove_from_agents.append(assigned_robots[0])
        assigned_humans = []
        for hum in self.human_agents:
            idx = np.argwhere(chromo.individual[hum.index] == rndT + self.no_of_agents)
            if len(idx) == 1:
                # Human has task
                assigned_humans.append((hum.index, idx[0][0]))
        if tc_new.no_of_humans + tc_new.no_of_overseers > len(assigned_humans):
            # Add a random human, assuming max 1 human in team formation
            add_to_agents.append(self.human_agents[self.rand_gen.randomUC.randrange(0,len(self.human_agents))].index)
        elif tc_new.no_of_humans + tc_new.no_of_overseers < len(assigned_humans):
            # Remove the task from the human, assuming max 1 human in team formation
            remove_from_agents.append(assigned_humans[0])

        # Remove the selected task from agents
        for data in remove_from_agents:
            self.RemoveTaskFromAgent(data[1], data[0], chromo)

        # Add the selected task to a random point for the agents
        for a_id in add_to_agents:
            self.AddTaskToAgentSequence2(rndT, a_id, chromo)

    def SwapTaskMutation(self, chromo):
        # Select two random agents
        rndAgents = self.rand_gen.randomUC.sample(range(0, self.no_of_agents), 2)

        # Find all tasks that can be swapped from agent 1 to agent 2 without affecting other agents (team formations are allowed to change).
        # The role of a human reciver of a swapped task is randomized (overseer or collaborative).
        a_id = rndAgents[0]
        a_id2 = rndAgents[1]
        swappable_tasks = []
        swappable_tasks2 = []
        prev_id = a_id
        t_id = chromo.individual[a_id][prev_id]
        while t_id > 0:
            # Get swappable tasks
            swap_info = self.GetTaskSwapInfo(prev_id, t_id, a_id, a_id2, chromo)
            if swap_info != None:
                swappable_tasks.append(swap_info)
            prev_id = t_id
            t_id = chromo.individual[a_id][prev_id]

        if len(swappable_tasks) > 0:
            prev_id2 = a_id2
            t_id2 = chromo.individual[a_id2][prev_id2]
            while t_id2 > 0:
                # Get swappable tasks
                swap_info = self.GetTaskSwapInfo(prev_id2, t_id2, a_id2, a_id, chromo)
                if swap_info != None:
                    swappable_tasks2.append(swap_info)
                prev_id2 = t_id2
                t_id2 = chromo.individual[a_id2][prev_id2]

            if len(swappable_tasks2):
                # Select a random task from the swappable tasks
                rndT_tuple = swappable_tasks[self.rand_gen.randomUC.randrange(0, len(swappable_tasks))]
                # Remove the task from the agent
                self.RemoveTaskFromAgent(rndT_tuple[0], a_id, chromo)
                # Update team formation
                chromo.indTaskData[rndT_tuple[1] - self.no_of_agents].team_formation = self.task_list[rndT_tuple[1] - self.no_of_agents].process.team_formations[rndT_tuple[2]]
                # Add the task to the 2nd agent
                self.AddTaskToAgentSequence2(rndT_tuple[1] - self.no_of_agents, a_id2, chromo)

                # Select a random task to swap with
                rndT2_tuple = swappable_tasks2[self.rand_gen.randomUC.randrange(0, len(swappable_tasks2))]
                # Remove the task from the agent
                # Note: Can not rely on prev_id. It may be invalid after the swapped task was added.
                self.RemoveTaskFromAgentSafer(rndT2_tuple[1], a_id2, chromo)
                # Update team formation
                chromo.indTaskData[rndT2_tuple[1] - self.no_of_agents].team_formation = self.task_list[rndT2_tuple[1] - self.no_of_agents].process.team_formations[rndT2_tuple[2]]
                # Add the task to the 1st agent
                self.AddTaskToAgentSequence2(rndT2_tuple[1] - self.no_of_agents, a_id, chromo)
                return True
        return False

    def InsertTaskMutation(self, chromo):
        # Select a random task
        rndT = self.rand_gen.randomUC.randrange(0, self.no_of_tasks)

        # Keep team formation
        team_formation = chromo.indTaskData[rndT].team_formation

        # Select random agents (according to team formation)
        team_agents = []
        # Select random robots
        rndRobots = self.rand_gen.randomUC.sample(range(0, len(self.robot_agents)), team_formation.no_of_robots)
        for r in rndRobots:
            team_agents.append(self.robot_agents[r].index)
        # Select random humans
        rndHumans = self.rand_gen.randomUC.sample(range(0, len(self.human_agents)), team_formation.no_of_humans + team_formation.no_of_overseers)
        for h in rndHumans:
            team_agents.append(self.human_agents[h].index)

        removed = set()
        added = set()
        for a_id in range(0, self.no_of_agents):
            idx = np.argwhere(chromo.individual[a_id]==rndT+self.no_of_agents)
            if len(idx) == 1:
                # Remove the task from the sequence
                self.RemoveTaskFromAgent(idx[0][0], a_id, chromo)
                removed.add((a_id, idx[0][0]))
                if len(idx[0]) > 1:
                    raise Exception("Multiple occurences of task in sequence")
            if a_id in team_agents:
                prev_id = self.AddTaskToAgentSequence2(rndT, a_id, chromo)
                if prev_id > 0:
                    added.add((a_id, prev_id))
        return added != removed

    def TaskToIdleMutation(self, chromo):
        # Select a random task
        rndT = self.rand_gen.randomUC.randrange(0, self.no_of_tasks)

        # Find assigned agents
        team_agents = []
        outsiders = []
        for a_id in range(0, len(self.agents)):
            idx = np.argwhere(chromo.individual[a_id] == rndT + self.no_of_agents)
            if len(idx) == 1:
                team_agents.append((idx[0][0], rndT + self.no_of_agents, a_id))
            else:
                outsiders.append(a_id)
        
        # Select a random team agent
        rndATuple = team_agents[self.rand_gen.randomUC.randrange(0, len(team_agents))]

        # Identify possible receivers of the task outside the team.
        receivers = []
        for a_id in outsiders:
            swap_info = self.GetTaskSwapInfo(rndATuple[0], rndATuple[1], rndATuple[2], a_id, chromo)
            if swap_info != None:
                # Surplus idle time is the actual idle time subtracted by the desired idle time
                # Consider total idle time:
                surplus_total_idle_time = chromo.idle_time_median[a_id] - self.agents[a_id].idle_min * chromo.makespan_median
                receivers.append(swap_info + (a_id, surplus_total_idle_time))

        if len(receivers) > 0:
            # Sort the receivers in order of idle time
            receivers.sort(key = lambda x:x[4])

            # Select the most idle receiver
            IdleTuple = receivers[-1]

            self.RemoveTaskFromAgent(rndATuple[0], rndATuple[2], chromo)
            # Update team formation
            chromo.indTaskData[rndATuple[1] - self.no_of_agents].team_formation = self.task_list[rndATuple[1] - self.no_of_agents].process.team_formations[IdleTuple[2]]
            # Add the task to the idle agent
            self.AddTaskToAgentSequence2(rndATuple[1] - self.no_of_agents, IdleTuple[3], chromo)
            return True
        return False

    def GetTaskSwapInfo(self, prev_id, t_id, a_id, a_id2, chromo):
        swap_info = None
        idx = np.argwhere(chromo.individual[a_id2] == t_id)
        if len(idx) == 1:
            # 2nd agent has task already
            return None
        else:
            # Check possible team formations
            t_tc = chromo.indTaskData[t_id - self.no_of_agents].team_formation
            all_tc = self.task_list[t_id - self.no_of_agents].process.team_formations
            if self.agents[a_id].human == self.agents[a_id2].human:
                alternatives = []
                if self.agents[a_id].human:
                    # Swapping from human to human
                    # The team formation may change (human <-> overseer)
                    for i in range(0, len(all_tc)):
                        if t_tc.no_of_humans + t_tc.no_of_overseers == all_tc[i].no_of_humans + all_tc[i].no_of_overseers and \
                            t_tc.no_of_robots == all_tc[i].no_of_robots:
                            alternatives.append((prev_id, t_id, i))
                else:
                    # Swapping from robot to robot
                    # Keep team formation
                    alternatives.append((prev_id, t_id, all_tc.index(t_tc)))

                if len(alternatives) > 1:
                    # Add a random team formation alternative
                    swap_info = alternatives[self.rand_gen.randomUC.randrange(0, len(alternatives))]
                elif len(alternatives) == 1:
                    swap_info = alternatives[0]
            elif self.agents[a_id].human:
                # Swapping from human to robot
                # Requires change of team formation
                alternatives = []
                for i in range(0, len(all_tc)):
                    if (t_tc.no_of_humans + t_tc.no_of_overseers - 1 == all_tc[i].no_of_humans + all_tc[i].no_of_overseers) and \
                        (t_tc.no_of_robots + 1 == all_tc[i].no_of_robots):
                        alternatives.append((prev_id, t_id, i))
                if len(alternatives) > 1:
                    # Add a random team formation alternative
                    swap_info = alternatives[self.rand_gen.randomUC.randrange(0, len(alternatives))]
                elif len(alternatives) == 1:
                    swap_info = alternatives[0]
            else:
                # Swapping from robot to human
                # Requires change of team formation
                alternatives = []
                for i in range(0, len(all_tc)):
                    if (t_tc.no_of_humans + t_tc.no_of_overseers + 1 == all_tc[i].no_of_humans + all_tc[i].no_of_overseers) and \
                        (t_tc.no_of_robots - 1 == all_tc[i].no_of_robots):
                        alternatives.append((prev_id, t_id, i))
                if len(alternatives) > 1:
                    # Add a random team formation alternative
                    swap_info = alternatives[self.rand_gen.randomUC.randrange(0, len(alternatives))]
                elif len(alternatives) == 1:
                    swap_info = alternatives[0]

        return swap_info

    def AdjustChromoFromDeadlockRepair(self, chromo, moved_tasks):
        for mt in moved_tasks:
            # mt = (task_id, a_id, seq_index)
            idx = np.argwhere(chromo.individual[mt[1]] == mt[0] + self.no_of_agents)
            if len(idx) == 1:
                # Remove the task from the sequence
                self.RemoveTaskFromAgent(idx[0][0], mt[1], chromo)
            else:
                raise Exception("Task missing in chromo")
            # Insert the task at the given insertion point
            self.AddTaskToAgentSequence(mt[0], mt[1], chromo, mt[2])

    def RemoveTaskFromAgent(self, prev_id, a_id, chromo):
        t_id = chromo.individual[a_id][prev_id]
        chromo.individual[a_id][prev_id] = chromo.individual[a_id][t_id]
        chromo.individual[a_id][t_id] = -1
        chromo.numTasks[a_id] -= 1

    def RemoveTaskFromAgentSafer(self, t_id, a_id, chromo):
        # Does not rely on a possibly invalid prev_id
        idx = np.argwhere(chromo.individual[a_id] == t_id)
        if len(idx) != 1:
            raise Exception("Task is missing in agent")
        else:
            self.RemoveTaskFromAgent(idx[0][0], a_id, chromo)

    def AddTaskToAgentSequence(self, task_id, a_id, chromo, AgentSeqIndex = None):
        # Add the task to the sequence
        if chromo.numTasks[a_id] == 0:
            # First task in agent's sequence
            if not (AgentSeqIndex == None or AgentSeqIndex == 0):
                raise Exception("Invalid agent sequence index")
            # No need to check precedence constraints
            chromo.individual[a_id][a_id] = task_id + self.no_of_agents
            chromo.individual[a_id][task_id + self.no_of_agents] = -1
            chromo.numTasks[a_id] = 1
        else:
            if AgentSeqIndex != None:
                if AgentSeqIndex < 0 or AgentSeqIndex > chromo.numTasks[a_id]:
                    raise Exception("Agent sequence index out of range")
                # Use given insertion point in the agent's sequence
                RndAgentSeqIndex = AgentSeqIndex
            else:
                # Select a random insertion point in the agent's sequence
                RndAgentSeqIndex = self.rand_gen.randomUC.randrange(0, chromo.numTasks[a_id] + 1)

            # Insert the add task as close as possible to the insertion point
            # Search for the best valid insertion point, starting from the very first task,
            # until precedence constraints prevent the added task to be inserted behind a task.
            cur_prev_index = -1
            cur_index = -1
            cur_i = -1
            # Check the first tasks, up to the insertion point
            PrevCheckIndex = a_id
            CheckIndex = chromo.individual[a_id][PrevCheckIndex]
            for i in range(0, chromo.numTasks[a_id]):
                # Check if it can be inserted before the check task
                if (self.task_list[task_id] in self.task_list[CheckIndex - self.no_of_agents].intra_schedule_preceding_tasks or \
                    self.task_list[task_id] not in self.task_list[CheckIndex - self.no_of_agents].intra_schedule_succeeding_tasks):
                    # Task can be inserted before the check task
                    if cur_index < 0 or abs(i - RndAgentSeqIndex) < abs(cur_i - RndAgentSeqIndex):
                        cur_prev_index = PrevCheckIndex
                        cur_index = CheckIndex
                        cur_i = i
                    if (self.task_list[task_id] in self.task_list[CheckIndex - self.no_of_agents].intra_schedule_succeeding_tasks or \
                        self.task_list[task_id] not in self.task_list[CheckIndex - self.no_of_agents].intra_schedule_preceding_tasks):
                        # Tack can also be inserted after the check task
                        # Check next task
                        PrevCheckIndex = CheckIndex
                        CheckIndex = chromo.individual[a_id][PrevCheckIndex]
                    else:
                        # Not possible to move beyond this task
                        # Keep the current index
                        break
                else:
                    # The task can not be inserted before the check task.
                    # Any previous insertion point is also invalid
                    cur_prev_index = -1
                    cur_index = -1
                    cur_i = -1
                    # It can be assumed it is possible to insert it after the check task.
                    # Check if it can be inserted before the next task.
                    PrevCheckIndex = CheckIndex
                    CheckIndex = chromo.individual[a_id][PrevCheckIndex]

            if CheckIndex < 0:
                # All tasks checked up to the last task.
                # Try to insert after the last task.
                if (self.task_list[task_id] in self.task_list[PrevCheckIndex - self.no_of_agents].intra_schedule_succeeding_tasks or \
                    self.task_list[task_id] not in self.task_list[PrevCheckIndex - self.no_of_agents].intra_schedule_preceding_tasks):
                    # The task can be inserted after the last task
                    if cur_index < 0 or abs(chromo.numTasks[a_id] - RndAgentSeqIndex) < abs(cur_i - RndAgentSeqIndex):
                        cur_prev_index = PrevCheckIndex
                        cur_index = CheckIndex
                        cur_i = chromo.numTasks[a_id]
                elif cur_index < 0:
                    raise Exception("No valid insertion point found for task!!!")

            if AgentSeqIndex != None and AgentSeqIndex != cur_i:
                raise Exception("Specified insertion index not selected")

            if chromo.individual[a_id][cur_prev_index] != task_id + self.no_of_agents:
                # Insert the task at the new location
                chromo.individual[a_id][cur_prev_index] = task_id + self.no_of_agents
                chromo.individual[a_id][task_id + self.no_of_agents] = cur_index
                chromo.numTasks[a_id] += 1
                return cur_prev_index
        return -1

    def AddTaskToAgentSequence2(self, task_id, a_id, chromo):
        # Add the task to the sequence
        if chromo.numTasks[a_id] == 0:
            # First task in agent's sequence
            # No need to check precedence constraints
            chromo.individual[a_id][a_id] = task_id + self.no_of_agents
            chromo.individual[a_id][task_id + self.no_of_agents] = -1
            chromo.numTasks[a_id] = 1
        else:
            # Find all valid insertion points,
            # starting from the very first task
            # until precedence constraints prevent the added task to be inserted behind a task.
            check_indexes = []
            prev_check_indexes = []

            # Search from the first task
            PrevCheckIndex = a_id
            CheckIndex = chromo.individual[a_id][PrevCheckIndex]
            for i in range(0, chromo.numTasks[a_id]):
                # Check if it can be inserted before the check task
                if (self.task_list[task_id] in self.task_list[CheckIndex - self.no_of_agents].intra_schedule_preceding_tasks or \
                    self.task_list[task_id] not in self.task_list[CheckIndex - self.no_of_agents].intra_schedule_succeeding_tasks):
                    # Task can be inserted before the check task
                    check_indexes.append(CheckIndex)
                    prev_check_indexes.append(PrevCheckIndex)

                    if (self.task_list[task_id] in self.task_list[CheckIndex - self.no_of_agents].intra_schedule_succeeding_tasks or \
                        self.task_list[task_id] not in self.task_list[CheckIndex - self.no_of_agents].intra_schedule_preceding_tasks):
                        # Tack can also be inserted after the check task
                        # Check next task
                        PrevCheckIndex = CheckIndex
                        CheckIndex = chromo.individual[a_id][PrevCheckIndex]
                    else:
                        # Not possible to move beyond this task
                        # Stop search
                        break
                else:
                    # The task can not be inserted before the check task.
                    # Any previous insertion point is also invalid
                    check_indexes.clear()
                    prev_check_indexes.clear()

                    # It can be assumed it is possible to insert it after the check task.
                    # Check if it can be inserted before the next task.
                    PrevCheckIndex = CheckIndex
                    CheckIndex = chromo.individual[a_id][PrevCheckIndex]

            if CheckIndex < 0:
                # All tasks checked up to the last task.
                # Try to insert after the last task.
                if (self.task_list[task_id] in self.task_list[PrevCheckIndex - self.no_of_agents].intra_schedule_succeeding_tasks or \
                    self.task_list[task_id] not in self.task_list[PrevCheckIndex - self.no_of_agents].intra_schedule_preceding_tasks):
                    # The task can be inserted after the last task
                    check_indexes.append(CheckIndex)
                    prev_check_indexes.append(PrevCheckIndex)

            # Select a random valid insertion point in the agent's sequence
            RndAgentSeqIndex = self.rand_gen.randomUC.randrange(0, len(check_indexes))
            cur_prev_index = prev_check_indexes[RndAgentSeqIndex]
            cur_index = check_indexes[RndAgentSeqIndex]

            if chromo.individual[a_id][cur_prev_index] != task_id + self.no_of_agents:
                # Insert the task at the new location
                chromo.individual[a_id][cur_prev_index] = task_id + self.no_of_agents
                chromo.individual[a_id][task_id + self.no_of_agents] = cur_index
                chromo.numTasks[a_id] += 1
                return cur_prev_index
        return -1

    def randomizeSolution(self, chromo):
        # An array, initially consisting of all tasks [0,1,...,no_of_tasks-1]
        # One by one, a random task is removed and assigned to a random team formation with random agents
        tasks_to_assign = np.array(range(0, self.no_of_tasks))
        # Last assigned task for each agent, initially no tasks are assigned
        lastAssigned = np.array([-1]*self.no_of_agents)

        for i in range(0, self.no_of_tasks):
            # Randomly pick a task
            rndT = self.rand_gen.randomUC.randrange(tasks_to_assign.size)

            # Randomly pick a valid team formation
            process = self.task_list[tasks_to_assign[rndT]].process
            rndTeamcomp = process.team_formations[self.rand_gen.randomUC.randrange(0, len(process.team_formations))]

            team_agents = []
            # Randomly pick robots
            rndRobots = self.rand_gen.randomUC.sample(range(0, len(self.robot_agents)), rndTeamcomp.no_of_robots)
            for r in rndRobots:
                team_agents.append(self.robot_agents[r].index)
            # Randomly pick humans
            rndHumans = self.rand_gen.randomUC.sample(range(0, len(self.human_agents)), rndTeamcomp.no_of_humans + rndTeamcomp.no_of_overseers)
            for h in rndHumans:
                team_agents.append(self.human_agents[h].index)

            # Assign the task to all agents in the team
            for a_id in team_agents:
                if chromo.numTasks[a_id] == 0:
                    # Assign the first task in the agent's sequence
                    chromo.individual[a_id][a_id] = tasks_to_assign[rndT] + self.no_of_agents
                else:
                    # Assign the next task in the agent's sequence
                    chromo.individual[a_id][lastAssigned[a_id]] = tasks_to_assign[rndT] + self.no_of_agents
                lastAssigned[a_id] = tasks_to_assign[rndT] + self.no_of_agents
                chromo.numTasks[a_id] += 1;

            # Keep a pointer to the randomly selected team formation
            chromo.indTaskData[tasks_to_assign[rndT]].team_formation = rndTeamcomp
            # Keep a pointer to the process that holds the task
            chromo.indTaskData[tasks_to_assign[rndT]].process = process
            
            # Assignment of the task is ready
            # Remove the task from the array
            tasks_to_assign = removeTaskFromArray(tasks_to_assign, tasks_to_assign[rndT])

    def evaluate(self, chromo, dependency_order = None, debug_mode = False):
        # Evaluate solution cost
        if debug_mode:
            old_mode = CPM.debug_mode
            CPM.debug_mode = True  # Extract all info from the best solution
        self.cost_time -= timer()
        makespan, makespan_low = self.last_task.ExploreMakespan(self.transition_costs, self.deterministic_costs, dependency_order, self.agents)
        self.cost_time += timer()
        self.cost_count += 1
        if debug_mode:
            CPM.debug_mode = old_mode # Restore previous debug mode

        chromo.feasible, chromo.total_cost, chromo.makespan_cost, chromo.makespan_median, chromo.makespan_low_median, chromo.idle_quota, chromo.pref_quota = CPM.SolutionCost(makespan, makespan_low, self.agents, self.percentile)
        for i in range(0, len(self.agents)):
            chromo.total_activity_time_median[i] = self.agents[i].total_activity_time.Median()
            chromo.idle_time_median[i] = chromo.makespan_low_median - chromo.total_activity_time_median[i]
            chromo.total_preferred_activity_time_median[i] = self.agents[i].total_preferred_activity_time.Median()

    def ChromoToCpm(self, chromo):
        # Convert chromosome to CPM representation.
        # When: After initialization of chromosome, after mutation of chromosomes.
        # Why: Apply a new solution to CPM.

        # Reset old solution, prepare for deadlock detection/repair and makespan exploration.
        CPM.PrepareForNewSolution(self.proc1, self.agents)

        # Convert task sequences and task teams
        addIndex = np.array([-1]*self.no_of_agents)
        for i in range(0, self.no_of_agents):
            addIndex[i] = chromo.individual[i][i]
            if addIndex[i] >= 0:
                self.agents[i].tasks.append(self.task_list[addIndex[i]-self.no_of_agents])
                self.task_list[addIndex[i]-self.no_of_agents].team.AllocateAgent(self.agents[i])
            while addIndex[i] >= 0:
                addIndex[i] = chromo.individual[i][addIndex[i]]
                if addIndex[i] >= 0:
                    self.agents[i].tasks.append(self.task_list[addIndex[i]-self.no_of_agents])
                    self.task_list[addIndex[i]-self.no_of_agents].team.AllocateAgent(self.agents[i])

        # Convert team formation
        for i in range(0, self.no_of_tasks):
            self.task_list[i].team_formation_index = chromo.indTaskData[i].team_formation.index

    def CpmToChromo(self, chromo):
        # Convert agent.tasks, task.team and team formation to chromosome.
        # When/why: 1) To update the chromosome after deadlock repair and makespan computation.
        #           2) To update the chromosome from an initial solution computed with GRASP.

        # Initialize temp variable containing the last added task for each agent
        addIndex = np.array([-1]*self.no_of_agents)

        for i in range(0, self.no_of_agents):
            # Reset chromosome
            chromo.numTasks[i] = 0
            chromo.individual[i][:] = -1

            if len(self.agents[i].tasks) > 1:
                chromo.individual[i][i] = self.agents[i].tasks[1].index + self.no_of_agents
                addIndex[i] = self.agents[i].tasks[1].index + self.no_of_agents
                chromo.numTasks[i] += 1

            for j in range(2, len(self.agents[i].tasks)):
                chromo.individual[i][addIndex[i]] = self.agents[i].tasks[j].index + self.no_of_agents
                addIndex[i] = self.agents[i].tasks[j].index + self.no_of_agents
                chromo.numTasks[i] += 1

        for task_id in range(0, len(self.task_list)):
            task = self.task_list[task_id]
            chromo.indTaskData[task_id].team_formation = task.process.team_formations[task.team_formation_index]
            chromo.indTaskData[task_id].process = task.process

def removeTaskFromArray(tasks_to_assign, rndT):
    idx = np.where(tasks_to_assign == rndT)[0][0]
    if np.size(tasks_to_assign) > 1 and idx >= 0:
        last = tasks_to_assign[-1]
        tasks_to_assign[idx] = last
        return tasks_to_assign[:-1]
    elif np.size(tasks_to_assign) <= 1 and idx >= 0:
        return []
