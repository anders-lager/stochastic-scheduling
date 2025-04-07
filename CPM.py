import SDUR
import numpy as np

debug_mode = False

class NavLocation(object):
    def __init__(self, location_index, x, y, z, q1, q2, q3, q4, name_prefix = ""):
        self.location_index = location_index
        self.name = name_prefix + str(location_index)
        self.x = x
        self.y = y
        self.z = z
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3
        self.q4 = q4

class Item(object):
    def __init__(self, part = None):
        self.parts = []
        if part != None:
            self.parts.append(part)

class Buffer(object):
    def __init__(self, capacity, isOrder = False):
        self.capacity = capacity
        self.items = []
        # Producer and consumer provide a link between processes
        self.producer = None
        self.consumer = None
        # The order buffer is typically filled at an initial state of the scheduling problem
        # while other buffers are empty.
        self.isOrder = isOrder
        self.Reset()

    def Reset(self):
        if self.isOrder:
            self.items.clear()
            # Fill up with initial items
            for i in range(0, self.capacity):
                self.items.insert(0, Item(i+1))
        else:
            # Empty the buffer
            self.items = []

    def Level(self):
        return len(self.items)
    
    def SetProducer(self, process):
        self.producer = process

    def SetConsumer(self, process):
        self.consumer = process

# An interface to a (potentially) shared buffer.
class BufferInterface(object):
    def __init__(self, buffer, condition, no_of_items):
        # Buffer shared between output and input for two processes:
        self.buffer = buffer
        # Buffer condition for buffer access:
        self.condition = condition.replace("capacity","self.buffer.capacity")
        # Number of items to be pushed/popped during buffer access:
        self.no_of_items = no_of_items
        # If agent must be at the process location to access the buffer.
        # This is True except for order buffers
        self.at_location = not buffer.isOrder

    def EvaluateCondition(self):
        return eval("self.buffer.Level()" + self.condition)
    
    def MultiPop(self):
        # Remove a group of items from the buffer
        items = []
        if self.no_of_items <= self.buffer.Level():
            for i in range(0,self.no_of_items):
                items.append(self.buffer.items.pop())
        return items

    def MultiPush(self, items):
        #  Insert a list of items into the buffer
        if len(items) == self.no_of_items:
            # Insert the items idividually
            for i in range(0, len(items)):
                self.buffer.items.insert(0, items[i])
        elif self.no_of_items == 1:
            # Merge all items into one item (e.g. after creating an assembly of a group of items)
            merged_item = Item()
            for i in range(0, len(items)):
                merged_item.parts.extend(items[i].parts)
            self.buffer.items.insert(0, merged_item)
        else:
            raise Exception("WARNING: Item push mismatch")

class Agent(object):
    def __init__(self, name, index, human = False, idle_min = 0.0, preference_min = 0.5):
        self.name = name
        # Index in the agents list
        self.index = index
        self.human = human
        self.idle_min = idle_min
        # Preferred team formations
        self.preference_min = preference_min
        self.preferred_tf = {}

        # Scheduled sequence
        self.tasks = []

        # Accumulated activity time after scheduled tasks
        self.task_accumulated_activity_time = {}

        # Accumulated activity time during the mission
        self.total_activity_time = SDUR.StochasticDuration()

        # Accumulated preferred activity time during the mission
        self.total_preferred_activity_time = SDUR.StochasticDuration()

        # A Task representing the start location of the agent
        initial_task = Task(None, None, 1)
        initial_task.team.AllocateAgent(self)
        initial_task.name = self.name
        initial_task.start_time = None
        initial_task.end_time = None
        initial_task.completion_time = SDUR.StochasticDuration()
        initial_task.team.ResetMovingDurations()
        initial_task.DO_verified = True
        self.tasks.append(initial_task)

    def SetHumanPreferredTeamFormations(self, first_process):
        proc = first_process
        while proc != None:
            self.preferred_tf[(self.name, proc.name)] = []
            for tf in proc.team_formations:
                if tf.no_of_humans > 0:
                    if proc.name == "Assemble":
                        self.preferred_tf[(self.name, proc.name)].append(tf)
                if tf.no_of_overseers > 0:
                    if proc.name == "Assemble" or \
                        proc.name == "Inspect" or \
                        proc.name == "Palletize" or \
                        proc.name == "Transport":
                        self.preferred_tf[(self.name, proc.name)].append(tf)
            proc = proc.NextProcess()

    def InPreferredTeamFormation(self, task):
        if not self.human:
            raise Exception("Preferred team formation checked for robot agent")
        return task.process.team_formations[task.team_formation_index] in self.preferred_tf[(self.name, task.process.name)]

    def IsPreferredTeamFormation(self, task, tf_index):
        if not self.human:
            raise Exception("Preferred team formation checked for robot agent")
        return task.process.team_formations[tf_index] in self.preferred_tf[(self.name, task.process.name)]

    # Check if a corresponding TeamTask object is in a list
    def InListOf(self, list):
        return any(filter(lambda x: x.agent == self, list))

# Agent data for a specific task
class TaskAgent(object):
    def __init__(self, agent):
        self.agent = agent
        # Time for starting moving to the location of the input buffer (if such a location exists)
        self.start_moving_time = None
        # Time for reaching the input buffer
        self.end_moving_time = None

    def ResetMovingDurations(self):
        self.start_moving_time = None
        self.end_moving_time = None

# Team data for a specific task
class TaskTeam(object):
    def __init__(self):
        self.agents = []
        self.agent_set = set()

    def ClearAgents(self):
        self.agents = []
        self.agent_set = set()

    def AllocateAgent(self, agent):
        self.agents.append(TaskAgent(agent))
        self.agent_set.add(agent)

    def Intersection(self, team):
        return list(self.agent_set.intersection(team.agent_set))
    
    def ResetMovingDurations(self):
        for agent_task in self.agents:
            agent_task.ResetMovingDurations()

class SubTask(object):
    def __init__(self, location = None):
        self.location = location
        self.duration = []

class ExplorePath(object):
    def __init__(self, dur = None):
        self.variable_sequence = []
        self.variable_duration = []
        self.variables = set()
        self.compound_variables = {}
        self.compound_var_sequences = {}
        self.compound_var_seq_durs = {}
        self.duration = dur
        self.min_estimate = None
        self.max_estimate = None
        self.similarity_score = -1

    def CopyFrom(self, other):
        self.variable_sequence = other.variable_sequence.copy()
        self.variables = other.variables.copy()
        self.compound_variables = other.compound_variables.copy()
        for name in other.compound_var_sequences:
            # Deeper copy, cannot just copy the list
            self.compound_var_sequences[name] = [[],[]]
            self.compound_var_sequences[name][0] = other.compound_var_sequences[name][0].copy()
            self.compound_var_sequences[name][1] = other.compound_var_sequences[name][1].copy()
        self.compound_var_seq_durs = other.compound_var_seq_durs.copy()
        self.variable_duration = other.variable_duration.copy()
        self.duration = other.duration.Clone()
        self.min_estimate = other.min_estimate
        self.max_estimate = other.max_estimate
        self.similarity_score = other.similarity_score

    def Clone(self):
        cl = ExplorePath()
        cl.variable_sequence = self.variable_sequence.copy()
        cl.variables.update(self.variables)
        cl.compound_variables = self.compound_variables.copy()
        for name in self.compound_var_sequences:
            # Deeper copy, cannot just copy the list
            cl.compound_var_sequences[name] = [[],[]]
            cl.compound_var_sequences[name][0] = self.compound_var_sequences[name][0].copy()
            cl.compound_var_sequences[name][1] = self.compound_var_sequences[name][1].copy()

        cl.compound_var_seq_durs = self.compound_var_seq_durs.copy()
        cl.variable_duration = self.variable_duration.copy()
        cl.duration = self.duration.Clone()
        cl.min_estimate = self.min_estimate
        cl.max_estimate = self.max_estimate
        # Similarity score is not inherited
        cl.similarity_score = -1
        return cl

    def ResetSimilarityScore(self):
        self.similarity_score = -1

    def SimilarityScore(self, other):
        no_of_equal = 0
        sim_score = 0
        shortest_len = min(len(self.variable_sequence), len(other.variable_sequence))
        while no_of_equal < shortest_len:
            if self.variable_sequence[no_of_equal] == other.variable_sequence[no_of_equal]:
                if self.variable_sequence[no_of_equal] in self.compound_variables:
                    sim_score += len(self.compound_variables[self.variable_sequence[no_of_equal]])
                    no_of_equal += 1
                else:
                    sim_score += 1
                    no_of_equal += 1
            else:
                break
        self.similarity_score = max(self.similarity_score, sim_score)
        return self.similarity_score

    def EqualVariables(self, var_name, other, other_name):
        if var_name in self.compound_var_sequences:
            if other_name in other.compound_var_sequences:
                if var_name == other_name:
                    return True
                elif ((self.compound_var_sequences[var_name][0] == other.compound_var_sequences[other_name][0] and \
                    self.compound_var_sequences[var_name][1] == other.compound_var_sequences[other_name][1]) or \
                    (self.compound_var_sequences[var_name][0] == other.compound_var_sequences[other_name][1] and \
                    self.compound_var_sequences[var_name][1] == other.compound_var_sequences[other_name][0])):
                    return True
                else:
                    return False
            else:
                return False
        else:
            return var_name == other_name

    def Merge(self, other, process):
        succeeded = False
        # Find the initial sequence of equal variables (possibly an empty sequence)
        no_of_equal = 0
        equals = set()
        shortest_len = min(len(self.variable_sequence), len(other.variable_sequence))
        while no_of_equal < shortest_len:
            if self.EqualVariables(self.variable_sequence[no_of_equal], other, other.variable_sequence[no_of_equal]):
                if self.variable_sequence[no_of_equal] in self.compound_variables:
                    equals.update(self.compound_variables[self.variable_sequence[no_of_equal]])
                else:
                    equals.add(self.variable_sequence[no_of_equal])
                no_of_equal += 1
            else:
                break
        if no_of_equal == shortest_len:
            # Redundant sequences
            # If they do not have the same length, keep the longest
            if len(self.variable_sequence) == shortest_len and len(other.variable_sequence) > shortest_len:
                self.CopyFrom(other)
            succeeded = True
        elif no_of_equal >= 0:
            # Check if the two sequences can be merged with a Max computation.
            check_set1 = self.variables.difference(equals)
            check_set2 = other.variables.difference(equals)
            if not check_set1.intersection(check_set2):
                # Merge the sequences
                self.variables.update(other.variables)
                merged_variables = set()
                merged_var_sequences = [[],[]]
                merged_var_seq_durs = [None, None]
                if no_of_equal == 0:
                    # The sequences only have different variables
                    merged_var_seq_durs[0] = self.duration.Clone()
                    merged_var_seq_durs[1] = other.duration.Clone()
                    self.duration.Max(other.duration)
                    merged_variables.update(self.variables)
                    merged_variables.update(other.variables)
                    merged_var_sequences[0].extend(self.variable_sequence.copy())
                    merged_var_sequences[1].extend(other.variable_sequence.copy())
                    # Remember the duration of the new compound variable
                    compound_dur = self.duration.Clone()
                    compound_dur.Lock()
                else:
                    # The sequences are identical in the beginning
                    # and only have independent variables in the end
                    self.duration.SetZero()
                    for i in range(0, no_of_equal):
                        self.duration.Conv(self.variable_duration[i])
                    # Operand 1
                    compound_dur = SDUR.StochasticDuration()
                    for i in range(no_of_equal, len(self.variable_sequence)):
                        compound_dur.Conv(self.variable_duration[i])
                        if self.variable_sequence[i] in self.compound_variables:
                            merged_variables.update(self.compound_variables[self.variable_sequence[i]])
                        else:
                            merged_variables.add(self.variable_sequence[i])
                        merged_var_sequences[0].append(self.variable_sequence[i])
                    merged_var_seq_durs[0] = compound_dur.Clone()
                    merged_var_seq_durs[0].Lock()
                    # Operand 2
                    op2 = SDUR.StochasticDuration()
                    for i in range(no_of_equal, len(other.variable_sequence)):
                        op2.Conv(other.variable_duration[i])
                        if other.variable_sequence[i] in other.compound_variables:
                            merged_variables.update(other.compound_variables[other.variable_sequence[i]])
                        else:
                            merged_variables.add(other.variable_sequence[i])
                        merged_var_sequences[1].append(other.variable_sequence[i])
                    merged_var_seq_durs[1] = op2
                    merged_var_seq_durs[1].Lock()
                    compound_dur.Max(op2)
                    compound_dur.Lock()
                    self.duration.Conv(compound_dur)
                # Remove merged variables/durations from the sequence
                self.variable_sequence = self.variable_sequence[0:no_of_equal]
                self.variable_duration = self.variable_duration[0:no_of_equal]
                # Create and add a compound variable/duration
                compound_variable_name = process.name + str(process.GetUniqueId())
                self.variable_sequence.append(compound_variable_name)
                self.variable_duration.append(compound_dur)
                self.compound_variables[compound_variable_name] = merged_variables
                self.compound_var_sequences[compound_variable_name] = merged_var_sequences
                self.compound_var_seq_durs[compound_variable_name] = merged_var_seq_durs
                succeeded = True
            else:
                # Check equality with operands of a compound variable
                check1 = [other, self]
                check2 = [self, other]
                for c_id in range(0,2):
                    # Require check1 next task to be a compound variable
                    if len(check1[c_id].variable_sequence) >= no_of_equal+1 and \
                        len(check2[c_id].variable_sequence) >= no_of_equal+1 and \
                        check1[c_id].variable_sequence[no_of_equal] in check1[c_id].compound_variables:
                        equal_id = -1
                        equal_operand_variables = None
                        compound_variable_name = check1[c_id].variable_sequence[no_of_equal]
                        # Check if any of the operands of the compound variable is identical with the other sequence
                        for i in range(0,2):
                            equal_operand_variables = check1[c_id].compound_var_sequences[compound_variable_name][i]
                            if no_of_equal + len(equal_operand_variables) < len(check2[c_id].variable_sequence) and \
                                no_of_equal + 1 == len(check1[c_id].variable_sequence):
                                if equal_operand_variables == check2[c_id].variable_sequence[no_of_equal:no_of_equal+len(equal_operand_variables)]:
                                    # Found an equal operand, where the check2 sequence has additional variables but the compound sequence has none.
                                    equal_id = i
                                    break
                            else:
                                if no_of_equal + len(equal_operand_variables) >= len(check2[c_id].variable_sequence):
                                    if equal_operand_variables[0:len(check2[c_id].variable_sequence)-no_of_equal] == check2[c_id].variable_sequence[no_of_equal:]:
                                        # Found an operand whose start is equal to the remaining sequence of check2
                                        # Just keep the compound sequence (which may have additional variables)
                                        if check1[c_id] == other:
                                            self.CopyFrom(other)
                                        succeeded = True
                                        break
                        if equal_id > 0:
                            # Need to recompute the duration of the sequence with the ending compound, considering the compared sequence
                            check1[c_id].duration.SetZero()
                            for i in range(0, no_of_equal):
                                check1[c_id].duration.Conv(check1[c_id].variable_duration[i])
                            # Merging the equal operand with the sequence
                            compound_dur = check1[c_id].compound_var_seq_durs[compound_variable_name][equal_id].Clone()
                            for i in range(no_of_equal + len(equal_operand_variables), len(check2[c_id].variable_sequence)):
                                compound_dur.Conv(check2[c_id].variable_duration[i])
                                if check2[c_id].variable_duration[i] in check2[c_id].compound_variables:
                                    check1[c_id].compound_variables[compound_variable_name].update(check1[c_id].compound_variables[check2[c_id].variable_sequence[i]])
                                else:
                                    check1[c_id].compound_variables[compound_variable_name].add(check2[c_id].variable_sequence[i])
                                check1[c_id].compound_var_sequences[compound_variable_name][equal_id].append(check2[c_id].variable_sequence[i])
                            # Update the operand duration
                            check1[c_id].compound_var_seq_durs[compound_variable_name][equal_id] = compound_dur.Clone()
                            check1[c_id].compound_var_seq_durs[compound_variable_name][equal_id].Lock()
                            # Get the other operand duration
                            op2 = check1[c_id].compound_var_seq_durs[compound_variable_name][not equal_id]
                            # Compute the compound variable duration
                            compound_dur.Max(op2)
                            check1[c_id].variable_duration[no_of_equal] = compound_dur
                            check1[c_id].variable_duration[no_of_equal].Lock()
                            # Update the total duration
                            check1[c_id].duration.Conv(compound_dur)
                            if check1[c_id] == other:
                                # Move the merged sequence to "self"
                                self.CopyFrom(other)

                            # Change the name of the modified compound variable to avoid mixing it up with the previous version
                            new_compound_name = process.name + str(process.GetUniqueId())
                            self.variable_sequence.remove(compound_variable_name)
                            self.variable_sequence.append(new_compound_name)
                            self.compound_variables[new_compound_name] = self.compound_variables.pop(compound_variable_name)
                            self.compound_var_sequences[new_compound_name] = self.compound_var_sequences.pop(compound_variable_name)
                            self.compound_var_seq_durs[new_compound_name] = self.compound_var_seq_durs.pop(compound_variable_name)
                            succeeded = True
                            break
        return succeeded

    def MinMaxEstimate(self):
        self.min_estimate, self.max_estimate = self.duration.MinMaxEstimate(0.01)
        return self.min_estimate, self.max_estimate

class Task(object):
    def __init__(self, process, processed_items, no_of_sub_tasks):
        self.process = process
        # The items handled by the task
        self.processed_items = processed_items
        self.name = self.Name()
        self.index = -1

        # Create sub tasks
        self.subtasks = []
        for i in range(0,no_of_sub_tasks):
            if process != None:
                # A process task
                self.subtasks.append(SubTask(process.location))
            else:
                # The task (and the subtask) represents the start location of a single agent
                self.subtasks.append(SubTask())

        # Task constraints identified from the process model

        # A list of tasks that cannot start until the completion (of this task)
        self.succeeding_tasks = []
        # A list of tasks that needs to complete before the start (of this task)
        self.preceding_tasks = []

        # A list of ALL tasks (not just the next ones) that succeed this task in the same schedule
        self.intra_schedule_succeeding_tasks = []
        # A list of ALL tasks (not just the next ones) that precede this task in the same schedule
        self.intra_schedule_preceding_tasks = []

        # A list of ALL tasks (not just the next ones) that cannot complete until the completion of this task
        self.all_succeeding_task_completions = []

        # A task that needs to start before the completion
        self.task_start_preceding_completion = None
        # A task that cannot complete until the start
        self.task_completion_succeeding_start = None

        # A task that needs to start before the start
        self.task_start_preceding_start = None
        # A task that cannot start before the start
        self.task_start_succeeding_start = None

        # A task that needs to complete before the completion
        self.task_completion_preceding_completion = None
        # A task that cannot complete until the completion
        self.task_completion_succeeding_completion = None

        # Indicate if the dependency order has been verified
        self.DO_verified = False

        # Allocated team to handle the task
        self.team = TaskTeam() # An empty team
        # Team formation (undefined)
        self.team_formation_index = -1

        self.start_time = None

        self.start_durations = []
        self.completion_durations = []
        self.merged_start_time = None
        self.merged_completion_time = None

    def CommonAgentWith(self, task):
        return len(self.team.agent_set & task.team.agent_set) > 0
    
    def TeamPreviousTaskCompleteAfter(self, task):
        for agent in self.team.agents:
            # Check previous tasks of the agent
            prev_task_index = agent.agent.tasks.index(self)-1
            while prev_task_index > 0:
                prev_task = agent.agent.tasks[prev_task_index]
                if prev_task in task.intra_schedule_preceding_tasks:
                    # No need to check earlier tasks for this agent
                    break
                elif prev_task in task.all_succeeding_task_completions:
                    return True
                # Check earlier scheduled tasks
                prev_task_index -= 1
        return False

    def VerifyDependencyOrder(self, task_lock):
        verified = True

        # Check precedence constraints
        for t in self.preceding_tasks:
            verified = verified and t.DO_verified
        if verified and self.task_start_preceding_start != None:
            verified = self.task_start_preceding_start.DO_verified
        if verified and self.task_completion_preceding_completion != None:
            verified = self.task_completion_preceding_completion.DO_verified

        if verified:
            if self.task_start_preceding_completion != None:
                verified = self.task_start_preceding_completion.DO_verified
                if not self.task_start_preceding_completion.DO_verified:
                    if self.task_start_preceding_completion.name not in task_lock.keys():
                        task_lock[self.task_start_preceding_completion.name] = []
                    if self not in task_lock[self.task_start_preceding_completion.name]:
                        task_lock[self.task_start_preceding_completion.name].append(self)

            # Check agent task sequences
            for agent in self.team.agents:
                prev_task = agent.agent.tasks[agent.agent.tasks.index(self)-1]
                verified = verified and prev_task.DO_verified
                if not prev_task.DO_verified:
                    # Agent's previous task is blocking the task
                    # Add a task lock to search this task again when the agents's previous task is released.
                    if prev_task.name not in task_lock.keys():
                        task_lock[prev_task.name] = []
                    if self not in task_lock[prev_task.name]:
                        task_lock[prev_task.name].append(self)

        return verified

    def SameProcessValidSuccessor(self, task_lock = None):
        succ = None
        for s in self.succeeding_tasks:
            if s.process == self.process and not s.DO_verified:
                if s.VerifyDependencyOrder(task_lock):
                    succ = s
                break
        if succ == None and self.task_start_succeeding_start != None:
            if self.task_start_succeeding_start.process == self.process and not self.task_start_succeeding_start.DO_verified:
                if self.task_start_succeeding_start.VerifyDependencyOrder(task_lock):
                    succ = self.task_start_succeeding_start
        if succ == None and self.task_completion_succeeding_completion != None:
            if self.task_completion_succeeding_completion.process == self.process and not self.task_completion_succeeding_completion.DO_verified:
                if self.task_completion_succeeding_completion.VerifyDependencyOrder(task_lock):
                    succ = self.task_completion_succeeding_completion
        return succ

    def NextProcessValidSuccessor(self, task_lock = None):
        succ = None
        for s in self.succeeding_tasks:
            if s.process == self.process.NextProcess() and not s.DO_verified:
                if s.VerifyDependencyOrder(task_lock):
                    succ = s
                break
        return succ
    
    def NextAgentTaskValidSuccessor(self, agent, task_lock = None):
        succ = None
        if len(agent.agent.tasks) > agent.agent.tasks.index(self)+1:
            next_task = agent.agent.tasks[agent.agent.tasks.index(self)+1]
            if not next_task.DO_verified:
                if next_task.VerifyDependencyOrder(task_lock):
                    succ = next_task
        return succ

    def ComputeAllSucceedingTaskCompletions(self, visited):
        all_successors = set()
        if not self.name in visited:
            # Find all successors and connect them to the current task.
            for task in self.succeeding_tasks:
                st = task.ComputeAllSucceedingTaskCompletions(visited)
                all_successors.update(st)

            if self.task_completion_succeeding_completion != None:
                st = self.task_completion_succeeding_completion.ComputeAllSucceedingTaskCompletions(visited)
                all_successors.update(st)

            # Other PC types are not considered:
            # task_completion_succeeding_start
            # self.task_start_succeeding_start

            # Record all successors
            self.all_succeeding_task_completions.extend(all_successors)

            # Include the current task.
            all_successors.add(self)

            visited[self.name] = True
        else:
            # Successors already computed
            all_successors.update(set(self.all_succeeding_task_completions))

            # Include the current task.
            all_successors.add(self)

        return all_successors

    def ComputeIntraScheduleConstraints(self, visited):
        all_successors = set()
        if not self.name in visited:
            # Find all successors and connect them to the current task.
            for task in self.succeeding_tasks:
                st = task.ComputeIntraScheduleConstraints(visited)
                all_successors.update(st)

            if self.task_completion_succeeding_start != None:
                st = self.task_completion_succeeding_start.ComputeIntraScheduleConstraints(visited)
                all_successors.update(st)

            if self.task_start_succeeding_start != None:
                st = self.task_start_succeeding_start.ComputeIntraScheduleConstraints(visited)
                all_successors.update(st)

            if self.task_completion_succeeding_completion != None:
                st = self.task_completion_succeeding_completion.ComputeIntraScheduleConstraints(visited)
                all_successors.update(st)

            # Record precedence constraints with all successors
            self.intra_schedule_succeeding_tasks.extend(all_successors)
            for task in all_successors:
                task.intra_schedule_preceding_tasks.append(self)

            # Include the current task.
            all_successors.add(self)

            visited[self.name] = True
        else:
            # Successors already computed
            all_successors.update(set(self.intra_schedule_succeeding_tasks))

            # Include the current task.
            all_successors.add(self)

        return all_successors

    def SearchDependencyOrder(self, dependency_order, agents, task_lock, do_limit = None):
        if self.DO_verified:
            raise Exception("WARNING: Task already checked")
        
        if do_limit != None and len(dependency_order) >= do_limit:
            # task limit reached (Grasp search)
            return
        else:
            dependency_order.append(self)
            self.DO_verified = True

        # 1) Continue with a valid successor in the NEXT process
        succ = self.NextProcessValidSuccessor(task_lock)
        if succ != None:
            succ.SearchDependencyOrder(dependency_order, agents, task_lock, do_limit)

        # 2) Continue with a valid successor in the SAME process
        succ = self.SameProcessValidSuccessor(task_lock)
        if succ != None:
            succ.SearchDependencyOrder(dependency_order, agents, task_lock, do_limit)

        # 3) Continue with tasks (previously) locked by this task
        if self.name in task_lock.keys():
            succ_list = task_lock[self.name].copy()
            for succ in succ_list:
                # Remove the task lock
                task_lock[self.name].remove(succ)
                if not succ.DO_verified:
                    # Verify that the successor not is locked by any other task
                    if not any(succ in lst for lst in task_lock.values()):
                        succ.SearchDependencyOrder(dependency_order, agents, task_lock, do_limit)

    def GetLocations(self, visited):
        locations = []
        names = []
        if not(self.name in visited):
            for sub in self.subtasks:
                locations.append(sub.location)
                names.append(self.name)

            for nxt in self.succeeding_tasks:
                locs, ns = nxt.GetLocations(visited)
                locations.extend(locs)
                names.extend(ns)
            if self.task_start_succeeding_start != None:
                locs, ns = self.task_start_succeeding_start.GetLocations(visited)
                locations.extend(locs)
                names.extend(ns)
            if self.task_completion_succeeding_completion != None:
                locs, ns = self.task_completion_succeeding_completion.GetLocations(visited)
                locations.extend(locs)
                names.extend(ns)

            visited[self.name] = True
        return locations, names

    def Name(self):
        name = None
        if self.process != None:
            # First letter is process name
            name = self.process.name[0]
            # Thereafter, numbers follow of processed items within this task
            for i in range(0,len(self.processed_items)):
                for part in self.processed_items[i].parts:
                    name += str(part)
                if i != len(self.processed_items)-1:
                    name += ","
        return name

    def ExploreDurations(self, cost_dict):

        # 0) If the task already has been explored, return the explored durations. Otherwise continue with 1
        if self.completion_durations:
            return self.start_durations, self.completion_durations
        
        completion_durations = []
        compute_start_durations = True
        if self.task_start_succeeding_start == None and self.task_completion_succeeding_start == None and not debug_mode:
            # No user of the task's start durations
            compute_start_durations = False

        # 1) Explore team agents' previous tasks
        prev_task_completions = {}
        prev_task_team_agents = {}
        agent_durations = {}
        prev_task_durations = []

        # Explore the previous tasks of all team agents
        for agent in self.team.agents:
            prev_task_idx = agent.agent.tasks.index(self)-1
            if prev_task_idx < 0:
                # No previous tasks, create and return a zero duration representing the start location
                self.completion_durations.append(ExplorePath())
                self.completion_durations[-1].variables.add(self.name)
                self.completion_durations[-1].variable_sequence.append(self.name)
                self.completion_durations[-1].duration = SDUR.StochasticDuration()
                self.completion_durations[-1].duration.Lock()
                self.completion_durations[-1].variable_duration.append(self.completion_durations[-1].duration)
                if debug_mode:
                    agent.agent.task_accumulated_activity_time[self.name] = SDUR.StochasticDuration()
                return None, self.completion_durations
            else:
                # Explore the agent's previous task (if not already explored)
                prev_task = agent.agent.tasks[prev_task_idx]
                if prev_task.name not in prev_task_completions: 
                    _, prev_task_completions[prev_task.name] = prev_task.ExploreDurations(cost_dict)
                    prev_task_team_agents[prev_task.name] = []
                # Add the agent to the list of team agents used for the agent's previous task
                prev_task_team_agents[prev_task.name].append(agent)

                # Get the agent's routing duration from the previous task
                if (not self.process.input_buffer.at_location) or self.process != prev_task.process or self.process.first_location != None:
                    # Add routing duration from the previous task to the waiting location for the input buffer of this task
                    agent_durations[agent.agent.name] = cost_dict[prev_task.subtasks[-1].location.name, self.subtasks[0].location.name]
                else:
                    # No routing, zero duration
                    agent_durations[agent.agent.name] = SDUR.StochasticDuration()

        # For all (unique) previous tasks, add agent routing durations
        for task_name, agents in prev_task_team_agents.items():
            routing_duration = agent_durations[agents[0].agent.name].Clone()
            for i in range(1, len(agents)):
                # Compute the combined routing duration for agents moving from the same previous task
                routing_duration.Max(agent_durations[agents[i].agent.name])
                # Update agent's total and preferred activity time
                agents[i].agent.total_activity_time.Conv(agent_durations[agents[i].agent.name])
                if agents[i].agent.human and agents[i].agent.InPreferredTeamFormation(self):
                    agents[i].agent.total_preferred_activity_time.Conv(agent_durations[agents[i].agent.name])

            # Update the first agent's total and preferred activity time
            agents[0].agent.total_activity_time.Conv(agent_durations[agents[0].agent.name])
            if agents[0].agent.human and agents[0].agent.InPreferredTeamFormation(self):
                agents[0].agent.total_preferred_activity_time.Conv(agent_durations[agents[0].agent.name])

            # Add the routing duration to all explored completions of the previous task
            for dur in prev_task_completions[task_name]:
                prev_task_durations.append(dur.Clone())
                prev_task_durations[-1].duration.Conv(routing_duration)
                variable_name = task_name + '-'
                # variable_name = ''
                for agent in prev_task_team_agents[task_name]:
                    variable_name += agent.agent.name
                variable_name += '-' + self.name
                prev_task_durations[-1].variables.add(variable_name)
                prev_task_durations[-1].variable_sequence.append(variable_name)
                # Record variable duration
                prev_task_durations[-1].variable_duration.append(routing_duration)

        # Collect explored arrival durations for the agents
        completion_durations.extend(prev_task_durations)

        # 2) Explore task completions that precede the start.
        # Consider preceding tasks completed by non-team agents,
        # but not if they precede a previous task by a team member.
        for ptc in self.preceding_tasks:
            if not self.CommonAgentWith(ptc) and not \
            self.TeamPreviousTaskCompleteAfter(ptc):
                _, cps_completion = ptc.ExploreDurations(cost_dict)
                for dur in cps_completion:
                    completion_durations.append(dur.Clone())

        # 3) Explore task starts that precede the start.
        # Consider a preceding task start by non-team agents,
        # but not if it precedes a previous task by a team member.
        if self.task_start_preceding_start != None and not \
            self.CommonAgentWith(self.task_start_preceding_start) and not \
            self.TeamPreviousTaskCompleteAfter(self.task_start_preceding_start):
                sps_start, _ = self.task_start_preceding_start.ExploreDurations(cost_dict)
                for dur in sps_start:
                    completion_durations.append(dur.Clone())

        # Filter and merge explored durations
        completion_durations = FilterAndMerge(completion_durations, self.process)
        no_of_start_durations = len(completion_durations)
        if compute_start_durations:
            self.start_durations = []
            for dur in completion_durations:
                self.start_durations.append(dur.Clone())
                # Lock explored durations from further updates
                self.start_durations[-1].duration.Lock()
        else:
            self.start_durations = None

        # The current task is included in all explored sequences (so far).
        # Update the variables for completion durations (not start durations).
        for dur in completion_durations:
            variable_name = ''
            for agent in self.team.agents:
                variable_name += agent.agent.name
            variable_name += self.name
            dur.variables.add(variable_name)
            dur.variable_sequence.append(variable_name)

        # 4) Add task durations
        # Task duration for the first subtask
        task_dur = self.subtasks[0].duration[self.team_formation_index].Clone()
        # Add routing and task durations for the remaining subtasks
        prev_team_location = self.subtasks[0].location
        for i in range(1, len(self.subtasks)):
            if prev_team_location.name != self.subtasks[i].location.name:
                task_dur.Conv(cost_dict[prev_team_location.name, self.subtasks[i].location.name])
            prev_team_location = self.subtasks[i].location
            task_dur.Conv(self.subtasks[i].duration[self.team_formation_index])
        # Add task duration
        for dur in completion_durations:
            dur.duration.Conv(task_dur)
            # Record variable duration
            dur.variable_duration.append(task_dur)

        # Update the total and preferred activity time of team agents
        for agent in self.team.agents:
            agent.agent.total_activity_time.Conv(task_dur)
            if agent.agent.human and agent.agent.InPreferredTeamFormation(self):
                agent.agent.total_preferred_activity_time.Conv(task_dur)
            if (debug_mode):
                # Note: debug mode is temporary activated for evaluation of the final GA solution
                agent.agent.task_accumulated_activity_time[self.name] = agent.agent.total_activity_time.Clone()

        # 5) Explore task completions that precede the completion
        # Consider a preceding task completion by non-team agents,
        # but not if it precedes a previous task by a team member.
        if self.task_completion_preceding_completion != None and not \
            self.CommonAgentWith(self.task_completion_preceding_completion) and not \
            self.TeamPreviousTaskCompleteAfter(self.task_completion_preceding_completion):
            _, cpc_completion = self.task_completion_preceding_completion.ExploreDurations(cost_dict)
            for dur in cpc_completion:
                completion_durations.append(dur.Clone())

        # 6) Explore task starts that precede the completion
        # Consider a preceding task start by non-team agents,
        # but not if it precedes a previous task by a team member.
        if self.task_start_preceding_completion != None and not \
            self.CommonAgentWith(self.task_start_preceding_completion) and not \
            self.TeamPreviousTaskCompleteAfter(self.task_start_preceding_completion):
            spc_start, _ = self.task_start_preceding_completion.ExploreDurations(cost_dict)
            for dur in spc_start:
                completion_durations.append(dur.Clone())

        # 7) Filter and merge explored durations
        if len(completion_durations) > no_of_start_durations:
            self.completion_durations = FilterAndMerge(completion_durations, self.process)
        else:
            # No update
            self.completion_durations = completion_durations

        # Lock explored durations from further updates
        for dur in self.completion_durations:
            dur.duration.Lock()

        if len(self.completion_durations) == 0:
            raise Exception("Missing explored duration from task " + self.name)

        return self.start_durations, self.completion_durations

    def ExploreMakespan(self, cost_dict, deterministic_mode = False, dependency_order = None, agents = None):
        if deterministic_mode:
            ResetSimulationData(agents, dependency_order)
            Simulate(dependency_order, cost_dict, deterministic_mode)
            makespan = self.completion_time_sim
            makespan_low = self.completion_time_sim
        else:
            # Explore the durations of all alternative sequences leading up to the task
            self.ExploreDurations(cost_dict)
            # Merge the remaining sequences
            makespan = SDUR.StochasticDuration()
            makespan.MultiMax([o.duration for o in self.completion_durations])
            makespan_low = SDUR.StochasticDuration()
            makespan_low.MultiMaxLow([o.duration for o in self.completion_durations])
        return makespan, makespan_low
    
class TeamFormation(object):
    def __init__(self, no_of_robots, no_of_humans, no_of_overseers, index):
        self.no_of_robots = no_of_robots
        self.no_of_humans = no_of_humans
        self.no_of_overseers = no_of_overseers
        # Index in the list of team formations for a process
        self.index = index

class Process(object):
    def __init__(self, name, concurrent_steps, no_of_subtasks, location, first_location = None):
        self.name = name
        self.unique_id = -1
        # Process steps starts and completes in a given order.
        # However, they can run concurrently if the flag is set.
        # E.g., two agents may run transportation tasks concurrently.
        self.concurrent_steps = concurrent_steps
        # Each subtask includes a routing and a subtask duration.
        # E.g., a transportation task may include a fetch subtask and a deliver subtask.
        self.no_of_subtasks = no_of_subtasks
        # The location of the processing
        self.location = location
        # A first location of processing,
        # e.g., the fetch location of a transportation task where "location" is the delivery location.
        self.first_location = first_location
        # Input interface to a buffer
        self.input_buffer = None
        # Output interface to a buffer
        self.output_buffer = None
        # Derived process tasks
        self.tasks = []
        # 
        self.team_formations = []

        # Temporary search data when deriving the precedence constraints:
        # 1) Number of items pushed to the input buffer, without popping them
        self.no_of_pushed_items = 0
        # 2) Last operated task, when generating output buffer related constraints
        self.last_operated_task = None

    def GetUniqueId(self):
        self.unique_id += 1
        return self.unique_id

    def AddInputBufferInterface(self, buffer, condition, no_of_items):
        buffer.SetConsumer(self)
        self.input_buffer = BufferInterface(buffer, condition, no_of_items)

    def AddOutputBufferInterface(self, buffer, condition, no_of_items):
        buffer.SetProducer(self)
        self.output_buffer = BufferInterface(buffer, condition, no_of_items)

    def NextProcess(self):
        next = None
        if self.output_buffer != None:
            next = self.output_buffer.buffer.consumer
        return next
    
    def PrevProcess(self):
        prev = None
        if self.input_buffer != None:
            prev = self.input_buffer.buffer.producer
        return prev

    def FirstProcess(self):
        first = None
        if self.input_buffer.buffer.producer != None:
            first = self.input_buffer.buffer.producer.FirstProcess()
        else:
            first = self
        return first
    
    # Assuming, there is only one last process
    def LastProcess(self):
        last = None
        if self.output_buffer != None:
            last = self.output_buffer.buffer.consumer.LastProcess()
        else:
            last = self
        return last
    
    def IsFirstProcess(self):
        return self.input_buffer.buffer.producer == None
    
    def ResetDependencyOrder(self):
        for task in self.tasks:
            task.DO_verified = False
        next = self.NextProcess()
        if next != None:
            next.ResetDependencyOrder()
    
    def PrepareTasksForNewSolution(self):
        self.unique_id = -1
        for task in self.tasks:
            task.DO_verified = False
            task.start_time = None
            task.team.ClearAgents()
            task.start_durations = None
            task.completion_durations = None
            task.merged_start_time = None
            task.merged_completion_time = None
            task.team_formation_index = -1
        next = self.NextProcess()
        if next != None:
            next.PrepareTasksForNewSolution()
    
    # Create new process tasks until the buffer interface conditions does not hold
    # Note: The very first process always pops a single item from the order buffer
    def CreateEnabledTasks(self):
        tasks_created = 0
        # Check input buffer conditions
        input_blocked = True
        while self.input_buffer.EvaluateCondition():
            task = None
            input_blocked = False
            items = []
            self.no_of_pushed_items += 1
            # Pop all pushed items from the input buffer
            # The first process will only pop a single item
            for i in range(0,self.no_of_pushed_items):
                items.extend(self.input_buffer.MultiPop())

            if self.output_buffer != None:
                # Push all accumulated items to the output buffer
                if self.output_buffer.EvaluateCondition():
                    self.output_buffer.MultiPush(items)
                    # Create task
                    task = Task(self, items, self.no_of_subtasks)
                    if self.tasks != []:
                        # Preceded by task in the previous cycle
                        if self.concurrent_steps:
                            if not self.input_buffer.buffer.isOrder:
                                # The task accessing an order buffer is not dependent on the previous task to be started first
                                # There is no physical in queue to be accessed. However, the task must end after it's predecessor.
                                self.tasks[-1].task_start_succeeding_start = task
                                task.task_start_preceding_start = self.tasks[-1]
                            self.tasks[-1].task_completion_succeeding_completion = task
                            task.task_completion_preceding_completion = self.tasks[-1]
                        else:
                            self.tasks[-1].succeeding_tasks.append(task)
                            task.preceding_tasks.append(self.tasks[-1])
                    self.tasks.append(task)
                    tasks_created += 1
                    self.no_of_pushed_items = 0
                else:
                    raise Exception("WARNING: Could not push popped items")
            else:
                # Last process, no output buffer
                # Create task
                task = Task(self, items, self.no_of_subtasks)
                if self.tasks != []:
                    # Task is preceded by task in the previous cycle
                    if self.concurrent_steps:
                        self.tasks[-1].task_start_succeeding_task = task
                        task.task_start_preceding_start = self.tasks[-1]
                        self.tasks[-1].task_completion_succeeding_completion = task
                        task.task_completion_preceding_completion = self.tasks[-1]
                    else:
                        self.tasks[-1].succeeding_tasks.append(task)
                        task.preceding_tasks.append(self.tasks[-1])
                self.tasks.append(task)
                tasks_created += 1
                self.no_of_pushed_items = 0

            if tasks_created == 1:
                if not self.IsFirstProcess():
                    # The first created task is preceded by the latest created task in the previous process
                    task.preceding_tasks.append(self.PrevProcess().tasks[-1])
                    self.PrevProcess().tasks[-1].succeeding_tasks.append(task)

            if self.IsFirstProcess():
                # Pop only one item (from the initially filled order buffer)
                break

        if input_blocked:
            # The input condition does not hold
            # Pop one extra item the next time when the input condition hold
            # E.g. condition may require the popping of two items.
            self.no_of_pushed_items += 1

        return tasks_created > 0
    
    def OperateNextProcessTask(self):
        # Pop the input buffer and push to the output buffer,
        # If the output buffer condition does not hold:
        # Operate the next process (recursively), task by task, until the output buffer condition holds again,
        # thus providing a task_start_preceding_completion constraint between the last tasks of the two processes.
        output_ok = False
        if self.last_operated_task == None:
            self.last_operated_task = self.tasks[0]
        elif self.last_operated_task == self.tasks[-1]:
            self.last_operated_task = None
            raise Exception("WARNING: operation of too many tasks")
        else:
            self.last_operated_task = self.tasks[self.tasks.index(self.last_operated_task)+1]

        if self.input_buffer.EvaluateCondition():
            # Pop needed items from the input buffer
            items = self.input_buffer.MultiPop()

            if self.output_buffer != None:
                # Push the items to the output buffer
                if self.output_buffer.EvaluateCondition():
                    # Push the items to the output buffer
                    self.output_buffer.MultiPush(items)
                    output_ok = True
                else:
                    # Operate the next process, task by task, until the output condition holds
                    while not self.output_buffer.EvaluateCondition():
                        self.NextProcess().OperateNextProcessTask()

                    # Push the items to the output buffer
                    self.output_buffer.MultiPush(items)
                    output_ok = True

                    # Record the constraint requiring the start of the latest operated task in the next process
                    self.last_operated_task.task_start_preceding_completion = self.NextProcess().last_operated_task
                    self.NextProcess().last_operated_task.task_completion_succeeding_start = self.last_operated_task
            else:
                output_ok = True

        return output_ok
    
    def AddTeamFormation(self, no_of_robots, no_of_humans, no_of_overseers):
        self.team_formations.append(TeamFormation(no_of_robots, no_of_humans, no_of_overseers, len(self.team_formations)))

def ResetProcessBuffers(first_process):
    cur = first_process
    while cur != None:
        cur.no_of_pushed_items = 0
        cur.last_operated_task = None
        if cur.input_buffer != None:
            cur.input_buffer.buffer.Reset()
        if cur.output_buffer != None:
            cur.output_buffer.buffer.Reset()
        cur = cur.NextProcess()

def GenerateOutputBufferConstraints(first_process):
    # Fill the order buffer and empty all other buffers
    ResetProcessBuffers(first_process)
    # Loop processes
    cur = first_process
    while cur != None:
        # Loop tasks
        while cur.last_operated_task != cur.tasks[-1]:
            # Operate the next task:
            # Pop the input buffer and push to the output buffer,
            # If the output buffer condition does not hold:
            # Operate the next process (recursively), task by task, until the output buffer condition holds again,
            # thus providing a task_start_preceding_completion constraint between the last tasks of the two processes.
            if not cur.OperateNextProcessTask():
                raise Exception("WARNING: Recursive operate failed")
        cur = cur.NextProcess()

def GenerateTasks(first_process):
    # Fill the order buffer and empty all other buffers
    ResetProcessBuffers(first_process)
    # Loop items in the order buffer
    while first_process.input_buffer.buffer.Level() != 0:
        if first_process.CreateEnabledTasks():
            next_process = first_process.NextProcess()
        else:
            raise Exception("WARNING: Not possible to create a task for the first process")
        # Loop processes
        while next_process != None:
            if next_process.CreateEnabledTasks():
                next_process = next_process.NextProcess()
            else:
                # Pop, process and push the next item from the order buffer (if any)
                next_process = None

def GenerateTaskIndexes(first_process):
    first_process.all_tasks = {}
    proc = first_process
    idx = 0
    while proc != None:
        for task in proc.tasks:
            task.index = idx
            idx += 1
            first_process.all_tasks[task.name] = task
        proc = proc.NextProcess()
    return idx  # Returns number of tasks

def Simulate(dependency_order, cost_dict, deterministic_mode = False):

    for task in dependency_order:
        agent_activity_time = []
        # Compute arrival time for team agents
        for agent in task.team.agents:
            # 1) Start moving time

            # Consider completion time of the last task
            prev_task = agent.agent.tasks[agent.agent.tasks.index(task)-1]
            agent.start_moving_time_sim = prev_task.completion_time_sim

            # 2) End moving time

            # Consider start moving time
            agent.end_moving_time_sim = agent.start_moving_time_sim

            # Consider routing duration from the previous task to the waiting location for the input buffer
            if (not task.process.input_buffer.at_location) or task.process != prev_task.process or task.process.first_location != None:
                routing_start_time = cost_dict[prev_task.subtasks[-1].location.name,task.subtasks[0].location.name].start_time
                routing_end_time = routing_start_time  + cost_dict[prev_task.subtasks[-1].location.name,task.subtasks[0].location.name].duration
                if deterministic_mode:
                    routing_time_sample = (routing_start_time + routing_end_time) / 2
                else:
                    routing_time_sample = np.random.uniform(routing_start_time, routing_end_time)
                agent.end_moving_time_sim += routing_time_sample

                agent_activity_time.append(routing_time_sample)
            else:
                agent_activity_time.append(0.0)

        # 3) Start time

        # Consider preceding tasks completed by non-team agents,
        # but not if they precede a previous task by a team member.
        for pt in task.preceding_tasks:
            if not task.CommonAgentWith(pt) and not \
                task.TeamPreviousTaskCompleteAfter(pt):
                if task.start_time_sim != None:
                    task.start_time_sim = max(task.start_time_sim, pt.completion_time_sim)
                else:
                    task.start_time_sim = pt.completion_time_sim

        # Consider a preceding task start by non-team agents,
        # but not if it precedes a previous task by a team member.
        if task.task_start_preceding_start != None and not \
            task.CommonAgentWith(task.task_start_preceding_start) and not \
            task.TeamPreviousTaskCompleteAfter(task.task_start_preceding_start):
            if task.start_time_sim != None:
                task.start_time_sim = max(task.start_time_sim, task.task_start_preceding_start.start_time_sim)
            else:
                task.start_time_sim = task.task_start_preceding_start.start_time_sim

        # Add time to decrement the input buffer
        pass

        # Compute idle time while waiting for the task to start.
        # Note: 
        #       Idle time is always zero for single agent tasks, if all preceding tasks are allocated to the agent.
        #       Multi agent tasks may always cause idle time, caused by waiting on the arrival of other agents.
        # 
        if task.start_time_sim != None or len(task.team.agents) > 1:
            for agent in task.team.agents:
                # Compute earliest start time for this agent

                # Consider start time (excluding other agents arrivals)
                if task.start_time_sim != None:
                    earliest_start_time = task.start_time_sim
                else:
                    earliest_start_time = None

                # Consider other agents's arrival times
                for h in task.team.agents:
                    if h != agent:
                        if earliest_start_time != None:
                            earliest_start_time = max(earliest_start_time, h.end_moving_time_sim)
                        else:
                            earliest_start_time = h.end_moving_time_sim

                # Compute idle time by subtracting the agent's arrival time from earliest start time.
                # Idle time can not be negative.
                idle_time = max(0, earliest_start_time - agent.end_moving_time_sim)

                # Accumulate idle time
                agent.agent.total_idle_time_sim += idle_time

        for agent in task.team.agents:
            # Save accumulated idle time for every task for presentation purpose
            agent.agent.task_accumulated_idle_time_sim[task.name] = agent.agent.total_idle_time_sim

        # Consider arrival time of all agents
        for agent in task.team.agents:
            if task.start_time_sim != None:
                # Consider agent's moving time
                task.start_time_sim = max(task.start_time_sim, agent.end_moving_time_sim)
            else:
                task.start_time_sim = agent.end_moving_time_sim

        # 4) End time

        # Consider start time
        task.end_time_sim = task.start_time_sim

        # Add task duration for the first subtask
        task_start_time = task.subtasks[0].duration[task.team_formation_index].start_time
        task_end_time = task_start_time  + task.subtasks[0].duration[task.team_formation_index].duration
        if deterministic_mode:
            task_time_sample = (task_start_time + task_end_time) / 2
        else:
            task_time_sample = np.random.uniform(task_start_time, task_end_time)

        task.end_time_sim += task_time_sample
        activity_time = task_time_sample

        # Add routing and task durations for the remaining subtasks
        prev_team_location = task.subtasks[0].location
        for i in range(1, len(task.subtasks)):
            if prev_team_location.name != task.subtasks[i].location.name:

                routing_start_time = cost_dict[prev_team_location.name, task.subtasks[i].location.name].start_time
                routing_end_time = routing_start_time  + cost_dict[prev_team_location.name, task.subtasks[i].location.name].duration
                if deterministic_mode:
                    routing_time_sample = (routing_start_time + routing_end_time) / 2
                else:
                    routing_time_sample = np.random.uniform(routing_start_time, routing_end_time)
                task.end_time_sim += routing_time_sample
                activity_time += routing_time_sample
            prev_team_location = task.subtasks[i].location

            task_start_time = task.subtasks[i].duration[task.team_formation_index].start_time
            task_end_time = task_start_time  + task.subtasks[i].duration[task.team_formation_index].duration
            if deterministic_mode:
                task_time_sample = (task_start_time + task_end_time) / 2
            else:
                task_time_sample = np.random.uniform(task_start_time, task_end_time)

            task.end_time_sim += task_time_sample
            activity_time += task_time_sample

        for a_id in range(0, len(task.team.agents)):
            agent_activity_time[a_id] += activity_time

        # 5) Completion time

        # Consider end time
        task.completion_time_sim = task.end_time_sim
        earliest_completion_time = None

        # Consider a preceding task completion by non-team agents,
        # but not if it precedes a previous task by a team member.
        if task.task_completion_preceding_completion != None and not \
            task.CommonAgentWith(task.task_completion_preceding_completion) and not \
            task.TeamPreviousTaskCompleteAfter(task.task_completion_preceding_completion):
            earliest_completion_time = task.task_completion_preceding_completion.completion_time_sim

        # Consider a preceding task start by non-team agents,
        # but not if it precedes a previous task by a team member.
        if task.task_start_preceding_completion != None and not \
            task.CommonAgentWith(task.task_start_preceding_completion) and not \
            task.TeamPreviousTaskCompleteAfter(task.task_start_preceding_completion):
            if earliest_completion_time != None:
                earliest_completion_time = max(earliest_completion_time, task.task_start_preceding_completion.start_time_sim)
            else:
                earliest_completion_time = task.task_start_preceding_completion.start_time_sim

        if earliest_completion_time != None:
            task.completion_time_sim = max(task.completion_time_sim, earliest_completion_time)

            # Compute idle time for all team agents while waiting for the task to complete,
            # i.e., waiting for a preceding task to start or complete, as required for task completion.
            # Idle time can not be negative.
            idle_time = max(0, earliest_completion_time - task.end_time_sim)

            for agent in task.team.agents:
                agent.agent.total_idle_time_sim += idle_time
                if task.name in agent.agent.task_accumulated_idle_time_sim:
                    # Adding to previous idle time for task
                    agent.agent.task_accumulated_idle_time_sim[task.name] += idle_time
                else:
                    agent.agent.task_accumulated_idle_time_sim[task.name] = agent.agent.total_idle_time_sim

        # Compute activity time (total and preferred)
        for i in range (0, len(task.team.agents)):
            # Total
            task.team.agents[i].agent.total_activity_time_sim += agent_activity_time[i]
            task.team.agents[i].agent.task_accumulated_activity_time_sim[task.name] = task.team.agents[i].agent.total_activity_time_sim
            # Preferred
            if task.team.agents[i].agent.human and task.team.agents[i].agent.InPreferredTeamFormation(task):
                task.team.agents[i].agent.total_preferred_activity_time_sim += agent_activity_time[i]

def ResetSimulationData(agents, dependency_order):
    for agent in agents:
        agent.total_idle_time_sim = 0
        agent.tasks[0].start_time_sim = 0
        agent.tasks[0].completion_time_sim = 0
        agent.task_accumulated_idle_time_sim = {}
        agent.task_accumulated_idle_time_sim[agent.tasks[0].name] = 0
        agent.total_activity_time_sim = 0
        agent.task_accumulated_activity_time_sim = {}
        agent.task_accumulated_activity_time_sim[agent.tasks[0].name] = 0
        agent.total_preferred_activity_time_sim = 0
    for task in dependency_order:
        task.start_time_sim = None
        task.completion_time_sim = 0

def MonteCarloSimulate(first_process, dependency_order, agents, cost_dict, no_of_runs):
    # Initialize counter data
    arrsize = int(SDUR.max_dur/SDUR.delta) + 1
    interval_counter_makespan = [0] * arrsize
    for agent in agents:
        agent.interval_counter_total_idle_time = [0] * arrsize
        agent.interval_counter_total_activity_time = [0] * arrsize
        agent.interval_counter_total_preferred_activity_time = [0] * arrsize
        agent.interval_counter_task_start = []
        agent.interval_counter_task_completion = []
        agent.interval_counter_task_activity = []
        agent.interval_counter_task_idle = []
        for task in agent.tasks:
            agent.interval_counter_task_start.append([0] * arrsize)
            agent.interval_counter_task_completion.append([0] * arrsize)
            agent.interval_counter_task_activity.append([0] * arrsize)
            agent.interval_counter_task_idle.append([0] * arrsize)

    last_task = first_process.LastProcess().tasks[-1] 

    # Run simulations
    for i in range(0,no_of_runs):
        ResetSimulationData(agents, dependency_order)

        # Simulate the tasks in the given dependency order
        Simulate(dependency_order, cost_dict)

        # Get makespan, i.e., the completion time of the last task (preceded by all other tasks)
        makespan_sim = last_task.completion_time_sim

        # Update makespan interval counter
        assert(makespan_sim >= 0 and makespan_sim < SDUR.max_dur)
        idx = round((arrsize-1)*makespan_sim/SDUR.max_dur)
        interval_counter_makespan[idx] += 1
        #print("Simulated duration: "+str(makespan_sim)+" "+str(self.big_grid[idx]))

        for agent in agents:
            # Compute idle time from the last agent task to the very last task
            idle_time = makespan_sim - agent.tasks[-1].completion_time_sim

            # Update total idle time
            agent.total_idle_time_sim += idle_time
            # Update total idle time interval counter
            assert(agent.total_idle_time_sim >= 0 and agent.total_idle_time_sim < SDUR.max_dur)
            idx = round((arrsize-1)*agent.total_idle_time_sim/SDUR.max_dur)
            agent.interval_counter_total_idle_time[idx] += 1

            # Update total activity time interval counter
            assert(agent.total_activity_time_sim >= 0 and agent.total_activity_time_sim < SDUR.max_dur)
            idx = round((arrsize-1)*agent.total_activity_time_sim/SDUR.max_dur)
            agent.interval_counter_total_activity_time[idx] += 1

            # Update preferred activity time interval counter
            assert(agent.total_preferred_activity_time_sim >= 0 and agent.total_preferred_activity_time_sim < SDUR.max_dur)
            idx = round((arrsize-1)*agent.total_preferred_activity_time_sim/SDUR.max_dur)
            agent.interval_counter_total_preferred_activity_time[idx] += 1

            for j in range(0, len(agent.tasks)):
                assert(agent.tasks[j].completion_time_sim >= 0 and agent.tasks[j].completion_time_sim < SDUR.max_dur)
                idx = round((arrsize-1)*agent.tasks[j].completion_time_sim/SDUR.max_dur)
                agent.interval_counter_task_completion[j][idx] += 1

                assert(agent.tasks[j].start_time_sim >= 0 and agent.tasks[j].start_time_sim < SDUR.max_dur)
                idx = round((arrsize-1)*agent.tasks[j].start_time_sim/SDUR.max_dur)
                agent.interval_counter_task_start[j][idx] += 1

                assert(agent.task_accumulated_activity_time_sim[agent.tasks[j].name] >= 0 and agent.task_accumulated_activity_time_sim[agent.tasks[j].name] < SDUR.max_dur)
                idx = round((arrsize-1)*agent.task_accumulated_activity_time_sim[agent.tasks[j].name]/SDUR.max_dur)
                agent.interval_counter_task_activity[j][idx] += 1

                assert(agent.task_accumulated_idle_time_sim[agent.tasks[j].name] >= 0 and agent.task_accumulated_idle_time_sim[agent.tasks[j].name] < SDUR.max_dur)
                idx = round((arrsize-1)*agent.task_accumulated_idle_time_sim[agent.tasks[j].name]/SDUR.max_dur)
                agent.interval_counter_task_idle[j][idx] += 1

    # Normalize the Monte Carlo distributions
    for agent in agents:
        agent.normalized_ic_total_idle_time = np.divide(agent.interval_counter_total_idle_time, no_of_runs)
        agent.normalized_ic_total_activity_time = np.divide(agent.interval_counter_total_activity_time, no_of_runs)
        agent.normalized_ic_total_preferred_activity_time = np.divide(agent.interval_counter_total_preferred_activity_time, no_of_runs)
        agent.normalized_ic_start_time = []
        agent.normalized_ic_completion_time = []
        agent.normalized_ic_activity_time = []
        agent.normalized_ic_idle_time = []
        for j in range(0, len(agent.tasks)):
            agent.normalized_ic_start_time.append(np.divide(agent.interval_counter_task_start[j], no_of_runs))
            agent.normalized_ic_completion_time.append(np.divide(agent.interval_counter_task_completion[j], no_of_runs))
            agent.normalized_ic_activity_time.append(np.divide(agent.interval_counter_task_activity[j], no_of_runs))
            agent.normalized_ic_idle_time.append(np.divide(agent.interval_counter_task_idle[j], no_of_runs))

    first_process.normalized_ic_makespan = np.divide(interval_counter_makespan, no_of_runs)

def ComputeAllSucceedingTaskCompletions(first_process):
    visited = {}
    first_process.tasks[0].ComputeAllSucceedingTaskCompletions(visited)

def ComputeIntraScheduleConstraints(first_process):
    visited = {}
    first_process.tasks[0].ComputeIntraScheduleConstraints(visited)

def RepairAgentTaskSequences(agents):
    # Keep the original sequence order as long as possible
    order_updated = False
    for agent in agents:
        new_seq = []
        # Start with the 2nd task.
        # The first task has no constraints and represents the agent start location
        for idx in range(1,len(agent.tasks)):
            task = agent.tasks[idx]
            if new_seq == []:
                # Insert the task into the empty list.
                new_seq.append(task)
            else:
                # Compare the task with the last task
                cmp_index = len(new_seq)-1
                # Insert after the last task if possible.
                current_insert_index = cmp_index + 1 
                while cmp_index >= 0:
                    cmp_task = new_seq[cmp_index]
                    if task in cmp_task.intra_schedule_preceding_tasks:
                        # The task precedes the compared task.
                        # Insert before the compared task.
                        current_insert_index = cmp_index
                        order_updated = True
                    # Compare with the previous task (if existing)
                    cmp_index -= 1
                # All tasks checked.
                # Insert the task at the current insertion point.
                new_seq.insert(current_insert_index, task)

        # Insert the initial task representing the start location of the agent
        new_seq.insert(0, agent.tasks[0])
        # Apply the new task sequence
        agent.tasks = new_seq
        
    return order_updated

class SwapAlt(object):
    def __init__(self, locking_task_obj, move_task, move_in_agent):
        self.locking_task_obj = locking_task_obj
        self.move_task = move_task
        self.move_in_agent = move_in_agent

def RepairInterScheduleDeadLocks(first_process, agents):
    # Check for deadlocks recursively and compute the dependency order
    deadlock = True
    moved_tasks = []
    while deadlock:
        dependency_order = []
        task_lock = {}
        first_process.tasks[0].SearchDependencyOrder(dependency_order, agents, task_lock)
        last_task = first_process.LastProcess().tasks[-1]
        if last_task.DO_verified:
            # A valid dependency order found
            deadlock = False
            if not all([tl == [] for tl in task_lock.values()]):
                raise Exception("No deadlock but remaining agent locks !!")
        else:
            if debug_mode:
                for tl in task_lock:
                    if task_lock[tl] != []:
                        msg = "Locking task " + tl
                        for t in task_lock[tl]:
                            msg += ", Locked task "+ t.name
                            for a in t.team.agents:
                                msg += ", Locked agent " + a.agent.name
                        print(msg)

            if all([tl == [] for tl in task_lock.values()]):
                raise Exception("Deadlock without agent locks !!")

            # Try to move a locked task before the locking task when these are assigned to the same agent
            swap_alts = []
            locking_task_obj = None
            for locking_task in task_lock:
                locking_task_obj = first_process.all_tasks[locking_task]
                for locked_task in task_lock[locking_task]:
                    # Find agents allocated to both the locked task and to the locking task
                    common_agents = locked_task.team.Intersection(first_process.all_tasks[locking_task].team)
                    for agent in common_agents:
                        locking_task_scheduled_before_locked_task = agent.tasks.index(locking_task_obj) <= agent.tasks.index(locked_task)
                        # Verify that the locked task not already is scheduled before the locking task
                        # Note: Multiple agents may be allocated to the same locking/locked task pair
                        #       but only one of them caused the lock.
                        if locking_task_scheduled_before_locked_task:
                            # Verify that it is possible to schedule the locked task before the locking task
                            if locked_task not in first_process.all_tasks[locking_task].intra_schedule_succeeding_tasks:
                                # The locked task may be moved before the locking task in the schedule
                                swap_alts.append(SwapAlt(locking_task_obj, locked_task, agent))
            # Select a locked task that does not suceed any other locked task
            selected_alt = None
            for alt1 in swap_alts:
                alt1_ok = True
                for alt2 in swap_alts:
                    if alt1 != alt2 and alt1.move_task in alt2.move_task.intra_schedule_succeeding_tasks:
                        alt1_ok = False
                        break
                if alt1_ok:
                    selected_alt = alt1
                    break

            swap_list = []
            swap_list.append(selected_alt)
            agent_list = []
            agent_list.append(selected_alt.move_in_agent)

            # Avoid creating a new cycle.
            for alt in swap_alts:
                # Check occurences of the locked task with other agents
                if alt != selected_alt and alt.move_task == selected_alt.move_task and alt.move_in_agent not in agent_list:
                    swap_list_temp = swap_list.copy()
                    for swap in swap_list_temp:
                        # 1) Check if precedence relation of the locking tasks will create a cycle if not swapped.
                        # 2) Check if the same locking task. Agents need to visit common locking/locked tasks in the same order.
                        if swap.locking_task_obj in alt.locking_task_obj.intra_schedule_preceding_tasks or \
                            swap.locking_task_obj == alt.locking_task_obj:
                            # Creates a cycle if not swapped
                            # Add to swap list
                            swap_list.append(alt)
                            agent_list.append(alt.move_in_agent)

            for swap in swap_list:
                # Swap the tasks in the agent schedule
                locking_task_obj = swap.locking_task_obj
                move_task = swap.move_task
                move_in_agent = swap.move_in_agent

                old_index = move_in_agent.tasks.index(move_task)
                new_index = move_in_agent.tasks.index(locking_task_obj)

                cur_index = -1
                for i in range(old_index - 1, -1, -1):
                    if move_task in move_in_agent.tasks[i].intra_schedule_succeeding_tasks or move_in_agent.tasks[i].DO_verified:
                        # Can not move the task further forward or no need to do it
                        break
                    elif i <= new_index:
                        cur_index = i

                if cur_index > 0:
                    if debug_mode:
                        print("Moving " + move_task.name + " before " + move_in_agent.tasks[cur_index].name  + " and " + locking_task_obj.name + " (locking task), for agents " + move_in_agent[a].name)
                    move_in_agent.tasks.remove(move_task)
                    move_in_agent.tasks.insert(cur_index, move_task)
                    moved_tasks.append((move_task.index, move_in_agent.index, cur_index - 1))
                else:
                    raise Exception("Deadlock repair failed")

            # Prepare for next deadlock check
            first_process.ResetDependencyOrder()

    return dependency_order, moved_tasks

def PrepareForNewSolution(first_process, agents):
    # Prepare tasks
    first_process.PrepareTasksForNewSolution()

    # Prepare agents
    for agent in agents:
        agent.task_accumulated_activity_time = {}
        agent.total_activity_time.SetZero()
        agent.total_preferred_activity_time.SetZero()
        # Remove all tasks except the first, i.e., the agent.
        agent.tasks = agent.tasks[0:1]

def SolutionCost(makespan, makespan_low, agents, percentile):
    if isinstance(makespan, SDUR.StochasticDuration):
        deterministic_costs = False
        makespan_cost = makespan.AtPercentile(percentile)
        makespan_median = makespan.Median()
        makespan_low_median = makespan_low.Median()
    else:
        deterministic_costs = True
        makespan_cost = makespan
        makespan_median = makespan
        makespan_low_median = makespan_low

    total_idle_penalty = 0
    total_activity_penalty = 0
    idle_quota = {}
    pref_quota = {}
    for agent in agents:
        if deterministic_costs:
            total_activity_time_median = agent.total_activity_time_sim
        else:
            total_activity_time_median = agent.total_activity_time.Median()
        idle_quota[agent.name] = (makespan_low_median - total_activity_time_median)/makespan_median
        if agent.human:
            total_idle_penalty += max(agent.idle_min - idle_quota[agent.name], 0) * makespan_median
            if deterministic_costs:
                total_preferred_time_median = agent.total_preferred_activity_time_sim
            else:
                total_preferred_time_median = agent.total_preferred_activity_time.Median()
            if total_activity_time_median != 0:
                pref_quota[agent.name] = total_preferred_time_median/total_activity_time_median
                total_activity_penalty += max(agent.preference_min - pref_quota[agent.name], 0) * total_activity_time_median
            else:
                pref_quota[agent.name] = 1.0
    cost = makespan_cost + total_idle_penalty + total_activity_penalty
    feasible = total_idle_penalty == 0

    return  feasible, cost, makespan_cost, makespan_median, makespan_low_median, idle_quota, pref_quota

def FilterAndMerge(durations, process):
    for dur in durations:
        dur.MinMaxEstimate()
    max_min_estimate = max(durations, key= lambda x: x.min_estimate).min_estimate
    filtered_durations = []
    for dur in durations:
        if dur.max_estimate >= max_min_estimate:
            filtered_durations.append(dur)
    ret_durations = []
    while len(filtered_durations) > 1:
        most_dep_dur = MostDependentDuration(filtered_durations)
        ret_durations.append(most_dep_dur)
        filtered_durations.remove(most_dep_dur)
        filtered_durations.sort(key=lambda x: x.similarity_score, reverse = True)
        remaining = filtered_durations.copy()
        for dur in remaining:
            if most_dep_dur.Merge(dur, process):
                filtered_durations.remove(dur)
    if len(filtered_durations) == 1:
        ret_durations.append(filtered_durations[0])
    return ret_durations

def MostDependentDuration(durations):
    if len(durations) <= 2:
        return durations[0]
    else:
        for dur in durations:
            dur.ResetSimilarityScore()
        sim_score = -1
        md = None
        for dur1 in durations:
            for dur2 in durations:
                if dur1 != dur2:
                    sc = dur1.SimilarityScore(dur2)
                    if sc > sim_score:
                        sim_score = sc
                        md = dur1
        return md
