import CPM
import CPMGA
import CPMGrasp
import CPMsup
import SDUR
import CPMUC
import sys
from timeit import default_timer as timer
import matplotlib.pyplot as plt

###############################################################################
# MAIN program
#

no_of_use_cases = len(sys.argv) - 1
eval_rows = 5
eval_cols = 2
eval_plots_created = False

for uc_index in range(0, no_of_use_cases):
    # Load arguments
    value = False
    param = ""
    arguments = []
    arguments.extend(sys.argv[uc_index + 1].split())
    arguments.append("dummy")

    # Default selections
    genetic_algorithm = False
    warm_start = False
    grasp_algorithm = False
    evaluate_solution = False
    deterministic_costs = False
    debug_mode = False
    plot_factory_locations = False
    plot_process_model = False
    plot_task_constraints = False
    plot_agent_constraints = False
    plot_agent_sequences = False
    plot_task_locations = False
    plot_schedule_durations = False
    seed = -1
    load_random_state = False
    old_random_state_path = ""
    log_path = "array"
    use_case = "1"
    no_of_runs = 1
    first_run = 0
    no_of_robots = 1
    no_of_humans = 1
    percentile = 0.5
    idle_min = 0.0
    preference_min = 0.0
    no_of_iterations = 1
    population = 200
    generations = 1
    timeout = 16200    # Time limit in seconds for scheduling algorithms
    no_of_monte_carlo_sim = 0

    for arg in arguments:
        if arg[0] == "-":
            param = arg
            value = True
        elif value:
            value = False
            if param == "-uc":
                use_case_name = arg
            elif param == "-r":
                no_of_runs = int(arg)
            elif param == "-fr":
                first_run = int(arg)
            elif param == "-rob":
                no_of_robots = int(arg)
            elif param == "-hum":
                no_of_humans = int(arg)
            elif param == "-perc":
                percentile = float(arg)
            elif param == "-idle_min":
                idle_min = float(arg)
            elif param == "-pref_min":
                preference_min = float(arg)
            elif param == "-esol":
                evaluate_solution = (arg == "True")
            elif param == "-gr":
                grasp_algorithm = (arg == "True")
            elif param == "-iter":
                no_of_iterations = int(arg)
            elif param == "-ga":
                genetic_algorithm = (arg == "True")
            elif param == "-warm":
                warm_start = (arg == "True")
            elif param == "-pop":
                population = int(arg)
            elif param == "-gen":
                generations = int(arg)
            elif param == "-timeout":
                timeout = int(arg)
            elif param == "-mc":
                no_of_monte_carlo_sim = int(arg)
            elif param == "-dc":
                deterministic_costs = (arg == "True")
            elif param == "-dbg":
                debug_mode = (arg == "True")
            elif param == "-plfl":
                plot_factory_locations = (arg == "True")
            elif param == "-plpm":
                plot_process_model = (arg == "True")
            elif param == "-pltc":
                plot_task_constraints = (arg == "True")
            elif param == "-plac":
                plot_agent_constraints = (arg == "True")
            elif param == "-plas":
                plot_agent_sequences = (arg == "True")
            elif param == "-pltl":
                plot_task_locations = (arg == "True")
            elif param == "-plpd":
                plot_schedule_durations = (arg == "True")
            elif param == "-seed":
                seed = int(arg)
            elif param == "-ld":
                # Load the state of the random generators.
                # Can be used to restart the experiments from a certain point.
                load_random_state = True
                old_random_state_path = arg
            elif param == "-log":
                # Log file name
                log_path = arg
            else:
                print("Error: Unknown command %s"%(param))
                quit()

    task_locations, agent_locations, process_locations = CPMUC.GetLocations(1, 5)
    if plot_factory_locations:
        fig, axs = plt.subplots(1, num = "Factory locations")
        CPMsup.PlotFactory(task_locations, agent_locations, axs, process_locations)
        plt.show(block = False)

    # Setup random generators for use case
    if seed >= 0:
        print("Setting use case seed "+str(seed))
        repeatable_results = True
    else:
        print("No seed")
        repeatable_results = False
    rand_gen = CPMsup.RandomGenerators(repeatable_results, seed)

    SDUR.stochastic_debug = debug_mode
    CPM.debug_mode = debug_mode
    CPMGA.debug_mode = debug_mode

    if genetic_algorithm or grasp_algorithm:
        SDUR.SetModelSize(2800, 5E-1)
    else:
        SDUR.SetModelSize(2800, 1E-1)

    ########################################################
    # 1) Setup the use case:
    #    - Create a process model and generate tasks
    ########################################################
    proc1 = CPMUC.SetupUseCase(use_case_name, process_locations)
    if plot_process_model:
        plt.figure("Process model "+ use_case_name)
        CPMsup.PlotCPM(proc1)
        plt.show(block = False)

    # Generate all tasks. Additionally,
    # generate all constraints of types:
    # -succeeding_tasks, preceding_tasks
    # -task_start_preceding_start, task_start_succeeding_start
    # -task_completion_preceding_completion, task_completion_succeeding_completion
    CPM.GenerateTasks(proc1)
    no_of_tasks = CPM.GenerateTaskIndexes(proc1)

    # Generate all constraints of types:
    # -task_start_preceding_completion, task_completion_succeeding_start
    CPM.GenerateOutputBufferConstraints(proc1)

    # Compute all precedence constraints for every task that must hold in a sequence for any agent
    CPM.ComputeIntraScheduleConstraints(proc1)

    # Compute all tasks that due to precedence constraints must complete after the completion of a given task
    CPM.ComputeAllSucceedingTaskCompletions(proc1)

    last_task = proc1.LastProcess().tasks[-1]

    if plot_task_constraints and not evaluate_solution:
        plt.figure("General task constraints")
        CPMsup.PlotCPMTaskConstraints(proc1)
        plt.show(block = False)
        # CPMsup.PrintTaskConstraints(proc1)

    ########################################################
    # 2) Create the agents
    ########################################################
    agents = CPMUC.CreateAgents(proc1, no_of_robots, no_of_humans, idle_min, preference_min)

    if genetic_algorithm:
        Genetic = CPMGA.GA(proc1, agents, percentile, population, generations, rand_gen, warm_start)

    if grasp_algorithm:
        Grasp = CPMGrasp.GR(proc1, agents, percentile, no_of_iterations, rand_gen)

    ########################################################
    # 3) Randomize initial conditions,
    #    including task and routing durations
    ########################################################

    # Randomize agent locations
    CPMUC.RandomizeAgentLocations(agents, agent_locations, rand_gen)

    # Randomize task locations
    locations = CPMUC.RandomizeTaskLocations(proc1, task_locations, agent_locations, rand_gen)
    if plot_task_locations and not evaluate_solution:
        fig, axs = plt.subplots(1, num = "Task locations")
        CPMsup.PlotTaskLocations(axs, proc1, agents)
        plt.show(block = False)

    # Compute randomized routing durations
    transition_costs = CPMUC.ComputeRandomizedTransitionDurations(locations)

    # Randomize task durations
    CPMUC.RandomizeTaskDurations(proc1, rand_gen)

    if genetic_algorithm:
        Genetic.SetTransitionCosts(transition_costs, deterministic_costs)
    if grasp_algorithm:
        Grasp.SetTransitionCosts(transition_costs, deterministic_costs)

    ########################################################
    # 4) Solve the scheduling problem
    ########################################################
    CPMsup.CreateLogFile(log_path, agents)
    if genetic_algorithm:
        CPMsup.CreateConvergenceLogFile(log_path+"_generations")
    if grasp_algorithm:
        CPMsup.CreateConvergenceLogFile(log_path+"_iterations")

    for r in range(first_run, no_of_runs):
        if r == first_run and load_random_state:
            # Load saved random state
            print("Loading random state from %s"%(old_random_state_path))
            rand_gen.LoadState("random_states//"+old_random_state_path+".data")
        # Save current random state
        rand_gen.SaveState("random_states//"+log_path+"_run_"+ str(r)+".data")

        if genetic_algorithm:
            exec_time = timer()
            cost_count, cost_time = Genetic.RunAlgorithm(r, timeout, log_path+"_generations")
            exec_time = timer() - exec_time
            if no_of_runs > 1:
                makespan, makespan_low = last_task.ExploreMakespan(transition_costs)
            else:
                # Get makespan for plot
                CPM.debug_mode = True # Plot everything
                makespan, makespan_low = last_task.ExploreMakespan(transition_costs)
            CPMsup.AppendSolutionToLogFiles(use_case_name, no_of_tasks, r, "GA", makespan, makespan_low, agents, percentile, cost_count, cost_time, exec_time, log_path)
        elif grasp_algorithm:
            exec_time = timer()
            cost_count, cost_time = Grasp.RunAlgorithm(r, timeout, log_path+"_iterations")
            exec_time = timer() - exec_time
            if no_of_runs > 1:
                makespan, makespan_low = last_task.ExploreMakespan(transition_costs)
            else:
                # Get makespan for plot
                CPM.debug_mode = True # Plot everything
                makespan, makespan_low = last_task.ExploreMakespan(transition_costs)
            CPMsup.AppendSolutionToLogFiles(use_case_name, no_of_tasks, r, "GR", makespan, makespan_low, agents, percentile, cost_count, cost_time, exec_time, log_path)
        elif evaluate_solution:
            # Apply a specified solution for a given use case
            CPMUC.ApplySolution(proc1, agents, use_case_name)

            # Get makespan for plot
            CPM.debug_mode = True # Plot everything
            # Compute stochastic makespan
            dependency_order, moved_tasks = CPM.RepairInterScheduleDeadLocks(proc1, agents)
            if len(moved_tasks) > 0:
                raise Exception('Solution had deadlocks!')
            makespan, makespan_low = last_task.ExploreMakespan(transition_costs)
            # Compute deterministic makespan
            makespan_det, _ = last_task.ExploreMakespan(transition_costs, True, dependency_order, agents)

            kld_val = makespan.KullbackLeiblerDivergence(makespan_low)
        else:
            raise Exception("Undefined experiment")

    if evaluate_solution or no_of_runs == 1:
        # Prepare for Monte Carlo simulations
        proc1.ResetDependencyOrder()
        dependency_order, moved_tasks = CPM.RepairInterScheduleDeadLocks(proc1, agents)
        if moved_tasks != []:
            raise Exception("Unresolved deadlocks detected in the final solution")

        if no_of_monte_carlo_sim > 0:
            # Perform Monte Carlo simulations
            print("Starting Monte Carlo")
            time1 = timer()
            CPM.MonteCarloSimulate(proc1, dependency_order, agents, transition_costs, no_of_monte_carlo_sim)
            time2 = timer()
            print("Finished Monte Carlo")
            print("Execution time: %f ms"%(1000*(time2 - time1)))

        print("\nUse case " + use_case_name)
        CPMsup.PrintAgentSequences(agents)

    if no_of_runs == 1 and not evaluate_solution:
        # Plot various solution information
        if plot_agent_constraints:
            for agent in agents:
                plt.figure("Agent "+ agent.name + " task constraints")
                CPMsup.PlotCPMTaskConstraints(proc1, agent)
                plt.show(block = False)

        if plot_agent_sequences:
            plt.figure("Agent sequences")
            CPMsup.PlotCPMTaskSequences(agents)
            plt.show(block = False)
            fig, axs = plt.subplots(1, num = "Agent routes")
            CPMsup.PlotRoutes(axs, agents)
            plt.show(block = False)

        if plot_schedule_durations:
            min_x = -10.0
            max_x = makespan.MaxEstimate()

            # Schedule durations
            no_of_subplots = 2*len(agents)
            fig, axs = plt.subplots(no_of_subplots, num = "Schedule durations")
            if grasp_algorithm:
                title = "GRASP, iter " + str(no_of_iterations) + ", makespan " + str(round(makespan.AtPercentile(percentile),1))
            elif genetic_algorithm:
                title = "GA, pop " + str(population) + ", gen " + str(generations) + ", makespan " + str(round(makespan.AtPercentile(percentile),1))
            else:
                title = "Check given solution" + ", makespan " + str(round(makespan.AtPercentile(percentile),1))
            fig.suptitle(title)
            for i in range(0, len(agents)):
                CPMsup.PlotStartAndCompletionDurations(agents[i], axs[i], True, percentile, min_x, max_x)
            for i in range(0, len(agents)):
                CPMsup.PlotStartAndCompletionDurations(agents[i], axs[i+len(agents)], False, percentile, min_x, max_x)

            # Activity durations and idle durations
            no_of_subplots = 2*len(agents)
            fig, axs = plt.subplots(no_of_subplots, num = "Activity and idle durations")
            if genetic_algorithm:
                title = "GA, pop " + str(population) + ", gen " + str(generations) + ", median makespan " + str(round(makespan.Median(),1))
            elif grasp_algorithm:
                title = "GRASP, iter " + str(no_of_iterations) + ", median makespan " + str(round(makespan.Median(),1))
            fig.suptitle(title)
            for i in range(0, len(agents)):
                CPMsup.PlotOtherDurations(agents[i], axs[i], True, True, makespan_low, min_x, max_x)
            for i in range(0, len(agents)):
                CPMsup.PlotOtherDurations(agents[i], axs[i+len(agents)], True, False, makespan_low, min_x, max_x)

    if evaluate_solution:
        # Plot stochastic makespan information for solution
        if not eval_plots_created:
            fig_eval, axs_eval = plt.subplots(eval_rows, eval_cols, figsize=(9, 10), num = "Makespan")
            fig_eval.tight_layout()
            eval_plots_created = True
        min_x = makespan_low.MinEstimate()
        max_x = makespan.MaxEstimate()
        r = uc_index // eval_cols
        c = uc_index % eval_cols
        x_label = y_label = legend = False
        ax = axs_eval[r][c]
        if r == eval_rows - 1:
            x_label = True
        if c == 0:
            y_label = True
        if r == 0 and c == 0:
            legend = True
        ax.set_title(use_case_name)
        CPMsup.PlotTask(last_task, makespan, makespan_low, makespan_det, proc1, ax, x_label, y_label, legend, kld_val)

plt.show(block = True)

print("CPM Experiments completed")
