# Stochastic Scheduling

<!-- [Paper location](https://xxx.xx.xxx.xx/x_xxxxxxx/xxxx.pdf) -->

This repository contains Python code for running the experiments in our paper. It includes use cases and a stochastic scheduling framework.

<!-- ![image](xxxxxxxx_xxxxx.jpg) -->

## Files and Directories
Structure of the repository:
* `CPM_experiments.py`: Entry point for running experiments.
* `CPMUC.py`: Modeling of use cases.
* `SDUR.py`: Modeling of stochastic durations.
* `CPM.py`: 
    * Modeling of manufacturing process, agents and team formations.
    * Construction of task network.
    * Stochastic makespan, solution cost.
    * Deadlock handling. 
    * Monte Carlo simulations.
* `CPMGrasp.py`: GRASP algorithm with heuristic function.
* `CPMGA.py`: GA and GAW algorithms.
* `CPMsup.py`: Visualization and logging.
* `logs`: Folder where log files are saved to indicate the progress of the running algorithm.
* `random_states`: Folder where states for random number generators are saved.

## Dependencies
* Python 3.9.13
* numpy 2.2.4
* scipy 1.15.2
* matplotlib 3.7.5
* networkx 3.2.1

## Makespans of best schedules found
Command for generating a plot with stochastic makespans computed for the best schedules found.
```
python 'CPM_experiments.py' '-uc 1A -rob 2 -hum 1 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -esol True -r 1 -fr 0 -iter 100000 -pop 200 -gen 10000 -mc 100000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 1 -log uc_1A_esol' '-uc 1B -rob 3 -hum 2 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -esol True -r 1 -fr 0 -iter 100000 -pop 200 -gen 10000 -mc 100000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 2 -log uc_1B_esol' '-uc 2A -rob 2 -hum 1 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -esol True -r 1 -fr 0 -iter 100000 -pop 200 -gen 10000 -mc 100000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 3 -log uc_2A_esol' '-uc 2B -rob 3 -hum 2 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -esol True -r 1 -fr 0 -iter 100000 -pop 200 -gen 10000 -mc 100000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 4 -log uc_2B_esol' '-uc 3A -rob 2 -hum 1 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -esol True -r 1 -fr 0 -iter 100000 -pop 200 -gen 10000 -mc 100000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 5 -log uc_3A_esol' '-uc 3B -rob 3 -hum 2 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -esol True -r 1 -fr 0 -iter 100000 -pop 200 -gen 10000 -mc 100000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 6 -log uc_3B_esol' '-uc 4A -rob 2 -hum 1 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -esol True -r 1 -fr 0 -iter 100000 -pop 200 -gen 10000 -mc 100000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 7 -log uc_4A_esol' '-uc 4B -rob 3 -hum 2 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -esol True -r 1 -fr 0 -iter 100000 -pop 200 -gen 10000 -mc 100000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 8 -log uc_4B_esol' '-uc 5A -rob 2 -hum 1 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -esol True -r 1 -fr 0 -iter 100000 -pop 200 -gen 10000 -mc 100000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 9 -log uc_5A_esol' '-uc 5B -rob 3 -hum 2 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -esol True -r 1 -fr 0 -iter 100000 -pop 200 -gen 10000 -mc 100000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 10 -log uc_5B_esol'
```

## Convergence of GRASP
Commands for running the GRASP algorithm for use cases 1A, 1B, 2A, 2B, 3A, 3B, 4A, 4B, 5A and 5B. Generates log files which are updated periodically and indicate various data, for example the cost of the best solution over time (convergence).
```
python 'CPM_experiments.py' '-uc 1A -rob 2 -hum 1 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr True -ga False -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 1 -log uc_1A_gr'

python 'CPM_experiments.py' '-uc 1B -rob 3 -hum 2 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr True -ga False -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 2 -log uc_1B_gr'

python 'CPM_experiments.py' '-uc 2A -rob 2 -hum 1 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr True -ga False -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 3 -log uc_2A_gr'

python 'CPM_experiments.py' '-uc 2B -rob 3 -hum 2 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr True -ga False -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 4 -log uc_2B_gr'

python 'CPM_experiments.py' '-uc 3A -rob 2 -hum 1 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr True -ga False -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 5 -log uc_3A_gr'

python 'CPM_experiments.py' '-uc 3B -rob 3 -hum 2 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr True -ga False -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 6 -log uc_3B_gr'

python 'CPM_experiments.py' '-uc 4A -rob 2 -hum 1 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr True -ga False -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 7 -log uc_4A_gr'

python 'CPM_experiments.py' '-uc 4B -rob 3 -hum 2 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr True -ga False -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 8 -log uc_4B_gr'

python 'CPM_experiments.py' '-uc 5A -rob 2 -hum 1 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr True -ga False -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 9 -log uc_5A_gr'

python 'CPM_experiments.py' '-uc 5B -rob 3 -hum 2 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr True -ga False -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 10 -log uc_5B_gr'

```

## Convergence of GA
Commands for running the GA algorithm for use cases 1A, 1B, 2A, 2B, 3A, 3B, 4A, 4B, 5A and 5B. Generates log files which are updated periodically and indicate various data, for example the cost of the best solution over time (convergence).
```
python 'CPM_experiments.py' '-uc 1A -rob 2 -hum 1 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr False -ga True -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 1 -log uc_1A_ga'

python 'CPM_experiments.py' '-uc 1B -rob 3 -hum 2 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr False -ga True -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 2 -log uc_1B_ga'

python 'CPM_experiments.py' '-uc 2A -rob 2 -hum 1 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr False -ga True -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 3 -log uc_2A_ga'

python 'CPM_experiments.py' '-uc 2B -rob 3 -hum 2 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr False -ga True -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 4 -log uc_2B_ga'

python 'CPM_experiments.py' '-uc 3A -rob 2 -hum 1 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr False -ga True -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 5 -log uc_3A_ga'

python 'CPM_experiments.py' '-uc 3B -rob 3 -hum 2 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr False -ga True -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 6 -log uc_3B_ga'

python 'CPM_experiments.py' '-uc 4A -rob 2 -hum 1 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr False -ga True -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 7 -log uc_4A_ga'

python 'CPM_experiments.py' '-uc 4B -rob 3 -hum 2 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr False -ga True -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 8 -log uc_4B_ga'

python 'CPM_experiments.py' '-uc 5A -rob 2 -hum 1 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr False -ga True -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 9 -log uc_5A_ga'

python 'CPM_experiments.py' '-uc 5B -rob 3 -hum 2 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr False -ga True -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 10 -log uc_5B_ga'

```
## Convergence of GAW
Commands for running the GAW algorithm for use cases 1A, 1B, 2A, 2B, 3A, 3B, 4A, 4B, 5A and 5B. Generates log files which are updated periodically and indicate various data, for example the cost of the best solution over time (convergence).
```
python 'CPM_experiments.py' '-uc 1A -rob 2 -hum 1 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr False -ga True -warm True -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 1 -log uc_1A_gaw'

python 'CPM_experiments.py' '-uc 1B -rob 3 -hum 2 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr False -ga True -warm True -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 2 -log uc_1B_gaw'

python 'CPM_experiments.py' '-uc 2A -rob 2 -hum 1 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr False -ga True -warm True -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 3 -log uc_2A_gaw'

python 'CPM_experiments.py' '-uc 2B -rob 3 -hum 2 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr False -ga True -warm True -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 4 -log uc_2B_gaw'

python 'CPM_experiments.py' '-uc 3A -rob 2 -hum 1 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr False -ga True -warm True -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 5 -log uc_3A_gaw'

python 'CPM_experiments.py' '-uc 3B -rob 3 -hum 2 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr False -ga True -warm True -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 6 -log uc_3B_gaw'

python 'CPM_experiments.py' '-uc 4A -rob 2 -hum 1 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr False -ga True -warm True -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 7 -log uc_4A_gaw'

python 'CPM_experiments.py' '-uc 4B -rob 3 -hum 2 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr False -ga True -warm True -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 8 -log uc_4B_gaw'

python 'CPM_experiments.py' '-uc 5A -rob 2 -hum 1 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr False -ga True -warm True -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 9 -log uc_5A_gaw'

python 'CPM_experiments.py' '-uc 5B -rob 3 -hum 2 -perc 0.5 -idle_min 0.4 -pref_min 0.5 -gr False -ga True -warm True -r 20 -fr 0 -iter 1000000 -pop 200 -gen 100000 -timeout 16200 -mc 500000 -dbg False -plfl False -plpm False -pltc False -plac False -plas True -pltl False -plpd True -seed 10 -log uc_5B_gaw'

```

<!-- ## Citation
If you find this stochastic scheduling framework useful for your research, please cite this work:
```
<A reference will be added here later>
``` -->
