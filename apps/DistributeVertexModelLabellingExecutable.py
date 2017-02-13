import multiprocessing
import os
import subprocess

import numpy as np

executable = '../../chaste_build/apps/VertexModelLabellingExecutable'

if not(os.path.isfile(executable)):
    raise Exception('Could not find executable: ' + executable)

number_of_simulations = 50

def main():
    run_simulations()


# Create a list of commands and pass them to separate processes
def run_simulations():

    # Make a list of calls to a Chaste executable
    command_list = []

    base_command = 'nice -n 19 ' + executable

    num_runs = 10
    num_labels = [1, 2, 3, 4]
    labelling_prop = [0.05, 0.1, 0.15, 0.2, 0.25]
    
    for l in num_labels
        for p in labelling_prop
            for s in range(num_runs):
                command = base_command + ' --L ' + str(l) + ' --P ' + str(p) + ' --R ' + str(num_runs) + ' --S ' + str(s)
                command_list.append(command)

    # Use processes equal to the number of cpus available
    count = multiprocessing.cpu_count()

    print("Starting simulations with " + str(count) + " processes")

    # Generate a pool of workers
    pool = multiprocessing.Pool(processes=count)

    # Pass the list of bash commands to the pool
    pool.map_async(execute_command, command_list).get(86400)


# This is a helper function for run_simulation that runs bash commands in separate processes
def execute_command(cmd):
    return subprocess.call(cmd, shell=True)

if __name__ == "__main__":
    main()