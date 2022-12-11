#!/usr/bin/python
import argparse
import glob
from pathlib import Path
from cbs import CBSSolver
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from visualize import Animation
from single_agent_planner import get_sum_of_cost, get_max_path
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.colors import ListedColormap

SOLVER = "CBS"


def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))]
                  for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@' or cell == 'T':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    # #agents
    line = f.readline()
    num_agents = int(line)
    # #agents lines with the start/goal positions
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    return my_map, starts, goals


def compare_performance():
    instances = ["instances/env3.txt",
                 "instances/env4.txt", "instances/env5.txt", "instances/env6.txt"]

    cbs_cost_list = []
    pp_cost_list = []

    CBS_time_list = []
    PP_time_list = []

    CBS_makespan_list = []
    PP_makespan_list = []

    for instance in instances:
        my_map, starts, goals = import_mapf_instance(instance)

        cbs = CBSSolver(my_map, starts, goals)
        cbs_paths, CBS_time = cbs.find_solution(
            args.disjoint, CBSSolver.NORMAL)
        cbs_cost = get_sum_of_cost(cbs_paths)
        CBS_makespan = get_max_path(cbs_paths)

        solver = PrioritizedPlanningSolver(my_map, starts, goals)
        pp_paths, PP_time = solver.find_solution()
        pp_cost = get_sum_of_cost(pp_paths)
        PP_makespan = get_max_path(pp_paths)

        cbs_cost_list.append(cbs_cost)
        pp_cost_list.append(pp_cost)
        CBS_time_list.append(CBS_time)
        PP_time_list.append(PP_time)
        CBS_makespan_list.append(CBS_makespan)
        PP_makespan_list.append(PP_makespan)

    print(cbs_cost_list, pp_cost_list, CBS_time_list,
          PP_time_list, CBS_makespan_list, PP_makespan_list)

    x_axis_lst = ["env3", "env4", "env5",  "env6"]
    font1 = {'family': 'serif', 'color': 'black', 'size': 20}
    font2 = {'family': 'serif', 'color': 'darkred', 'size': 15}
    plt.subplot(3, 1, 1)
    plt.scatter(x_axis_lst, cbs_cost_list, marker='o', label='CBS')
    plt.scatter(x_axis_lst, pp_cost_list, marker='^',
                label='Prioritized Planning')

    plt.xlabel('Environments')
    plt.ylabel('Total Cost')
    plt.title("Total Cost vs Environments")
    plt.legend(bbox_to_anchor=(1.70, 0.5), loc='right')

    plt.subplot(3, 1, 2)
    plt.scatter(x_axis_lst, CBS_time_list, marker='o', label='CBS')
    plt.scatter(x_axis_lst, PP_time_list, marker='^',
                label='Prioritized Planning')

    plt.xlabel('Environments')
    plt.ylabel('Total Time in ms')
    plt.title("Total Time in ms \\Environments")
    plt.legend(bbox_to_anchor=(1.70, 0.5), loc='right')

    plt.subplot(3, 1, 3)
    plt.scatter(x_axis_lst, CBS_makespan_list, marker='o', label='CBS')
    plt.scatter(x_axis_lst, PP_makespan_list,
                marker='^', label='Prioritized Planning')

    plt.xlabel('Environments')
    plt.ylabel('Makespan')
    plt.title("Makespan \\ Environments")
    plt.legend(bbox_to_anchor=(1.70, 0.5), loc='right')

    plt.suptitle("Performance Analysis", fontdict=font1)

    # plt.subplot_tool()
    plt.subplots_adjust(left=0.41,
                        bottom=0.17,
                        right=0.58,
                        top=0.88,
                        wspace=0.20,
                        hspace=0.4)

    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--disjoint', action='store_true', default=False,
                        help='Use the disjoint splitting')
    parser.add_argument('--solver', type=str, default=SOLVER,
                        help='The solver to use (one of: {CBS,Independent,Prioritized}), defaults to ' + str(SOLVER))
    parser.add_argument('--compare', type=str, default=False,
                        help='This is compare the performance of the CBS and Prioritized Planning algorithm')
    args = parser.parse_args()

    if args.compare == "True":
        print("Hello")
        compare_performance()

    result_file = open("results.csv", "w", buffering=1)

    print("args.instance", glob.glob(args.instance))

    for file in sorted(glob.glob(args.instance)):
        print("file", file)

        print("\n\n*********************************************************")
        print("***Import an instance***")
        print(file, end='\n\n')
        my_map, starts, goals = import_mapf_instance(file)
        print_mapf_instance(my_map, starts, goals)

        if args.solver == "CBS":
            print("***Run CBS***\n")
            cbs = CBSSolver(my_map, starts, goals)
            paths, CBS_time = cbs.find_solution(
                args.disjoint, CBSSolver.NORMAL)
            print("PP_time", CBS_time)
            print(paths)
        elif args.solver == "Independent":
            print("***Run Independent***\n")
            solver = IndependentSolver(my_map, starts, goals)
            paths = solver.find_solution()
        elif args.solver == "Prioritized":
            print("***Run Prioritized***\n")
            solver = PrioritizedPlanningSolver(my_map, starts, goals)
            paths, PP_time = solver.find_solution()
            print("PP_time", PP_time)
        else:
            raise RuntimeError("Unknown solver!")

        cost = get_sum_of_cost(paths)
        result_file.write("{},{}\n".format(file, cost))

        if not args.batch:
            print("***Test paths on a simulation***")
            animation = Animation(my_map, starts, goals, paths)
            animation.save("env6.mp4", 1.0)
            animation.show()

    result_file.close()
