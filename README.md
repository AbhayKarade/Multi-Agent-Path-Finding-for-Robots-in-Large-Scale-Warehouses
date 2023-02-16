# Multi-Agent Path Finding for Robots in Large-Scale Warehouses

The Multi-Agent Pathfinding (MAPF) is a crucial
problem of warehouse mobile robots to find collision free path
to the target position. In this paper, we explore MAPF algorithms
that produce an optimal and suboptimal path for a fixed number
of robots to navigate in a warehouse environment. In addition,
this paper also elaborates on the comparative analysis of the
studied algorithms, with increasing environmental complexity
and increasing number of agents. The compared parameters are
the total cost, flowtime and makespan of each algorithm.

## Algorithms Implemented:

CBS 
Prioritized Planning

## Running the scripts:

All environment are available in the instances folder 

Ex:

To run the CBS on warehouse environment no 6

```python
python run_experiments.py --instance instances/env6.txt --solver CBS
```

To run the Prioritized Planning on warehouse environment no 6

```python
python run_experiments.py --instance instances/env6.txt --solver Prioritized 
```

To compare the performance of CBS and Prioritized Planning

```python
python run_experiments.py --compare True
```



## References : 

-  [Planning Algorithms and Differential Models](http://lavalle.pl/planning/ch13.pdf)

-[msaudulhassan] (https://github.com/msaudulhassan)
