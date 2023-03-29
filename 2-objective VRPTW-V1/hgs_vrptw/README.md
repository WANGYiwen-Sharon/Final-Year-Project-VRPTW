# Version Information



- 2023.1.15 - by yiwen wang

  implement the MOEA/D+HGS, experimental results on solomon benchmark outperform other algorithms in terms of distance

- 2023.1.19 - by Fei Liu

  improve the ability of searching solutions with minimum vehicle number.

- 2023.03.29 - by yiwen wang

 enhancing the multiple split strategy and adding restart operation

## Compiling the executable 

You need [`CMake`](https://cmake.org) to compile.

Build with:
```console
cd hgs_vrptw
make test
```

## Running the algorithm

* Run the make command: `make test`
* Try another example: `./genvrp ../../instances/R101.txt test.sol -seed 1 -t 30`

After building the executable, try an example: 
```console
./genvrp ../../instances/R101.txt test.sol -seed 1 -t 30
```

The following options are supported:
```
-it           Sets a maximum number of iterations without improvement. Defaults to 20,000
-t            Sets a time limit in seconds. If this parameter is set, the code will be restart iteratively until the time limit
-bks          Sets an optional path to a BKS in CVRPLib format. This file will be overwritten in case of improvement 
-seed         Sets a fixed seed. Defaults to 0     
-veh          Sets a prescribed fleet size. Otherwise a reasonable UB on the fleet size is calculated       
```

There exist different conventions regarding distance calculations in the academic literature.
The default code behavior is to apply integer rounding, as it should be done on the X instances of Uchoa et al. (2017).
To change this behavior (e.g., when testing on the CMT or Golden instances), give a flag `-round 0`, when you run the executable.

## Code structure

The main classes containing the logic of the algorithm are the following:
* **Individual**: Represents an individual solution in the genetic algorithm, also provide I/O functions to read and write individual solutions in CVRPLib format.
* **Population**: Stores the solutions of the genetic algorithm into population. Also includes the functions in charge of diversity management.
* **Genetic**: Contains the main procedures of the genetic algorithm as well as the crossover, multiple split and selection.
* **LocalSearch**: Includes the local search functions, including the SWAP* neighborhood. A small code used to represent and manage arc sectors (to efficiently restrict the SWAP* neighborhood).
* **Params**: Stores the method parameters, instance data and I/O functions.
* **Commandline**: Reads the line of command.
* **Split**: Contains all the methods to split the customer sequences into routes.
* **main**: Main code to start the algorithm.
