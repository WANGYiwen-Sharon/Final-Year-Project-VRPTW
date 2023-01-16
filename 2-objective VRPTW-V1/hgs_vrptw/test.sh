#!/bin/bash
# get all filename in specified path

path=$"../test"
files=$(ls $path)
for filename in $files
do
   ./genvrp ../test/$filename test.sol -seed 1 -t 60 >> multi_objective_result.txt
done
