This folder contains the source code of the EGRD search algorithm, including a model checker for FO-CTL and the goal query graph translation algorithm.

The problem is written in C++20, and it has been successfully compiled using gcc version 11.2 on both Mac(macOS Big Sur version 11.5.2) and Linux (Ubuntu 18.04 LTE) platforms.

In the "executable" folder, you can find the executables of the program on the two platforms. You should be able to run these programs directly.

If you prefer compiling the program yourself, you will have to use the latest version of cmake, bison and flex.  The minimum version of cmake is 3.19.  The minimum version of bison is 3.7, and the minimum version of flex is 2.6.


The program accepts the following command line arguments:

  foctl -h

    -h  show the usage of the program.

  Focal [-v] [-dot] [-s depth is_use_pruned_reduce] config_filename env_filename gqs_filename

    -v    display information verbosely to standard output

    -dot  generate the dot (graph description language) files for the environment and the goal query statement.
          The dot files can be opened in Graphviz, a graph visualization software (https://graphviz.org)

    -s    run the EGRD search instead of evaluating of the goal query statement
          "depth" is the maximum search depth and "is_use_pruned_reduce" is an boolean value
          indicating whether pruned-reduce should be used in the EGRD search.  Note that
          pruned-reduce generates correct results only for certain goal query statement.

    config_filename   The configuration file. You can specify whether AP should be used instead
                      of EP when computing the WCD. You can also specify whether cache should
                      be used in model checking.

    env_filename      The environment file which specify the set of legal paths, the goals, and
                      the modification actions that can be used by the EGRD search.

    gqs_filename      The file containing the goal query statement.



The configuration file should contain two lines that specifies the boolean values of two variables:

  SEARCH_ALL_PATH false
  USE_CACHE true


If SEARCH_ALL_PATH is true, "AP" is used instead of "EP" when computing the WCD. However, as discussed in the technical appendix in the supplementary material, we should use EP most of the time.

If USE_CACHE is true, the caching mechanism is enabled.  If you disable the caching mechanism, both the evaluation of FO-CTL statement and the EGRD algorithm will be very slow.


The environment file should contains a sequence of commands, one on each line. There are six commands:


1) PATH_ENV

PATH_ENV specifies that the EGRD search should explore the legal paths only. An alternative to PATH_ENV is GRAPH_ENV, which do not limit the search to legal paths only. However, the semantics of an environment under GRAPH_ENV is different from of that under PATH_ENV.  Therefore, we recommend using PATH_ENV only.


2) GRID_LAYOUT

GRID_LAYOUT is an optional command.  If GRID_LAYOUT is used, the vertex number will be interpreted as a x-y coordinate of a cell in a grid, such that when the program generates the dot file, the vertex will be put in a grid. GRID_LAYOUT has two parameters: x and y, which are the dimension of the grid.  If a vertex number is i, then the x-y coordinate is (i % x and i/x)


3) START_VERTEX

START_VERTEX is the vertex id of the initial state in the environment. It takes one integer argument-the vertex id of the initial state. All legal paths should starts with this id.


4) PATH

PATH specifies a legal path. There can be more than one PATH command, but there should have at least one PATH command. The first argument of the PATH command is the name of the legal path, which can be any string.  Our current implementation of the program ignored the name of the legal path, and hence the name can be anything. After the first argument is a sequence of integers, each of them is a vertex id. The first vertex id in the list should be the vertex id specified in START_VERTEX.  The subsequent vertex ids forms a legal path in the environment.


5) GOAL

GOAL assigns goals to vertex id. A goal must be a string whose first letter must be a capital letter. The first argument of GOAL is a vertex id. The arguments after the first one is a sequence of goals that will be assigned to the vertex id. There can be more than one goal at each vertex.  Notice that the same vertex id can appear in several goal commands. Likewise, the same goal can appear in several different goal commands.


6) MOD

MOD defines a modification action. The arguments of MOD is a sequence of tuples, each of which specifies an addition or a deletion of an edge.  For edge addition, the tuple should be "ADD v1 v2", where v1 and v2 are the vertex ids of an edge. For edge deletion, the tuple should be "DEL v1 v2", where v1 and v2 are the vertex ids of an edge. Each edge should appear in at least one of the legal paths defined by the PATH command.  Note that there can be more than one ADD and DEL in a MOD command.



The goal query statement file contains the goal query statement.  Please see the supplementary material to see how to specify a goal query statement for a given goal query graph. Notice that the trace variable in the file must start with '$'.  If there are multiple traces in a goal query statement, the last trace will be the starting trace, and it should not be assigned to a trace variable.  

You can append additional constraints to the end of a goal query statement. The two constraints we have implemented are: (1) x1 != x2, which means that the variables x1 and x2 should not match with the same goal, and (2) x1 != G1, which means that the variable x1 should not match the goal G1. There can be multiple constraints and they should be separated by semicolons.




Author: Tsz-Chiu Au
Email: chiu.au@gmail.com
Date: 2021-12-15


