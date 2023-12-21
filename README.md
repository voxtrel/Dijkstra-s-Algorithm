# Dijkstra-s-Algorithm
README FILE

DIJKSTRA'S ALGORITHM

AUTHORS:

Yaniel Gonzalez

Carlos Caraballo

Jason Rodriguez

OVERVIEW 

This C++ project implements Dijkstra's algorithm to find the shortest path between two points in a graph. 
The program includes a graph class, a vertex class, and an edge class, all of which contribute to the 
implementation of an adjacency list. Dijkstra's algorithm, along with the use of a priority queue and a 
map, enables efficient traversal of the graph to determine the shortest path.

FEATURES 

-Dijkstra's Algorithm: Implements Dijkstra's algorithm to find the shortest path in a weighted undirected 
 graph.
 
-Adjacency List: Utilizes an adjacency list representation for efficient graph storage.

-Graph Operations: Provides methods for adding and removing vertices, adding and removing edges, and 
 finding the shortest path between two vertices.

DEPENDENCIES

This project relies on a C++ compiler that supports the C++11 standard or later.

USAGE

Compilation:

To compile the program, use the following command:

    g++ -Wall -std=c++17  Graph.cpp main.cpp

Input:

The program includes a user interface to execute its functions.

-Enter 'h' to read the list of commands to that the program includes

-Enter 'v' for adding a vertex

-Enter 'e' for adding an edge

-Enter 'd' for removing a vertex

-Enter 'r' for removing an edge

-Enter 's' for finding the shortest path between two vertices

-Enter 'l' for listing the current graph

-Enter 'q' to quit

CODE STRUCTURE

-'main.cpp': Contains the user interface for interacting with the program.

-'Graph.hpp' and 'Graph.cpp': Implement the graph class, which includes methods for graph operations and 
  Dijkstra's algorithm.
  
-'GraphBase.hpp': Contains the base class definition for the graph, to be specialized by Graph.hpp.

ACKNOLEDGEMENTS

The project was inspired by Dijkstra's algorithm, a key algorithm in graph theory for finding the shortest 
paths between nodes.
