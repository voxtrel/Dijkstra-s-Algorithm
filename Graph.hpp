// Authors:
// Yaniel Gonzalez
// Carlos Caraballo
// Jason Rodriguez

// Header file

#ifndef GRAPH_H
#define GRAPH_H

#include "GraphBase.hpp"
#include <list>
#include <vector>

class Vertex;                   // Declaration of Vertex class to allow the Edge class definition to recognize.
class Edge;                     // Declaration of Edge class to allow the Vertex class definition to recognize.

class Vertex {
public:
    Vertex(const std::string val) : value(val) {}   // Vertex constructor
    ~Vertex();                                      // Vertex destructor
    std::string value;                              // Label of Vertex
    std::list<Edge *> incidentEdges;                // List of incident edges to this Vertex
    bool isAdjacentTo(Vertex *v);                   // Is current Vertex object adjacent to v?
    friend class Graph;                             // Provide Graph class access
};

class Edge {
public:
    Edge(Vertex *u, Vertex *v, unsigned long dist) : distance(dist), u(u), v(v) {}  // Edge constructor
    unsigned long distance;                         // weight of Edge
    Vertex *u;                                      // 1/2 of the adjacent endpoints of undirected Edge
    Vertex *v;                                      // 1/2 of the adjacent endpoints of undirected Edge
    Vertex *opposite(Vertex *v);                    // return Vertex opposite of v along the same Edge
    friend class Graph;                             // Provide Graph class access
};

class Graph : public GraphBase {                    // Graph class inherits from abstract class GraphBase
public:
    ~Graph();                                       // Graph destructor
    std::list<Vertex *> vertices;                   // List of all vertices
    Vertex* findVertex(std::string label);          // Helper function to find Vertex containing specified label
    void addVertex(std::string label);              // Adds a new Vertex to the Graph
    void removeVertex(std::string label);           // Removes Vertex with specified label alongside the incident edges of its adjacent vertices
    void addEdge(std::string label1, std::string label2, unsigned long weight); // Adds a weighted Edge in between two vertices containing specified labels
    void removeEdge(std::string label1, std::string label2);                    // Removes Edge between the two specified labels
    unsigned long shortestPath(std::string startLabel, std::string endLabel, std::vector<std::string> &path);   // Calculates the shortest path
    void print();       //Display all vertices and edges in the graph
    bool isEmpty();     //Check if the graph is empty
};

//Interface functions
void help();
void insertVertex(Graph &g);
void insertEdge(Graph &g);
void deleteVertex(Graph &g);
void deleteEdge(Graph &g);
void shortest(Graph &g);
bool isPosNumber(std::string str);

#endif