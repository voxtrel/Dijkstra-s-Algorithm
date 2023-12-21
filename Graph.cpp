// Authors:
// Yaniel Gonzalez 
// Carlos Caraballo
// Jason Rodriguez

// This program is an implementation of an undirected weighted Graph ADT using an Adjacency List. 
// The main functionality is to find the shortest path between two vertices using Dijkstra's Algorithm.

#include "Graph.hpp"
#include <iostream>
#include <map>
#include <queue>
#include <limits>

Vertex::~Vertex() {                         // Vertex destructor:
    while (!incidentEdges.empty()) {        // While the incident edges list in Vertex object is not empty,
        Edge *e = incidentEdges.front();    // Store the Edge object into a temporary variable,
        incidentEdges.pop_front();          // Remove the front element of the incident edge's list,
        delete e;                           // Deallocate the memory for the Edge object.
    }
}

Graph::~Graph() {                           // Graph destructor:
    while (!vertices.empty()) {             // While the vertices list is not empty,
        Vertex *v = vertices.front();       // Store the Vertex object into a temporary variable,
        vertices.pop_front();               // Remove the front element of the vertices list,
        delete v;                           // Deallocate the memory for the Vertex object (and all the incident edges from the Vertex destructor).
    }
}

Vertex* Edge::opposite(Vertex *v) {             // return Vertex opposite of v along the same Edge:
    return (this->v == v) ? this->u : this->v;  // Since each Edge object is undirected, the opposite value can be either u or v
}

bool Vertex::isAdjacentTo(Vertex *v) {          // Is current Vertex object adjacent to v?
    if (v == nullptr) {                         // If the Vertex passed does not exist,
        return false;                           // It is not adjacent.
    }
    for (auto &edge : incidentEdges) {          // For all incident edges in Vertex object,
        if (edge->u == v || edge->v == v) {     // If either this edge's u or v equals the v from the argument,
            return true;                        // It is adjacent.
        }
    }
    return false;                               // By default it is not adjacent. 
}

Vertex* Graph::findVertex(std::string label) {  // Helper function to return Vertex containing specified label:
    for (auto &vertex : vertices) {             // For all vertices in vertices list,
        if (vertex->value == label) {           // If the label contained at Vertex matches the label passed as an argument,
            return vertex;                      // Return Vertex.
        }
    }
    return nullptr;                             // By default return null.
}

void Graph::addVertex(std::string label) {          // Adds a new Vertex to the graph:
    if (findVertex(label) == nullptr) {             // If the Vertex of the given label does not already exist,
        Vertex *newVertex = new Vertex(label);      // Allocate memory for the new Vertex object containing given label,
        vertices.push_back(newVertex);              // Add the Vertex to the list of vertices.
        std::cout << "Vertex added" << std::endl;
    }
}

void Graph::removeVertex(std::string label) {       // Removes Vertex with specified label alongside the incident edges of its adjacent vertices:
    
    Vertex *u = findVertex(label);                  // Return Vertex containing specified label.
    if (u == nullptr) {                             // If Vertex does not exist,
        return;                                     // Exit function.
    }

    for (auto &edge : u->incidentEdges) {                       // For all Edge objects incident to Vertex,
        Vertex *v = edge->opposite(u);                          // Go to the adjacent Vertex,
        for (auto &oppositeEdge : v->incidentEdges) {           // For all Edge objects incident to adjacent Vertex,
            if (oppositeEdge->v == v || oppositeEdge->u == v) { // Check for the u or v Vertex objects containing Vertex that will be removed,
                v->incidentEdges.remove(oppositeEdge);          // Remove such Edge from the std::list structure,
                delete oppositeEdge;                            // Deallocate memory for such Edge object,
                break;                                          // Break out of inner-loop.
            }
        }
        delete edge;                                        // Deallocate memory for each Edge object incident to Vertex that will be removed.
    }
    u->incidentEdges.clear();                           // Clear the std::list incidentEdges structure in Vertex object,
    vertices.remove(u);                                 // Remove Vertex object from the std::list vertices list structure,  
    delete u;                                           // Deallocate memory for the Vertex object.
    std::cout << "Vertex removed" << std::endl;
}

void Graph::addEdge(std::string label1, std::string label2, unsigned long weight) { // Add an edge with a weight between two vertices
    
    Vertex *u = findVertex(label1);                             // Find Vertex with label 1
    Vertex *v = findVertex(label2);                             // Find Vertex with lebel 2

    if (u == nullptr || v == nullptr || u->isAdjacentTo(v)) {   // If the vertices do not exist or if they are already adjacent to eachother,
        std::cout << "Edge already exists" << std::endl;
        return;                                                 // Exit function.
    }

    Edge *newEdgeU = new Edge(u, v, weight);                    // Create Edge with source u going to v and the specified weight,
    u->incidentEdges.push_back(newEdgeU);                       // Add the Edge to the list of incident edges on u.

    Edge *newEdgeV = new Edge(v, u, weight);                    // Create Edge with source v going to u and the specified weight,
    v->incidentEdges.push_back(newEdgeV);                       // Add the Edge to the list of incident edges on v.

    std::cout << "Edge added" << std::endl;
}

void Graph::removeEdge(std::string label1, std::string label2) {
    
    Vertex *u = findVertex(label1);                             // Find Vertex with label 1
    Vertex *v = findVertex(label2);                             // Find Vertex with label 2
    if (u == nullptr || v == nullptr || !u->isAdjacentTo(v)) {  // If the vertices do not exist or if they are already not adjacent to eachother,
        std::cout << "Edge does not exist" << std::endl;
        return;                                                 // Exit function.
    }

    for (auto &edge : u->incidentEdges) {                       // For all Edge objects incident on u,
        if ((edge->u == u && edge->v == v) || (edge->u == v && edge->v == u)) { // If either Edge object's u and v equal the u and v found above,
            u->incidentEdges.remove(edge);                      // Remove Edge object from std::list incident edges structure,
            delete edge;                                        // Deallocate memory for such Edge object,
            break;                                              // Break from loop.
        }
    }

    for (auto &edge : v->incidentEdges) {                       // For all Edge objects incident on v,
        if ((edge->u == u && edge->v == v) || (edge->u == v && edge->v == u)) { // If either Edge object's u and v equal the u and v found above,
            v->incidentEdges.remove(edge);                      // Remove Edge object from std::list incident edges structure,
            delete edge;                                        // Deallocate memory for such Edge object,
            break;                                              // Break from loop.
        }
    }
    std::cout << "Edge removed" << std::endl;

}

unsigned long Graph::shortestPath(std::string startLabel, std::string endLabel, std::vector<std::string> &path) {
    
    std::map<Vertex *, unsigned long> dist;                     // Create map to store the distances of each Vertex object with respect to the start Vertex.
    std::map<Vertex *, Vertex *> prev;                          // Create map to store the previous Vertex object visited prior to the Vertex at key.

    Vertex *start = findVertex(startLabel);                     // Find Vertex with start label.
    Vertex *end = findVertex(endLabel);                         // Find Vertex with end label.
    
    if (start == nullptr || end == nullptr) {                   // If neither of the Vertex objects exist,
        return 0;                                               // Return a distance of 0.
    }

    dist[start] = 0;                                                    // Initialize the start Vertex object's distance to 0.
    for (auto &vertex : vertices) {                                     // For all Vertex objects,
        if (vertex != start) {                                          // So long as the current Vertex object is not the starting point, 
            dist[vertex] = std::numeric_limits<unsigned long>::max();   // Initialize the distance of Vertex to infinity (since path has yet to be calculated).
        }
    }

    // Create a priority queue that stores Vertex objects in order with priority on their distances traveled.
    std::priority_queue<std::pair<Vertex *, unsigned long>, std::vector<std::pair<Vertex *, unsigned long>>, std::greater<std::pair<Vertex *, unsigned long>>> pq;
    pq.push({start, dist[start]});                          // Insert the start Vertex object in queue.
    while (!pq.empty()) {                                   // While the priority queue is not empty,
        Vertex *u = pq.top().first;                         // Store the Vertex object at the top of the queue into a temporary variable u,
        pq.pop();                                           // Remove the top Vertex object from queue.
        for (auto &edge : u->incidentEdges) {               // For all Edge objects incident to u,
            Vertex *v = edge->opposite(u);                  // Go to adjacent Vertex, v,       
            if (dist[u] + edge->distance < dist[v]) {       // If the distanced traveled to get to u plus the current distance of u to v is less than v's distance, 
                dist[v] = dist[u] + edge->distance;         // Perform edge relaxation,
                prev[v] = u;                                // Store u as the previous Vertex visited prior to visiting v,
                pq.push({v, dist[v]});                      // Insert Vertex v and its new shorter distance back into priority queue.
            }
        }
    }
    for (Vertex *u = end; u != nullptr; u = prev[u]) {      // Starting at the end Vertex, traverse through all the previous vertices that created the shortest path, 
        path.insert(path.begin(), u->value);                // And insert it into the front of the path vector (since the order is reversed).
    }
    return dist[end];                                       // Return the shortest distance of the end Vertex from the start Vertex.

}

void Graph::print() {                                   // Print function to help visualize adjacency list structure:
    for (auto &vertex : vertices) {                     // For all Vertex objects in vertices list,
        std::cout << vertex->value << ": ";             // Print the value of the current Vertex object,
        for (auto &edge : vertex->incidentEdges) {      // For all Edge objects incident on the current Vertex object,
            // Print the adjacent Vertex object's value and the weight between
            std::cout << "-> {'" << (edge->u == vertex ? edge->v->value : edge->u->value) << "', " << edge->distance << "} ";
        }
        std::cout << std::endl;
    }
}

bool Graph::isEmpty(){
    return vertices.empty();
}

//Main Interface functions
void help() {
	std::cout << "List of operation codes:" << std::endl;
	std::cout <<("\t'h' for help") << std::endl;
	std::cout <<"\t'v' for adding a vertex" << std::endl;
	std::cout <<"\t'e' for adding an edge" << std::endl;
	std::cout <<"\t'd' for removing a vertex" << std::endl;
	std::cout <<"\t'r' for removing an edge" << std::endl;
    std::cout <<"\t's' for finding the shortest path between two vertices" << std::endl;
	std::cout <<"\t'c' for clear all" << std::endl;
    std::cout <<"\t'l' for listing the current graph" << std::endl;
	std::cout <<"\t'q' for quit" << std::endl;
}

//Functions for receiving and handling input
void insertVertex(Graph &g){

    std::string label;
    std::cout << "Enter the label for the new Vertex: ";
    std::cin >> label;

    //Validate input for new vertex label
    while(g.findVertex(label)!=nullptr){
        std::cout << std::endl;
        std::cout << "Vertex with given label already exists, try again" << std::endl;

        std::cout << "Enter the label for the new Vertex: ";
        std::cin >> label;
    } 
    
    g.addVertex(label);
}

void insertEdge(Graph &g){

    std::string label1, label2, number;
    std::cout << "Enter the label of the first vertex: ";
    std::cin >> label1;

    //Validate input for label 1
    while(g.findVertex(label1)==nullptr){
        std::cout << std::endl;
        std::cout << "Invalid label, try again." << std::endl;

        std::cout << "Enter the label of the first vertex: ";
        std::cin >> label1;
    } 

    std::cout << std::endl;
    std::cout << "Enter the label of the second vertex: ";
    std::cin >> label2;

    //Validate input for label 2
    while(g.findVertex(label2)==nullptr){
        std::cout << std::endl;
        std::cout << "Invalid label, try again." << std::endl;

        std::cout << "Enter the label of the first vertex: ";
        std::cin >> label2;
    } 

    std::cout << std::endl;
    std::cout << "Enter a positive distance: ";
    std::cin >> number;

    //Validate input for edge weight
    while(isPosNumber(number) == 0){
        std::cout << std::endl;
        std::cout << "Invalid input, try again." << std::endl;

        std::cout << "Enter a positive distance: ";
        std::cin >> number;
    }


    g.addEdge(label1, label2, stoi(number));    
}

void deleteVertex(Graph &g){

    //if graph is empty
    if(g.isEmpty()){
        std::cout << "Graph empty" << std::endl;
        return;
    }

    std::string label;
    std::cout << "Enter the label for the Vertex to be removed: ";
    std::cin >> label;

    //Validate input for vertex label
    while(g.findVertex(label)==nullptr){
        std::cout << std::endl;
        std::cout << "Vertex with given label does not exists, try again" << std::endl;

        std::cout << "Enter the label for the Vertex to be removed: ";
        std::cin >> label;
    } 
    g.removeVertex(label);
}

void deleteEdge(Graph &g){

    //if graph is empty
    if(g.isEmpty()){
        std::cout << "Graph empty" << std::endl;
        return;
    }

    std::string label1;
    std::string label2; 

    std::cout << "Enter the label of the first vertex: ";
    std::cin >> label1;

    //Validate input for label 1
    while(g.findVertex(label1)==nullptr){
        std::cout << std::endl;
        std::cout << "Invalid label, try again." << std::endl;

        std::cout << "Enter the label of the first vertex: ";
        std::cin >> label1;
    } 

    std::cout << std::endl;
    std::cout << "Enter the label of the second vertex: ";
    std::cin >> label2;

    //Validate input for label 2
    while(g.findVertex(label2)==nullptr){
        std::cout << std::endl;

        std::cout << "Invalid label, try again." << std::endl;
        std::cout << "Here is a display of the current graph";
        g.print();
        std::cout << std::endl;

        std::cout << "Enter the label of the first vertex: ";
        std::cin >> label2;
    }

    g.removeEdge(label1, label2);

}

void shortest(Graph &g){

    //if graph is empty
    if(g.isEmpty()){
        std::cout << "Graph empty" << std::endl;
        return;
    }

    std::string startLabel, endLabel;
    std::vector<std::string> path;

    std::cout << "Enter the label of the starting vertex: ";
    std::cin >> startLabel;

    //Validate input for label 1
    while(g.findVertex(startLabel)==nullptr){
        std::cout << std::endl;
        std::cout << "Invalid label, try again." << std::endl;

        std::cout << "Enter the label of the starting vertex: ";
        std::cin >> startLabel;
    } 

    std::cout << std::endl;
    std::cout << "Enter the label of the second vertex: ";
    std::cin >> endLabel;

    //Validate input for label 2
    while(g.findVertex(endLabel)==nullptr){
        std::cout << std::endl;
        std::cout << "Invalid label, try again." << std::endl;

        std::cout << "Enter the label of the ending vertex: ";
        std::cin >> endLabel;
    }

    std::cout << std::endl;
    std::cout << "The shortest distance between " << startLabel << " and " << endLabel << " is ";
    std::cout << g.shortestPath(startLabel, endLabel, path) << std::endl;
    
    std::cout << std::endl;
    std::cout << "The path traveled was ";
    for(auto &elem : path){
        std::cout << elem << "  "; 
    }

    std::cout << std::endl;
}

bool isPosNumber(std::string str){
    for (auto &elem : str){
        if(isdigit(elem)==0 || elem=='-'){
            return false;
        }
    }
    return true;
}