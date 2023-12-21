#include "Graph.hpp"
#include <iostream>
#include <vector>

int main() {

    Graph g;
    help();
    for(;;){
        char code;
        std::cout << "Enter operation code:" << std::endl;
        std::cin >> code;

        while(std::cin.get() != '\n')
			;

        switch (code) {
			case 'h':
				help();
				break;
			case 'v':
				insertVertex(g);
                break;
			case 'e':
				insertEdge(g);
				break;
			case 'd':
				deleteVertex(g);
                break;
			case 'r':
				deleteEdge(g);
                break;
			case 's':
				shortest(g);
				break;
            case 'l':
				g.print();
                break;
            case 'q':
				return 0;
			default:
				std::cout << ("Illegal operation code!") << std::endl;
		}
		std::cout << std::endl;
    }



    return 0;

}