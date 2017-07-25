#include <iostream>
#include <list>

using namespace std;

class Node {
    private:
        int id;

    public:
        Node() { this->id = 0; }
        Node(int id) { this->id = id; }

        int getNodeId() { return this->id; }
        void setNodeId(int id) { this->id = id; }
};

class Edge {
    private:
        Node u, v;

    public:
        Edge(Node u, Node v) {
            this->u = u;
            this->v = v;
        }

        Node getNodeU() { return u; }
        Node getNodeV() { return v; }
};

class Graph {
    private:
        std::list<Node> nodes;
        std::list<Edge> edges;

    public:
        std::list<Node> getNodes() { return nodes; }
        std::list<Edge> getEdges() { return edges; }

        void addNode(Node newNode) {
            nodes.insert(nodes.end(), newNode);
        }

        void addEdge(Node u, Node v) {
            if (nodes.)
        }
};

int main(int argc, char **argv) {
    Graph graph;
    Node n1(1);
    Node n2(2);

    cout << "Node 1: " << n1.getNodeId() << "\n";
    cout << "Edge 1: " << e.getNodeV().getNodeId() << "\n";
}
