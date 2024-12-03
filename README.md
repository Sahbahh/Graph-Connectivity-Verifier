# CMPT 477 Project: Graph Connectivity
**By:** Sahba Hajihoseini, Layan Barrieshee, Liza Awwad  
**Fall 2024**  

## Project Overview
This project implements graph connectivity verification for both undirected and directed graphs using Z3 constraints. Graph connectivity is determined based on input provided through text files, specifying the graph type (undirected or directed), the number of nodes, and the edges between them. For undirected graphs, connectivity ensures that every node is reachable from every other node. For directed graphs, strong connectivity is verified by ensuring that every node is reachable from every other node in both directions.

### Algorithms and Implementation
- **Undirected Graphs:** Depth First Search (DFS) is used to verify that all nodes are reachable. Z3 constraints are applied to confirm mutual reachability and transitive closure.
- **Directed Graphs:** Strong connectivity is verified through a combination of DFS on the original and reversed graphs, alongside Tarjan's algorithm to identify strongly connected components (SCCs).

### Key Features
- A `GraphInput` class stores the graph type, nodes, and edges.
- A `Connectivity` class processes the input, applies algorithms, and verifies graph properties.
- Test cases were created by all team members to validate functionality.

For a detailed explanation of the code and algorithms, refer to the full [GraphConnectivityReport.pdf](GraphConnectivityReport.pdf).

## Running the code
```bash
mvn clean package
mvn package
java -cp lib/com.microsoft.z3.jar:target/classes connectivity.Connectivity input.txt

