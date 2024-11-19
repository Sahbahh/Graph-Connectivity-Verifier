package connectivity;

import com.microsoft.z3.*;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.*;

public class Connectivity {

    public static void main(String[] args) {
        if (args.length != 1) {
            System.err.println("Usage: java -cp lib/com.microsoft.z3.jar:target/classes connectivity.Connectivity <input_file>");
            System.exit(1);
        }

        String inputFilePath = args[0];

        try {
            // Parse the input file
            GraphInput graphInput = parseInputFile(inputFilePath);

            if (graphInput.isDirected) {
                // TODO: remove once directed it implmented
                System.err.println("Only undirected graphs at this step. Liza XD");
                System.exit(1);
            }

            // Print parsed data (for debugging purposes)
            System.out.println("Graph Type: " + (graphInput.isDirected ? "Directed" : "Undirected"));
            System.out.println("Number of Nodes: " + graphInput.numNodes);
            System.out.println("Edges:");
            for (int[] edge : graphInput.edges) {
                System.out.println(edge[0] + " -- " + edge[1]);
            }

            // Check connectivity
            boolean isConnected = checkConnectivityWithDFSAndZ3(graphInput);

            System.out.println("The graph is " + (isConnected ? "connected." : "NOT connected."));
        } catch (IOException e) {
            System.err.println("Error reading input file: " + e.getMessage());
            System.exit(1);
        }
    }

    /**
     * Parses the input file to extract graph details (directed or undirected),
     * the number of nodes, and the list of edges.
     *
     * @param filePath The path to the input file containing graph data.
     * @return A GraphInput object containing the graph type, number of nodes, and edges.
     * @throws IOException If an error occurs while reading the file.
     */
    private static GraphInput parseInputFile(String filePath) throws IOException {
        try (BufferedReader br = new BufferedReader(new FileReader(filePath))) {

            // Read the first line to determine the graph type ("D" for directed, "U" for undirected)
            String graphType = br.readLine().trim();
            boolean isDirected = graphType.equals("D");

            // Read the second line to determine the number of nodes in the graph
            int numNodes = Integer.parseInt(br.readLine().trim());

            // Initialize a list to store edges (ach edge is represented as an int array [u, v])
            List<int[]> edges = new ArrayList<>();

            // parse edges
            String line;
            while ((line = br.readLine()) != null) {
                // Split each line into node u and node v
                String[] parts = line.trim().split("\\s+");

                int u = Integer.parseInt(parts[0]);
                int v = Integer.parseInt(parts[1]);

                // Add the edge as an array to the edges list.
                edges.add(new int[]{u, v});
            }

            return new GraphInput(isDirected, numNodes, edges);
        }
    }


    /**
     * Checks the connectivity of an undirected graph using DFS and Z3 constraints.
     *
     * @param graphInput The input graph, containing the number of nodes and a list of edges.
     * @return True if the graph is connected; False otherwise.
     */
    private static boolean checkConnectivityWithDFSAndZ3(GraphInput graphInput) {
        // Create a Z3 context for managing logical constraints.
        Context ctx = new Context();
        try {
            int numNodes = graphInput.numNodes; // Total number of nodes in the graph
            List<int[]> edges = graphInput.edges; // List of edges in the graph
    
            // Step 1: Build adjacency list for DFS traversal.
            Map<Integer, List<Integer>> adjacencyList = new HashMap<>();
            for (int i = 1; i <= numNodes; i++) {
                // Initialize adjacency list for each node
                adjacencyList.put(i, new ArrayList<>());
            }
            for (int[] edge : edges) {
                adjacencyList.get(edge[0]).add(edge[1]); // Add the edge in one direction
                adjacencyList.get(edge[1]).add(edge[0]); // Add the edge in the other direction (undirected graph)
            }
    
            // Step 2: Perform DFS to check if all nodes are reachable from node 1.
            Set<Integer> reachable = new HashSet<>();
            // Start DFS from node 1
            dfs(1, adjacencyList, reachable);
    
            // If the size of the reachable set is less than the total number of nodes, the graph is disconnected.
            if (reachable.size() != numNodes) {
                return false;
            }
    
            // Step 3: Create a Z3 solver to enforce constraints
            Solver solver = ctx.mkSolver();
    
            // Step 4: Create Z3 variables to represent edges 
            // edgeVars[i][j] represents whether there is a path from node i to node j
            BoolExpr[][] edgeVars = new BoolExpr[numNodes + 1][numNodes + 1];
            for (int i = 1; i <= numNodes; i++) {
                for (int j = 1; j <= numNodes; j++) {
                    if (i != j) {
                        edgeVars[i][j] = ctx.mkBoolConst("edge_" + i + "_" + j);
                    }
                }
            }
    
            // Step 5: Add Z3 constraints for the edges that exist in the input graph
            for (int[] edge : edges) {
                int u = edge[0]; // Starting node of the edge.
                int v = edge[1]; // Ending node of the edge.
                
                // Add constraints indicating that u and v are connected directly (both directions since it's undirected)
                solver.add(ctx.mkOr(edgeVars[u][v], edgeVars[v][u]));
            }
    
            // Step 6: Add transitive closure constraints to ensure paths exist indirectly between nodes.
            for (int i = 1; i <= numNodes; i++) {
                for (int j = 1; j <= numNodes; j++) {
                    if (i != j) {
                        // Start with no paths
                        BoolExpr reachableThroughOthers = ctx.mkFalse();
                        for (int k = 1; k <= numNodes; k++) {
                            if (k != i && k != j) {
                                // If there is a path from i to k and k to j, then i can reach j
                                reachableThroughOthers = ctx.mkOr(reachableThroughOthers,
                                        ctx.mkAnd(edgeVars[i][k], edgeVars[k][j]));
                            }
                        }

                        // Enforce that if i and j are connected, it must be through a direct or transitive path
                        solver.add(ctx.mkImplies(edgeVars[i][j], reachableThroughOthers));
                    }
                }
            }
    
            // Step 7: Add a constraint to ensure all nodes are mutually reachable.
            BoolExpr allConnected = ctx.mkTrue();
            for (int i = 1; i <= numNodes; i++) {
                for (int j = i + 1; j <= numNodes; j++) {
                    // Enforce mutual reachability between nodes i and j
                    allConnected = ctx.mkAnd(allConnected, edgeVars[i][j], edgeVars[j][i]);
                }
            }
            solver.add(allConnected);
    
            // Step 8: Use the solver to check satisfiability of the constraints.
            return solver.check() == Status.SATISFIABLE;
    
        } finally {
            // Close the Z3 context
            ctx.close();
        }
    }


    /**
     * Performs a Depth-First Search (DFS) traversal to find all reachable nodes
     * from a given starting node in an undirected graph.
     *
     * @param node           The current node being visited.
     * @param adjacencyList  A map representing the adjacency list of the graph, where
     *                       each key is a node, and the value is a list of its neighbors.
     * @param visited        A set to keep track of all nodes that have already been visited.
     */
    private static void dfs(int node, Map<Integer, List<Integer>> adjacencyList, Set<Integer> visited) {
        if (visited.contains(node)) {
            return;
        }

        // Mark the current node as visited by adding it to the visited set
        visited.add(node);

        // Recursively visit all unvisited neighbors of the current node
        for (int neighbor : adjacencyList.get(node)) {
            dfs(neighbor, adjacencyList, visited);
        }
    }

    
    static class GraphInput {
        boolean isDirected;
        int numNodes;
        List<int[]> edges;

        GraphInput(boolean isDirected, int numNodes, List<int[]> edges) {
            this.isDirected = isDirected;
            this.numNodes = numNodes;
            this.edges = edges;
        }
    }
}
