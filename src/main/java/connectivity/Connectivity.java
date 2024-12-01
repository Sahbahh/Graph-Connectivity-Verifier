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

        /*    if (graphInput.isDirected) {
                // TODO: remove once directed it implmented
                System.err.println("Only undirected graphs at this step. Liza XD");
                System.exit(1);
            }*/

            // Print parsed data (for debugging purposes)
            System.out.println("Graph Type: " + (graphInput.isDirected ? "Directed" : "Undirected"));
            System.out.println("Number of Nodes: " + graphInput.numNodes);
            System.out.println("Edges:");
            for (int[] edge : graphInput.edges) {
                System.out.println(edge[0] + " -- " + edge[1]);
            }

            // Check connectivity
            boolean isConnected;
            if (graphInput.isDirected) {
                // Check strong connectivity for directed graphs
                isConnected = checkConnectivityDirected(graphInput);
                System.out.println("The directed graph is " + (isConnected ? "strongly connected." : "NOT strongly connected."));
                
                // Print SCCs ONLY if the graph is strongly connected
              //  if (isConnected) {
                    List<List<Integer>> sccs = tarjansSCC(graphInput);
                    System.out.println("Strongly connected components:");
                    for (List<Integer> scc : sccs) {
                        System.out.println(scc);
                    }
                //}
            } else {
                // Check connectivity for undirected graphs
                isConnected = checkConnectivityUndirected(graphInput);
                System.out.println("The undirected graph is " + (isConnected ? "connected." : "NOT connected."));
            }
        }catch (IOException e) {
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
    private static boolean checkConnectivityUndirected(GraphInput graphInput) {
        // Create a Z3 context for managing logical constraints.
        Context ctx = new Context();
        try {
            int numNodes = graphInput.numNodes; // Total number of nodes in the graph
            List<int[]> edges = graphInput.edges; // List of edges in the graph
    
            // Step 1: Build adjacency list for DFS traversal.
            Map<Integer, List<Integer>> adjacencyList = new HashMap<>();
            for (int i = 0; i <= numNodes; i++) {
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
     * TODO: Liza
     * Checks the strong connectivity of a directed graph
     *
     * @param graphInput The input graph, containing the number of nodes and a list of directed edges.
     * @return True if the graph is strongly connected; False otherwise.
     */
    private static boolean checkConnectivityDirected(GraphInput graphInput) {
        // Step 1: Create a Z3 context to manage logical constraints.
        Context ctx = new Context();

        try {
            int numNodes = graphInput.numNodes; // Total number of nodes in the graph
            List<int[]> edges = graphInput.edges; // List of directed edges in the graph

            // Step 2: Build adjacency lists for forward and reverse DFS traversals.
            Map<Integer, List<Integer>> adjacencyList = new HashMap<>();
            Map<Integer, List<Integer>> reverseAdjacencyList = new HashMap<>();

            // Initialize the adjacency lists for all nodes from 0 to numNodes - 1.
            for (int i = 0; i <= numNodes; i++) {
                adjacencyList.put(i, new ArrayList<>());
                reverseAdjacencyList.put(i, new ArrayList<>());
            }

            // Populate adjacency lists with the edges from the input graph.
            for (int[] edge : edges) {
                adjacencyList.get(edge[0]).add(edge[1]); // Forward edge
                reverseAdjacencyList.get(edge[1]).add(edge[0]); // Reverse edge
            }

            // Step 3: Perform a forward DFS from node 0 to check reachability.
            Set<Integer> reachable = new HashSet<>();
            dfs(1, adjacencyList, reachable); // Start DFS from node 0

            // If not all nodes are reachable, the graph is not strongly connected.
            if (reachable.size() != numNodes) {
                return false;
            }

            // Step 4: Perform a reverse DFS from node 0 to check reverse reachability.
            Set<Integer> reverseReachable = new HashSet<>();
            dfs(1, reverseAdjacencyList, reverseReachable);

            // If not all nodes are reachable in the reverse graph, the graph is not strongly connected.
            if (reverseReachable.size() != numNodes) {
                return false;
            }

            // The graph is strongly connected.
            return true;

        } finally {
            ctx.close(); // Close the Z3 context to release resources.
        }
    }


    
    private static List<List<Integer>> tarjansSCC(GraphInput graphInput) {
        int numNodes = graphInput.numNodes;

        // Create an adjacency list representation of the graph.
        List<List<Integer>> graph = new ArrayList<>();
        for (int i = 0; i <= numNodes; i++) { // Include 0 to numNodes for 1-based indexing
            graph.add(new ArrayList<>());
        }

        // Populate the adjacency list with directed edges from the input.
        for (int[] edge : graphInput.edges) {
            graph.get(edge[0]).add(edge[1]); // Add a directed edge from edge[0] to edge[1].
        }

        // List to store all strongly connected components (SCCs).
        List<List<Integer>> sccComponents = new ArrayList<>();

        // Arrays to keep track of discovery IDs and low-link values for each node.
        int[] ids = new int[numNodes + 1]; // Discovery IDs for each node (1-based).
        int[] low = new int[numNodes + 1]; // Lowest discovery ID reachable from each node (1-based).
        boolean[] onStack = new boolean[numNodes + 1]; // Track whether nodes are in the stack.

        // Stack to store the nodes currently being processed in the DFS.
        Stack<Integer> stack = new Stack<>();

        // Mutable variable to assign unique discovery IDs to each node.
        int[] id = {0}; // Starts from 0.

        // Initialize all nodes as unvisited by setting their discovery IDs to -1.
        Arrays.fill(ids, -1);

        // Perform a depth-first search (DFS) on all unvisited nodes.
        for (int i = 1; i <= numNodes; i++) { // Start from 1 and include all nodes
            if (ids[i] == -1) { // If the node has not been visited.
                dfsTarjan(i, ids, low, onStack, stack, graph, sccComponents, id);
            }
        }

        // Return the list of strongly connected components.
        return sccComponents;
    }



    private static void dfsTarjan(int at, int[] ids, int[] low, boolean[] onStack, Stack<Integer> stack,
                                  List<List<Integer>> graph, List<List<Integer>> sccComponents, int[] id) {
    	   // Assign the discovery ID and low-link value to the current node.
        ids[at] = low[at] = id[0]++;
        stack.push(at); // Push the current node onto the stack.
        onStack[at] = true; // Mark the current node as being on the stack.

        // Explore all neighbors of the current node.
        for (int to : graph.get(at)) {
            if (ids[to] == -1) {
                // If the neighbor hasn't been visited, recursively perform DFS on it.
                dfsTarjan(to, ids, low, onStack, stack, graph, sccComponents, id);

                // Update the low-link value of the current node based on the neighbor's low-link value.
                low[at] = Math.min(low[at], low[to]);
            } else if (onStack[to]) {
                // If the neighbor is already on the stack, update the low-link value based on its discovery ID.
                low[at] = Math.min(low[at], ids[to]);
            }
        }

        // If the current node is a root node (discovery ID equals low-link value),
        // it means we have found a strongly connected component (SCC).
        if (ids[at] == low[at]) {
            List<Integer> component = new ArrayList<>();

            // Pop all nodes from the stack that belong to this SCC.
            while (!stack.isEmpty()) {
                int node = stack.pop();
                onStack[node] = false; // Mark the node as no longer being on the stack.
                component.add(node); // Add the node to the SCC.

                // Stop when we reach the root node of the SCC.
                if (node == at) break;
            }

            // Add the completed SCC to the list of SCCs.
            sccComponents.add(component);
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
