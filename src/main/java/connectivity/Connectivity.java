package connectivity;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.*;

public class Connectivity {
    public static void main(String[] args) {
        // Ensure the command line argument for input.txt file is provided
        if (args.length != 1) {
            System.err.println("Usage: java -cp lib/com.microsoft.z3.jar:target/classes connectivity.Connectivity <input_file>");
            System.exit(1);
        }

        String inputFilePath = args[0];

        try {
            // Parse the input file
            GraphInput graphInput = parseInputFile(inputFilePath);

            if (graphInput.isDirected) {
                System.err.println("Only undirected graphs are supported at this step.");
                System.exit(1);
            }

            // Print parsed data
            // todo: remove later
            System.out.println("Graph Type: " + (graphInput.isDirected ? "Directed" : "Undirected"));
            System.out.println("Number of Nodes: " + graphInput.nodeCount);
            System.out.println("Edges:");
            for (int[] edge : graphInput.edges) {
                System.out.println(edge[0] + " -- " + edge[1]);
            }

            // Check if the graph is connected
            boolean isConnected = isGraphConnected(graphInput);
            System.out.println("The graph is " + (isConnected ? "connected." : "not connected."));
        } catch (IOException e) {
            System.err.println("Error reading input file: " + e.getMessage());
            System.exit(1);
        }
    }

    /**
     * Parses the input file and returns a GraphInput object.
     *
     * @param filePath the path to the input file
     * @return a GraphInput object containing the graph data
     * @throws IOException if an error occurs while reading the file
     */
    private static GraphInput parseInputFile(String filePath) throws IOException {
        try (BufferedReader reader = new BufferedReader(new FileReader(filePath))) {
            // Read the first line to determine if the graph is directed or undirected
            String graphType = reader.readLine().trim();
            boolean isDirected = graphType.equalsIgnoreCase("D");

            // Read the number of nodes
            int nodeCount = Integer.parseInt(reader.readLine().trim());

            // Read the edges
            List<int[]> edges = new ArrayList<>();
            String line;
            while ((line = reader.readLine()) != null) {
                String[] parts = line.trim().split("\\s+");
                if (parts.length != 2) {
                    throw new IllegalArgumentException("Invalid edge format: " + line);
                }
                int from = Integer.parseInt(parts[0]);
                int to = Integer.parseInt(parts[1]);
                edges.add(new int[]{from, to});
            }

            return new GraphInput(isDirected, nodeCount, edges);
        }
    }

    /**
     * Checks if the given undirected graph is connected.
     *
     * @param graphInput the graph input
     * @return true if the graph is connected, false otherwise
     */
    private static boolean isGraphConnected(GraphInput graphInput) {
        // Build an adjacency list representation of the graph
        Map<Integer, List<Integer>> adjacencyList = new HashMap<>();
        for (int i = 1; i <= graphInput.nodeCount; i++) {
            adjacencyList.put(i, new ArrayList<>());
        }
        for (int[] edge : graphInput.edges) {
            adjacencyList.get(edge[0]).add(edge[1]);
            adjacencyList.get(edge[1]).add(edge[0]); // Undirected graph
        }

        // Perform a DFS to check connectivity
        Set<Integer> visited = new HashSet<>();
        dfs(1, adjacencyList, visited);

        // Check if all nodes were visited
        return visited.size() == graphInput.nodeCount;
    }

    /**
     * Depth-First Search to traverse the graph.
     *
     * @param node           the current node
     * @param adjacencyList  the graph's adjacency list
     * @param visited        the set of visited nodes
     */
    private static void dfs(int node, Map<Integer, List<Integer>> adjacencyList, Set<Integer> visited) {
        if (visited.contains(node)) return;
        visited.add(node);
        for (int neighbor : adjacencyList.get(node)) {
            dfs(neighbor, adjacencyList, visited);
        }
    }

    /**
     * A class to hold the parsed graph input.
     */
    private static class GraphInput {
        boolean isDirected;
        int nodeCount;
        List<int[]> edges;

        public GraphInput(boolean isDirected, int nodeCount, List<int[]> edges) {
            this.isDirected = isDirected;
            this.nodeCount = nodeCount;
            this.edges = edges;
        }
    }
}
