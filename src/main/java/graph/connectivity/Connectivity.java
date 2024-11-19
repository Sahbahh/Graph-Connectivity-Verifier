package graph.connectivity;

import com.microsoft.z3.*;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.*;

public class Connectivity {

    // -------------------------------- Step 1: Parse Input --------------------------------
    private String graphType;
    private int numNodes;
    private List<int[]> edges;

    // -------------------------------- Step 2: Z3 Variables --------------------------------
    private Context ctx; // Z3 Context
    private Solver solver; // Z3 Solver
    private BoolExpr[] visited; // Z3 Boolean variables for visited nodes

    public static void main(String[] args) {
        System.out.println("Step 1: Parsing Input...");
        Connectivity connectivity = new Connectivity();

        if (args.length < 1) {
            System.err.println("Usage: java -cp <jar-file> graph.connectivity.Connectivity <input.txt>");
            return;
        }

        String inputPath = args[0];

        try {
            connectivity.parseInput(inputPath);
            System.out.println("Input parsed successfully!");

            // Step 2: Model Variables in Z3
            System.out.println("Step 2: Modeling Variables in Z3...");
            connectivity.initializeZ3();
            System.out.println("Z3 Variables initialized successfully!");
        } catch (Exception e) {
            System.err.println("Error: " + e.getMessage());
        }
    }

    public void parseInput(String inputPath) throws FileNotFoundException {
        try (Scanner scanner = new Scanner(new File(inputPath))) {
            // Read graph type
            if (scanner.hasNextLine()) {
                graphType = scanner.nextLine().trim();
                System.out.println("Graph Type: " + graphType);
                if (!graphType.equals("U") && !graphType.equals("D")) {
                    throw new IllegalArgumentException("Invalid graph type. Use 'U' for undirected or 'D' for directed.");
                }
            } else {
                throw new IllegalArgumentException("Missing graph type.");
            }

            // Read number of nodes
            if (scanner.hasNextLine()) {
                numNodes = Integer.parseInt(scanner.nextLine().trim());
                System.out.println("Number of Nodes: " + numNodes);
                if (numNodes <= 0) {
                    throw new IllegalArgumentException("Number of nodes must be positive.");
                }
            } else {
                throw new IllegalArgumentException("Missing number of nodes.");
            }

            // Read edges
            edges = new ArrayList<>();
            System.out.println("Edges:");
            while (scanner.hasNextLine()) {
                String[] edgeData = scanner.nextLine().trim().split("\\s+");
                if (edgeData.length != 2) {
                    throw new IllegalArgumentException("Invalid edge format.");
                }
                int u = Integer.parseInt(edgeData[0]);
                int v = Integer.parseInt(edgeData[1]);
                edges.add(new int[]{u, v});
                System.out.println(u + " -- " + v);
            }
        }
    }

    // -------------------------------- Step 2: Initialize Z3 Variables --------------------------------
    public void initializeZ3() {
        ctx = new Context(); // Initialize Z3 context
        solver = ctx.mkSolver(); // Initialize Z3 solver

        // Create Boolean variables for each node to track connectivity
        visited = new BoolExpr[numNodes + 1]; // +1 because nodes are 1-based indexed
        for (int i = 1; i <= numNodes; i++) {
            visited[i] = (BoolExpr) ctx.mkConst("visited_" + i, ctx.getBoolSort());
            System.out.println("Created Z3 variable: visited_" + i);
        }
    }
}


    // -------------------------------- Step 3: Add Start Node Constraint --------------------------------
    // -------------------------------- Step 4a: Define Connectivity Constraints For Undirected --------------------------------
    // -------------------------------- Step 4b: Define Connectivity Constraints for Directed --------------------------------
    // -------------------------------- Step 5: Solve the Z3 Model --------------------------------
    // -------------------------------- Step 6: Output the Result --------------------------------

