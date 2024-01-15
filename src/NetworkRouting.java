import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.*;

public class NetworkRouting {

    private static final int INFINITY = Integer.MAX_VALUE;

    // Class to represent edges in the graph
    static class Edge {
        int destination;
        int bandwidth;
        int distance;
        double reliability;

        public Edge(int destination, int bandwidth, int distance, double reliability) {
            this.destination = destination;
            this.bandwidth = bandwidth;
            this.distance = distance;
            this.reliability = reliability;
        }
    }

    // Class to represent vertices in the graph
    static class Vertex {
        List<Edge> edges;

        public Vertex() {
            this.edges = new ArrayList<>();
        }
    }

    // Class to represent the graph
    static class Graph {
        Map<Integer, Vertex> vertices;

        public Graph() {
            this.vertices = new HashMap<>();
        }

        public void addEdge(int source, Edge edge) {
            vertices.computeIfAbsent(source, k -> new Vertex()).edges.add(edge);
        }
    }

    // Read input file and create a graph
    public static Graph readInput(String fileName) throws IOException {
        Graph graph = new Graph();

        try (BufferedReader br = new BufferedReader(new FileReader(fileName))) {
            String line;
            int matrixIndex = 0;
            int source = 1; // Start from node 1

            while ((line = br.readLine()) != null) {
                if (line.isEmpty()) {
                    matrixIndex++;
                    continue;
                }

                String[] values = line.split(":");
                int[] matrix = Arrays.stream(values[1].trim().split("\\s+"))
                        .mapToInt(Integer::parseInt)
                        .toArray();

                for (int i = 0; i < matrix.length; i++) {
                    int destination = i + 1;
                    int bandwidth = (matrixIndex == 0) ? 3 + (int) (Math.random() * 8) : matrix[i];
                    int distance = (matrixIndex == 1) ? 1 + (int) (Math.random() * 4) : matrix[i];
                    double reliability = (matrixIndex == 3) ? 0.95 + (Math.random() * 0.04) : matrix[i] / 100.0;

                    graph.addEdge(source, new Edge(destination, bandwidth, distance, reliability));
                }

                source++;
            }
        }

        return graph;
    }

    // Dijkstra's algorithm to find the shortest path
    public static List<Integer> dijkstra(Graph graph, int source, int destination, int bandwidthThreshold, int delayThreshold, double reliabilityThreshold) {
        Map<Integer, Integer> distance = new HashMap<>();
        Map<Integer, Integer> bandwidth = new HashMap<>();
        Map<Integer, Double> reliability = new HashMap<>();
        Map<Integer, Integer> parent = new HashMap<>();

        PriorityQueue<Integer> pq = new PriorityQueue<>(Comparator.comparingInt(distance::get));
        Set<Integer> visited = new HashSet<>();

        for (int vertex : graph.vertices.keySet()) {
            distance.put(vertex, INFINITY);
            bandwidth.put(vertex, 0);
            reliability.put(vertex, 0.0);
            parent.put(vertex, null);
        }

        distance.put(source, 0);
        bandwidth.put(source, Integer.MAX_VALUE);
        reliability.put(source, 1.0);

        pq.add(source);

        while (!pq.isEmpty()) {
            int current = pq.poll();

            if (visited.contains(current)) continue;
            visited.add(current);

            for (Edge edge : graph.vertices.get(current).edges) {
                int neighbor = edge.destination;
                int newDistance = distance.get(current) + edge.distance;
                int newBandwidth = Math.min(bandwidth.get(current), edge.bandwidth);
                double newReliability = reliability.get(current) * edge.reliability;

                if (newDistance < distance.get(neighbor) && newBandwidth >= bandwidthThreshold && newDistance <= delayThreshold && newReliability >= reliabilityThreshold) {
                    distance.put(neighbor, newDistance);
                    bandwidth.put(neighbor, newBandwidth);
                    reliability.put(neighbor, newReliability);
                    parent.put(neighbor, current);
                    pq.add(neighbor);
                }
            }
        }

        return buildPath(parent, destination);
    }

    // Helper method to build the path from parent map
    private static List<Integer> buildPath(Map<Integer, Integer> parent, int destination) {
        List<Integer> path = new ArrayList<>();
        while (destination != 0 && parent.containsKey(destination)) {
            path.add(destination);
            destination = parent.get(destination);
        }
        Collections.reverse(path);
        return path;
    }
    

    // Objective function
    public static double objectiveFunction(int bandwidth, int distance) {
        return bandwidth * distance;
    }

    public static void main(String[] args) {
        try {
            String fileName = "your_input_file.txt";
            Graph graph = readInput(fileName);

            int sourceNode = 1;
            int destinationNode = 10;
            int bandwidthRequirement = 5;
            int delayThreshold = 40;
            double reliabilityThreshold = 0.7;

            // Example using Dijkstra's algorithm
            List<Integer> dijkstraPath = dijkstra(graph, sourceNode, destinationNode, bandwidthRequirement, delayThreshold, reliabilityThreshold);

            // Print the result
            System.out.println("Shortest path using Dijkstra's algorithm: " + dijkstraPath);

            // You can implement similar methods for Bellman-Ford, A*, and meta-heuristic algorithms
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
