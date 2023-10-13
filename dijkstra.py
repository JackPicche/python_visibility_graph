import heapq

class Dijkstra:
    def find_shortest_path(self, graph, start, end):
        queue = [(0, start, [])]

        distances = {node: float('infinity') for node in graph}
        distances[start] = 0
        

        paths = {node: [] for node in graph}
        
        while queue:
            current_distance, current_node, current_path = heapq.heappop(queue)
            
            if current_distance > distances[current_node]:
                continue
            
            current_path = current_path + [current_node]
            
            if current_node == end:
                return current_path
            
            for neighbor, weight in graph[current_node]:
                distance = current_distance + weight
                
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    heapq.heappush(queue, (distance, neighbor, current_path))
                    paths[neighbor] = current_path
        
        return []

