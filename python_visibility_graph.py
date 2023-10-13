from os import path
import sys
from pathlib import Path

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../../")

import matplotlib.pyplot as plt
from lib.algorithm.dijkstra import Dijkstra
from shapely.geometry import Polygon, LineString, Point
import math



class VisibilityGraph:
    def __init__(self, expansion = 0.3):            #default 0.3
        self.obstacles = [  [(10, 1), (10, 3), (13, 3), (13, 1)],          
                            [(5, 1), (5, 3), (7, 3), (7, 1)],                                     
                            [(6, 6), (6, 8), (7, 6)]]              
                         
        
        self.djikstra = Dijkstra()
        self.graph = []
        self.graph_weight = []
        self.expansion = expansion
        self.obstacles_expanded = self.create_obstacles(self.obstacles)
        self.vertices_obstacle = self.extract_vertices()
        self.coord_obstacle = []
        for obstacle in self.obstacles_expanded:
                self.coord_obstacle.append(obstacle.exterior.coords)

    

    def get_obstacles(self):
        return self.obstacles

    def get_path(self):
        return self.path    
        

    def delete_vertices(self):    #Delete vertex added (start and end) to restart algorithm with new start and end (or continue by end to next point)
        self.vertices_obstacle = self.vertices_obstacle[:-2] + [self.vertices_obstacle[-1]]  

    def add_vertex(self,vertex): #Add vertex (start and end)
        self.vertices_obstacle.append(vertex) 
        

    def calculate_distance(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


    def point_inside_obstacle(self, point):
        point = Point(point)

        for obstacle in self.obstacles_expanded:
            if point.within(obstacle):
                return True  
        return False



    def create_weighted_graph(self, edges):
        graph = {}

        for edge in edges:
            node1, node2 = edge
            weight = self.calculate_distance(node1, node2)

            if node1 not in graph:
                graph[node1] = []
            if node2 not in graph:
                graph[node2] = []

            graph[node1].append((node2, weight))
            graph[node2].append((node1, weight))

        return graph

    def can_see(self, p1, p2):
        for obstacle in self.obstacles_expanded:
            if p1 in obstacle.exterior.coords and p2 in obstacle.exterior.coords:
                return False

        line = LineString([p1, p2])
        for obstacle in self.obstacles_expanded:
            if line.crosses(obstacle):
                return False

        for vertex in self.vertices_obstacle:
            if vertex != p1 and vertex != p2:
                line = LineString([p1, p2])
                point = Point(vertex)
                if line.contains(point):
                    return False

        inside_count_p1 = 0
        for obstacle in self.obstacles_expanded:
            if obstacle.intersects(Point(p1)):
                inside_count_p1 += 1
        if inside_count_p1 >= 2:
            return False

        inside_count_p2 = 0
        for obstacle in self.obstacles_expanded:
            if obstacle.intersects(Point(p2)):
                inside_count_p2 += 1
        if inside_count_p2 >= 2:
            return False

        return True

    def create_obstacles(self,vertice):
        obstacles = [Polygon(vertices) for vertices in vertice]
        all_obstacles = []
        for v in obstacles:
            all_obstacle = Polygon(v)
            expanded_obstacle = all_obstacle.buffer(self.expansion, join_style='mitre')
            all_obstacles.append(expanded_obstacle)
        return all_obstacles

    def extract_vertices(self):
        vertices = []
        for obstacle in self.obstacles_expanded:
            vertices.extend(list(obstacle.exterior.coords))
        return vertices

    def visibility_graph(self):
        graph = []
        added_pairs = set()

        for vertex1 in self.vertices_obstacle:
            for vertex2 in self.vertices_obstacle:
                if vertex1 != vertex2 and self.can_see(vertex1, vertex2):
                    pair = tuple(sorted([vertex1, vertex2]))

                    if pair not in added_pairs:
                        graph.append(pair)
                        added_pairs.add(pair)
        return graph


    def plot_obstacles(self,start, end):
        plt.figure(figsize=(8, 8))

        for obstacle in self.coord_obstacle:
            x = []
            y = []

            for point in obstacle:
                x.append(point[0])

                y.append(point[1])

            x.append(obstacle[0][0])
            y.append(obstacle[0][1])

            plt.plot(x, y, 'k-')

        for vertex1, vertex2 in self.graph:
            plt.plot([vertex1[0], vertex2[0]], [vertex1[1], vertex2[1]], 'b')

        plt.plot(start[0], start[1], 'ro', label='Arbitrary Point')
        plt.xlabel('X')
        plt.ylabel('Y')

        plt.plot(end[0], end[1], 'ro', label='Arbitrary Point')
        plt.xlabel('X')
        plt.ylabel('Y')


        #Show
        plt.gca().set_aspect('equal', adjustable='box')
        plt.grid(True)
        plt.show()


    def do_dijkstra(self, start, end):         
        self.add_vertex(start) 
        self.add_vertex(end) 
        self.graph = self.visibility_graph()
        self.graph_weight = self.create_weighted_graph(self.graph)
        
        return self.djikstra.find_shortest_path(self.graph_weight, start, end)  


    def get_weighted_graph(self):                   
        return self.create_weighted_graph(self.visibility_graph())    


    

if __name__ == '__main__':  
    vis_graph = VisibilityGraph()
    start = (0.0,0.0)
    end = (10, 10)
    shortest_paths_start_end = vis_graph.do_dijkstra(start, end)
    print(shortest_paths_start_end)
    vis_graph.plot_obstacles(start, end)




