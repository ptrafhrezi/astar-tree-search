import heapq
from typing import Dict, List, Tuple, Set, Optional

class Node:
    def __init__(self, name: str, heuristic: int):
        self.name = name
        self.heuristic = heuristic
        self.g_cost = float('inf')
        self.f_cost = float('inf')
        self.parent = None
    
    def __lt__(self, other):
        return self.f_cost < other.f_cost
    
    def __repr__(self):
        return f"Node({self.name}, h={self.heuristic})"

class AStarGraph:
    def __init__(self):
        self.nodes: Dict[str, Node] = {}
        self.edges: Dict[str, List[Tuple[str, int]]] = {}
    
    def add_node(self, name: str, heuristic: int):
        self.nodes[name] = Node(name, heuristic)
        self.edges[name] = []
    
    def add_edge(self, from_node: str, to_node: str, cost: int):
        self.edges[from_node].append((to_node, cost))
    
    def a_star_search(self, start: str, goal: str) -> Tuple[Optional[List[str]], Set[str]]:
        start_node = self.nodes[start]
        start_node.g_cost = 0
        start_node.f_cost = start_node.heuristic
        
        open_set = [(start_node.f_cost, start)]
        explored = set()
        
        print(f"Starting A* search from {start} to {goal}\n")
        
        while open_set:
            current_f, current_name = heapq.heappop(open_set)
            current = self.nodes[current_name]
            
            if current_name in explored:
                continue
            
            explored.add(current_name)
            print(f"Exploring: {current_name} (g={current.g_cost}, h={current.heuristic}, f={current.f_cost})")
            
            if current_name == goal:
                print(f"\nGoal reached: {goal}")
                path = self._reconstruct_path(current)
                return path, explored
            
            for neighbor_name, edge_cost in self.edges[current_name]:
                if neighbor_name in explored:
                    continue
                
                neighbor = self.nodes[neighbor_name]
                tentative_g = current.g_cost + edge_cost
                
                if tentative_g < neighbor.g_cost:
                    neighbor.parent = current
                    neighbor.g_cost = tentative_g
                    neighbor.f_cost = neighbor.g_cost + neighbor.heuristic
                    
                    heapq.heappush(open_set, (neighbor.f_cost, neighbor_name))
                    print(f"  -> {neighbor_name}: g={neighbor.g_cost}, h={neighbor.heuristic}, f={neighbor.f_cost}")
        
        print("\nNo path found!")
        return None, explored
    
    def _reconstruct_path(self, node: Node) -> List[str]:
        path = []
        current = node
        while current:
            path.append(current.name)
            current = current.parent
        return list(reversed(path))


def create_example_graph():
    graph = AStarGraph()
    
    graph.add_node('S', 7)
    graph.add_node('A', 9)
    graph.add_node('D', 5)
    graph.add_node('B', 4)
    graph.add_node('C', 2)
    graph.add_node('E', 3)
    graph.add_node('G', 0)
    
    graph.add_edge('S', 'A', 2)
    graph.add_edge('S', 'D', 2)
    graph.add_edge('A', 'B', 1)
    graph.add_edge('A', 'D', 3)
    graph.add_edge('D', 'B', 5)
    graph.add_edge('D', 'E', 3)
    graph.add_edge('B', 'C', 3)
    graph.add_edge('B', 'E', 3)
    graph.add_edge('C', 'G', 4)
    graph.add_edge('E', 'G', 4)
    
    return graph


if __name__ == "__main__":
    graph = create_example_graph()
    path, explored = graph.a_star_search('S', 'G')
    
    print("\n" + "="*50)
    print("RESULTS:")
    print("="*50)
    print(f"Explored nodes: {explored}")
    print(f"Path found: {' -> '.join(path)}")
    print(f"Total cost: {graph.nodes[path[-1]].g_cost}")