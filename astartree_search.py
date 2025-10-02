import heapq
from typing import Dict, List, Tuple, Optional

class TreeNode:
    def __init__(self, name: str, heuristic: int, g_cost: int = 0, parent=None, path: List[str] = None):
        self.name = name
        self.heuristic = heuristic
        self.g_cost = g_cost
        self.f_cost = g_cost + heuristic
        self.parent = parent
        self.path = path if path else [name]
    
    def __lt__(self, other):
        return self.f_cost < other.f_cost
    
    def __repr__(self):
        return f"TreeNode({self.name}, g={self.g_cost}, h={self.heuristic}, f={self.f_cost})"

class AStarTree:
    def __init__(self):
        self.heuristics: Dict[str, int] = {}
        self.edges: Dict[str, List[Tuple[str, int]]] = {}
    
    def add_node(self, name: str, heuristic: int):
        self.heuristics[name] = heuristic
        self.edges[name] = []
    
    def add_edge(self, from_node: str, to_node: str, cost: int):
        self.edges[from_node].append((to_node, cost))
    
    def a_star_tree_search(self, start: str, goal: str) -> Optional[List[str]]:
        start_node = TreeNode(start, self.heuristics[start], 0, None, [start])
        open_set = [(start_node.f_cost, id(start_node), start_node)]
        iteration = 0
        
        print(f"Starting A* Tree Search from {start} to {goal}\n")
        print(f"{'Path':<20} {'h(x)':<8} {'g(x)':<8} {'f(x)':<8}")
        print("="*50)
        
        while open_set:
            _, _, current = heapq.heappop(open_set)
            
            path_str = " -> ".join(current.path)
            print(f"{path_str:<20} {current.heuristic:<8} {current.g_cost:<8} {current.f_cost:<8}")
            
            if current.name == goal:
                print(f"\nGoal reached: {goal}")
                print(f"Final path: {' -> '.join(current.path)}")
                print(f"Total cost: {current.g_cost}")
                return current.path
            
            if current.name in self.edges:
                for neighbor_name, edge_cost in self.edges[current.name]:
                    new_g_cost = current.g_cost + edge_cost
                    new_path = current.path + [neighbor_name]
                    
                    neighbor_node = TreeNode(
                        neighbor_name,
                        self.heuristics[neighbor_name],
                        new_g_cost,
                        current,
                        new_path
                    )
                    
                    heapq.heappush(open_set, (neighbor_node.f_cost, id(neighbor_node), neighbor_node))
            
            iteration += 1
        
        print("\nNo path found!")
        return None

def create_example_tree():
    tree = AStarTree()
    
    tree.add_node('S', 7)
    tree.add_node('A', 9)
    tree.add_node('B', 4)
    tree.add_node('C', 2)
    tree.add_node('D', 5)
    tree.add_node('E', 3)
    tree.add_node('G', 0)
    
    tree.add_edge('S', 'A', 3)
    tree.add_edge('S', 'D', 2)
    tree.add_edge('A', 'B', 5)
    tree.add_edge('B', 'C', 2)
    tree.add_edge('D', 'B', 1)
    tree.add_edge('B', 'E', 1)
    tree.add_edge('C', 'G', 4)
    tree.add_edge('D', 'E', 4)
    tree.add_edge('E', 'G', 3)
    tree.add_edge('A', 'C', 10)
    
    return tree

if __name__ == "__main__":
    tree = create_example_tree()
    path = tree.a_star_tree_search('S', 'G')
    
    print("\n" + "="*50)
    print("RESULT:")
    print("="*50)
    if path:
        print(f"Optimal path: {' -> '.join(path)}")