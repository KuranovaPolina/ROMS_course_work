import networkx as nx
import matplotlib.pyplot as plt

class Node:
    """Узел (соединение) в кинематической цепи"""
    def __init__(self, node_id):
        self.id = node_id
        self.neighbors = []  # Список соседних узлов

    def add_neighbor(self, neighbor):
        if neighbor not in self.neighbors:
            self.neighbors.append(neighbor)
            neighbor.add_neighbor(self)  # Двустороннее соединение

    def __repr__(self):
        return f"Node({self.id})"

class Edge:
    """Ребро (звено) между узлами с заданной длиной"""
    def __init__(self, start, end, length=1.0):
        self.start = start
        self.end = end
        self.length = length  # Длина звена

    def __repr__(self):
        return f"Edge({self.start.id} <-> {self.end.id}, {self.length})"

class KinematicGraph:
    """Графовая модель кинематической цепи с длинами звеньев"""
    def __init__(self):
        self.nodes = {}
        self.edges = []

    def add_node(self, node_id):
        if node_id not in self.nodes:
            self.nodes[node_id] = Node(node_id)
        return self.nodes[node_id]

    def add_edge(self, node_id1, node_id2, length=1.0):
        node1 = self.add_node(node_id1)
        node2 = self.add_node(node_id2)
        node1.add_neighbor(node2)
        self.edges.append(Edge(node1, node2, length))

    def has_cycle(self):
        """Проверка наличия циклов в графе (DFS-алгоритм)"""
        visited = set()
        parent_map = {}

        def dfs(node, parent):
            visited.add(node)
            parent_map[node] = parent
            for neighbor in node.neighbors:
                if neighbor not in visited:
                    if dfs(neighbor, node):
                        return True
                elif neighbor != parent:
                    return True
            return False

        for node in self.nodes.values():
            if node not in visited:
                if dfs(node, None):
                    return True
        return False

    def display_structure(self):
        """Вывод структуры графа с длинами звеньев"""
        print("Узлы:")
        for node in self.nodes.values():
            print(f"  {node.id}: {[n.id for n in node.neighbors]}")
        print("Рёбра:")
        for edge in self.edges:
            print(f"  {edge.start.id} <-> {edge.end.id} (длина: {edge.length})")
        # print("Наличие цикла:", "Да" if self.has_cycle() else "Нет")
        # print("Корректность длин:", "Да" if self.is_lengths_valid() else "Нет")

    def draw(self):
        """
        Визуализация графа с помощью networkx и matplotlib.
        Отображает узлы, рёбра и длины звеньев.
        """
        if not self.nodes:
            print("Граф пустой.")
            return

        # Создаем граф networkx
        G = nx.Graph()

        # Добавляем рёбра с длинами
        for edge in self.edges:
            G.add_edge(edge.start.id, edge.end.id, weight=edge.length)
                
        # Расположение узлов
        pos = nx.spring_layout(G)

        # Рисуем узлы
        nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=800, font_size=16)

        # Рисуем рёбра
        nx.draw_networkx_edges(G, pos, width=2)

        # Добавляем метки длин рёбер
        edge_labels = {(e.start.id, e.end.id): f"{e.length:.2f}" for e in self.edges}
        nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=12)

        # Отображаем граф
        plt.title("Кинематический граф")
        plt.show()
