
# # class KinematicGraph:
# #     """Графовая модель кинематической цепи"""
# #     def __init__(self):
# #         self.nodes = {}
# #         self.edges = []

# #     def add_node(self, node_id):
# #         if node_id not in self.nodes:
# #             self.nodes[node_id] = Node(node_id)
# #         return self.nodes[node_id]

# #     def add_edge(self, node_id1, node_id2):
# #         node1 = self.add_node(node_id1)
# #         node2 = self.add_node(node_id2)
# #         node1.add_neighbor(node2)
# #         self.edges.append(Edge(node1, node2))

# #     def has_cycle(self):
# #         """Проверка наличия циклов в графе (DFS-алгоритм)"""
# #         visited = set()
# #         parent_map = {}

# #         def dfs(node, parent):
# #             visited.add(node)
# #             parent_map[node] = parent
# #             for neighbor in node.neighbors:
# #                 if neighbor not in visited:
# #                     if dfs(neighbor, node):
# #                         return True
# #                 elif neighbor != parent:
# #                     return True
# #             return False

# #         for node in self.nodes.values():
# #             if node not in visited:
# #                 if dfs(node, None):
# #                     return True
# #         return False

# #     def display_structure(self):
# #         """Вывод структуры графа"""
# #         print("Узлы:")
# #         for node in self.nodes.values():
# #             print(f"  {node.id}: {[n.id for n in node.neighbors]}")
# #         print("Рёбра:")
# #         for edge in self.edges:
# #             print(f"  {edge.start.id} <-> {edge.end.id}")
# #         print("Наличие цикла:", "Да" if self.has_cycle() else "Нет")





# # # Пример использования
# # if __name__ == "__main__":
# #     node_ids = ['A', 'B', 'C', 'D']  # Задаем узлы для генерации топологий
# #     generate_all_graphs(node_ids)

# # #######

# class KinematicGraph:


#     def is_lengths_valid(self):
#         """
#         Проверка, могут ли звенья с заданными длинами образовать замкнутый цикл.
#         Пример: для четырёхзвенного механизма проверяем, может ли он замкнуться.
#         """
#         # Проверяем только циклы
#         if not self.has_cycle():
#             return True  # Не цикл — не проверяем длины

#         # Извлекаем цикл (упрощённый случай)
#         cycle_nodes = self.find_cycle()
#         if not cycle_nodes:
#             return False

#         # Для простоты проверяем, может ли четырёхзвенный механизм замкнуться
#         if len(cycle_nodes) == 4:
#             lengths = [self.get_edge_length(cycle_nodes[i], cycle_nodes[(i+1)%4]) for i in range(4)]
#             # Условие замкнутости четырёхугольника: сумма любых трёх сторон > четвёртой
#             for i in range(4):
#                 if sum(lengths[:i] + lengths[i+1:]) <= lengths[i]:
#                     return False
#             return True
#         return True  # Для других циклов пока считаем корректными

#     def get_edge_length(self, node1, node2):
#         """Возвращает длину ребра между двумя узлами"""
#         for edge in self.edges:
#             if (edge.start == node1 and edge.end == node2) or (edge.start == node2 and edge.end == node1):
#                 return edge.length
#         return None

#     def find_cycle(self):
#         """Находит один из циклов в графе (DFS)"""
#         visited = set()
#         path = []

#         def dfs(node, parent):
#             visited.add(node)
#             path.append(node)
#             for neighbor in node.neighbors:
#                 if neighbor not in visited:
#                     if dfs(neighbor, node):
#                         return True
#                 elif neighbor != parent:
#                     # Найден цикл
#                     path.append(neighbor)
#                     return True
#             path.pop()
#             return False

#         for node in self.nodes.values():
#             if node not in visited:
#                 if dfs(node, None):
#                     # Извлекаем цикл из path
#                     cycle_start = path[-1]
#                     idx = path.index(cycle_start)
#                     return path[idx:-1]
#         return []


