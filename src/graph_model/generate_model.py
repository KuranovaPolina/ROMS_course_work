from graph_model.graph_class import KinematicGraph

import itertools
import numpy as np

def is_graph_valid(graph, node_ids):
    """
    Проверяет, удовлетворяет ли граф условиям:
    1. Связность
    2. Наличие цикла
    3. Корректность длин звеньев
    """
    # Проверка связности
    visited = set()
    def dfs(node):
        visited.add(node)
        for neighbor in node.neighbors:
            if neighbor not in visited:
                dfs(neighbor)

    if graph.nodes:
        dfs(next(iter(graph.nodes.values())))  # Начинаем с первого узла
        if len(visited) != len(node_ids):
            return False  # Граф несвязный

    # Проверка наличия цикла
    # if not graph.has_cycle():
    #     return False

    # Проверка корректности длин
    # if not graph.is_lengths_valid():
    #     return False

    return True

def generate_all_graphs(node_ids, min_length=0.5, max_length=5.0, step=1.0):
    """
    Генерирует все возможные топологии графа с перебором длин рёбер.
    """
    all_edges = list(itertools.combinations(node_ids, 2))
    graph_topology_count = 0

    print(f"Генерация всех возможных топологий для {len(node_ids)} узлов ({len(all_edges)} возможных рёбер):")

    print(node_ids)
    print(all_edges)

    for r in range(1, len(all_edges) + 1):
        for edge_combination in itertools.combinations(all_edges, r):
            # Генерируем все возможные комбинации длин для текущего набора рёбер
            edges_with_lengths = []
            for edge in edge_combination:
                edge_lengths = [round(l, 1) for l in np.arange(min_length, max_length + step, step)]
                # print(edge_lengths)
                edges_with_lengths.append([(edge[0], edge[1], l) for l in edge_lengths])

            # Перебираем все комбинации длин
            for length_combination in itertools.product(*edges_with_lengths):
                kg = KinematicGraph()
                for u, v, l in length_combination:
                    kg.add_edge(u, v, l)

            # Проверяем условия: связность, цикл и корректность длин
                if is_graph_valid(kg, node_ids):
                    graph_topology_count += 1
                    print(f"\nТопология #{graph_topology_count}:")
                    kg.display_structure()

                    kg.draw()

    print(f"\nВсего подходящих топологий с корректными длинами: {graph_topology_count}")
