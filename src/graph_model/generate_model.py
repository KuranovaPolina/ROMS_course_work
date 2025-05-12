# from graph_model.graph_class import KinematicGraph

import itertools
import networkx as nx
import numpy as np

import matplotlib.pyplot as plt

# TODO - A, B - base nodes; C - pass node

def polygon_exists(sides):
    total = sum(sides)
    for side in sides:
        if not (side < total - side):
            return False
    return True

def check_cycle(G, nodes):
    lens = []

    for i in range(len(nodes) - 1):
        lens.append(G.get_edge_data(nodes[i], nodes[i + 1])['length'])
    lens.append(G.get_edge_data(nodes[len(nodes) - 1], nodes[0])['length'])

    return polygon_exists(lens)

def build_cayley_menger_matrix(G, weight='lenght'):
    # Строит матрицу Кэли-Менгера на основе кратчайших расстояний между всеми парами узлов.

    nodes = list(G.nodes())
    n = len(nodes)
    node_to_idx = {node: i for i, node in enumerate(nodes)}

    # Шаг 1: Вычисляем матрицу кратчайших расстояний
    dist_matrix = np.zeros((n, n))
    for u in G.nodes():
        lengths = nx.single_source_dijkstra_path_length(G, u, weight=weight)
        for v in G.nodes():
            dist_matrix[node_to_idx[u], node_to_idx[v]] = lengths.get(v, np.inf)

    # Шаг 2: Создаём матрицу Кэли-Менгера размером (n+1)x(n+1)
    CM = np.zeros((n + 1, n + 1))
    CM[0, :] = CM[:, 0] = 1  # Первая строка и столбец — единицы
    CM[0, 0] = 0  # Элемент (0, 0) — ноль

    # Заполняем остальную часть матрицы квадратами расстояний
    for i in range(n):
        for j in range(n):
            CM[i + 1, j + 1] = dist_matrix[i, j] ** 2

    return CM

def is_graph_valid(G, G_CM, expected_nodes_count, base1 = 'A', base2 = 'B', path = 'C'):
    # Добавили все узлы
    if expected_nodes_count != G.number_of_nodes():
        return False

    # Связность графа
    if not nx.is_connected(G):
        return False
    
    # Базовые узла связаны
    if not G.has_edge(base1, base2):
        return False
    
    # Все узлы имеют как минимум 2 соседа (замкнутая кинематика)
    for _, degree in G.degree():
        if degree < 2:
            return False
        
    # Корректность длин звеньев (циклы с такими длинами могут существовать)

    # Версия 1 - берем все циклы и проверяем, что такие многоугольники существуют
    for cycle_nodes in nx.cycle_basis(G):
        if not check_cycle(G, cycle_nodes):
            return False

    # # генрируем матрицу 
    det = np.linalg.det(G_CM)
    print("\nОпределитель матрицы Кэли-Менгера:", det)

    if np.isinf(det):
        # print("Ошибка: граф содержит недостижимые компоненты")
        return False
    elif det <= 0:
        print("Точки могут быть размещены в евклидовом пространстве")
    else:
        # print("Невозможно разместить точки в евклидовом пространстве")
        return False

    return True

def draw_graph(G):
    # Расположение узлов
    pos = nx.spring_layout(G, dim=2, weight='length')
    # pos = my_get_pos(G)
    print(pos)

    print(G.adj)

    # Рисуем узлы
    nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=800, font_size=16)

    # Рисуем рёбра
    nx.draw_networkx_edges(G, pos, width=2)

    edge_labels = {(e[0], e[1]): f"{e[2]:.2f}" for e in G.edges(data='length')}
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=12)

    plt.show()

def generate_all_graphs(node_ids, min_length=0.0, max_length=5.0, step=1.0):
    # Генерирует все возможные топологии графа с перебором длин рёбер.

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
                edges_with_lengths.append([(edge[0], edge[1], l) for l in edge_lengths])

            # Перебираем все комбинации длин
            for length_combination in itertools.product(*edges_with_lengths):
                G = nx.Graph()

                for node in node_ids:
                    G.add_node(node)

                for u, v, l in length_combination:
                    G.add_edge(u, v, length=l)

                G_CM = build_cayley_menger_matrix(G, weight='length')

                if is_graph_valid(G, G_CM, expected_nodes_count = len(node_ids)):
                    graph_topology_count += 1
                    print(f"Топология №{graph_topology_count}:")

                    draw_graph(G)
                    print(G_CM)
                    

    print(f"\nВсего подходящих топологий с корректными длинами: {graph_topology_count}")
