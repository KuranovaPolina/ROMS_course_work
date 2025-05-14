import itertools
import networkx as nx
import numpy as np

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

def is_graph_valid(G, expected_nodes_count, base1 = 'A', base2 = 'B'):
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

    # Версия 2 - проверка с пмомощью матрицы Кэли-Менгера
    G_CM = build_cayley_menger_matrix(G, weight='length')

    det = np.linalg.det(G_CM)

    if np.isinf(det) or det > 0:
        return False

    return True

def graphs_generator(node_ids, min_length=0.0, max_length=5.0, step=1.0):
    # Генерирует все возможные топологии графа с перебором длин рёбер.
    all_edges = list(itertools.combinations(node_ids, 2))

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

                if is_graph_valid(G, expected_nodes_count = len(node_ids)):
                    yield G.adj
