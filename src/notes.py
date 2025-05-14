def build_cayley_menger_matrix(G, weight='lenght'):
    """
    Строит матрицу Кэли-Менгера на основе кратчайших расстояний между всеми парами узлов.
    
    :param G: NetworkX Graph
    :param weight: Атрибут ребра, используемый как длина
    :return: Матрица Кэли-Менгера
    """
    nodes = list(G.nodes())
    n = len(nodes)
    node_to_idx = {node: i for i, node in enumerate(nodes)}

    # Шаг 1: Вычисляем матрицу кратчайших расстояний
    dist_matrix = np.zeros((n, n))
    for u in G.nodes():
        lengths = nx.single_source_dijkstra_path_length(G, u, weight=weight)
        for v in G.nodes():
            dist_matrix[node_to_idx[u], node_to_idx[v]] = lengths.get(v, np.inf)

    # Шаг 2: Создаём матрицу Кэли-Менгера размером (n+2)x(n+2)
    CM = np.zeros((n + 2, n + 2))
    CM[0, :] = CM[:, 0] = 1  # Первая строка и столбец — единицы
    CM[0, 0] = 0  # Элемент (0, 0) — ноль

    # Заполняем остальную часть матрицы квадратами расстояний
    for i in range(n):
        for j in range(n):
            CM[i + 1, j + 1] = dist_matrix[i, j] ** 2

    return CM

CM = build_cayley_menger_matrix(G, weight='length')

print("Матрица Кэли-Менгера:")
print(CM)

# Вычисляем определитель
det = np.linalg.det(CM)
print("\nОпределитель матрицы Кэли-Менгера:", det)

# Проверяем возможность размещения точек в евклидовом пространстве
if np.isinf(det):
    print("Ошибка: граф содержит недостижимые компоненты")
    # return False
elif det <= 0:
    print("Точки могут быть размещены в евклидовом пространстве")
else:
    print("Невозможно разместить точки в евклидовом пространстве")
    # return False



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
