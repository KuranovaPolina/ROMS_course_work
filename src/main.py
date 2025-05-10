from graph_model.generate_model import generate_all_graphs

if __name__ == "__main__":
    node_ids = ['A', 'B', 'C', 'D', 'E']  # Задаем узлы для генерации топологий
    generate_all_graphs(node_ids, min_length=1.0, max_length=1.0, step=1.0)
