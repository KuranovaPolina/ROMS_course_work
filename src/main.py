from graph_model.generate_model import graphs_generator

if __name__ == "__main__":
    node_ids = ['A', 'B', 'C', 'D']  # Задаем узлы для генерации топологий

    for i in graphs_generator(node_ids, min_length=1.0, max_length=3.0, step=1.0):
        print(i)
