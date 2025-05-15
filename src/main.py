from generate_model import graphs_generator
from metric import MSE_func
from mechanism_generator.simulate_graph import generate_mujoco_xml

if __name__ == "__main__":
    node_ids = ['A', 'B', 'C', 'D', 'E']  # Задаем узлы для генерации топологий

    func = "2*sin(x)**2 + 3*cos(x)**3"

    n = 0
    for graph in graphs_generator(node_ids, min_length=1.0, max_length=2.0, step=1.0):
        xml = generate_mujoco_xml(graph)
        mse_score = MSE_func(xml, func, visualize=False)
        n += 1
        if n == 100:
            break
