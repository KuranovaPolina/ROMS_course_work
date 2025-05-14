from generate_model import graphs_generator
from metric import MSE_func
from mechanism_generator.simulate_graph import generate_mujoco_xml

if __name__ == "__main__":
    node_ids = ['A', 'B', 'C', 'D']  # Задаем узлы для генерации топологий

    func = "5 * np.sqrt(1 - (x**2) / (3**2))"

    n = 0
    for graph in graphs_generator(node_ids, min_length=1.0, max_length=3.0, step=1.0):
        xml = generate_mujoco_xml(graph)
        mse_score = MSE_func(xml, func)
        print(MSE_func(xml, func))
        if mse_score <= 0.8:
            print (graph)
            break
        n += 1
        if n == 100:
            break
