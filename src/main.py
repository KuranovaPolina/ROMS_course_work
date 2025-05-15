from generate_model import graphs_generator
from metric import MSE_func
from mechanism_generator.simulate_graph import generate_mujoco_xml

if __name__ == "__main__":
    node_ids = ['A', 'B', 'C', 'D', 'E']  # Задаем узлы для генерации топологий
    func = "(2*sin(x)**2 + 3*cos(x)**3) - 2"
    
    # Список для хранения истории
    history = []
    
    n = 0
    for graph in graphs_generator(node_ids, min_length=0.5, max_length=1.0, step=1.0):
        xml = generate_mujoco_xml(graph)
        mse_score = MSE_func(xml, func, site=4, vis_sim=False, vis_plt=False)
        
        # Сохраняем данные в историю
        history.append({
            'id': n,
            'xml': xml,
            'mse': mse_score
        })
        
        print(f'id {n}, mse:{mse_score}')

        n += 1
        if n >= 50:
            break
    
    # Находим модель с минимальным MSE
    best_model = min(history, key=lambda x: x['mse'])
    print(f"Лучшая модель: ID = {best_model['id']}, MSE = {best_model['mse']}")

    MSE_func(best_model['xml'], func, site=3, vis_sim=True, vis_plt=True)