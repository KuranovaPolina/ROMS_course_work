import numpy as np

def compute_node_coordinates(connections, iterations=1000):
    """
    Вычисляет координаты узлов на основе словаря соединений и их длин
    с использованием алгоритма force-directed graph layout
    
    Args:
        connections: словарь связей {'A': {'B': {'length': 1.0}, ...}, ...}
        iterations: количество итераций алгоритма
        
    Returns:
        словарь с координатами узлов {'A': [x, y], 'B': [x, y], ...}
    """
    # Извлекаем все уникальные узлы
    nodes = list(connections.keys())
    n = len(nodes)
    
    # Преобразуем буквенные узлы в индексы для удобства
    node_to_idx = {node: i for i, node in enumerate(nodes)}
    
    # Инициализируем случайные координаты
    coords = np.random.rand(n, 2) * 2 - 1  # Случайные координаты в диапазоне [-1, 1]
    
    # Создаем матрицу длин соединений
    lengths = np.zeros((n, n))
    for i, node1 in enumerate(nodes):
        for node2 in connections[node1]:
            j = node_to_idx[node2]
            length = connections[node1][node2]['length']
            lengths[i, j] = length
            lengths[j, i] = length  # Симметричная матрица
    
    # Выполняем итерации алгоритма
    for _ in range(iterations):
        forces = np.zeros((n, 2))
        
        # Вычисляем силы между всеми парами узлов
        for i in range(n):
            for j in range(i+1, n):
                if lengths[i, j] > 0:  # Если есть соединение
                    # Вектор от i к j
                    direction = coords[j] - coords[i]
                    distance = np.linalg.norm(direction)
                    
                    # Нормализуем направление
                    if distance > 0:
                        direction = direction / distance
                    
                    # Сила пропорциональна разнице между текущим расстоянием и желаемой длиной
                    strength = (distance - lengths[i, j]) * 0.1
                    
                    # Применяем силу к обоим узлам в противоположных направлениях
                    forces[i] += strength * direction
                    forces[j] -= strength * direction
        
        # Обновляем позиции
        coords += forces
    
    # Масштабируем координаты для лучшей визуализации
    scale_factor = 2.0 / np.max(np.abs(coords))
    coords *= scale_factor
    
    # Преобразуем результат в словарь
    result = {nodes[i]: coords[i].tolist() for i in range(n)}
    return result

def topology_to_mechanism_format(connections, base_node='A', end_nodes=None, motor_nodes=None):
    """
    Преобразует топологию механизма в формат для mechanism_parser
    
    Args:
        connections: словарь связей {'A': {'B': {'length': 1.0}, ...}, ...}
        base_node: узел, который будет считаться базовым (по умолчанию 'A')
        end_nodes: список узлов, которые будут считаться конечными (якорями)
        motor_nodes: список узлов, у которых будет мотор
        
    Returns:
        строки в формате, подходящем для mechanism_parser
    """
    if end_nodes is None:
        end_nodes = []
    if motor_nodes is None:
        motor_nodes = []
    
    # Вычисляем координаты узлов на основе структуры соединений
    nodes = compute_node_coordinates(connections)
    
    # Создаем маппинг между буквенными и числовыми идентификаторами
    node_mapping = {node: i+1 for i, node in enumerate(sorted(nodes.keys()))}
    
    # Создаем обратный маппинг для удобства
    reverse_mapping = {v: k for k, v in node_mapping.items()}
    
    result_lines = []
    
    # Преобразуем каждый узел в строку формата parser'а
    for node_id, node_label in reverse_mapping.items():
        pos_x, pos_y = nodes[node_label]
        
        # Определяем флаги для узла
        is_motor = 1 if node_label in motor_nodes else 0
        is_base = 1 if node_label == base_node else 0
        is_end = 1 if node_label in end_nodes else 0
        
        # Определяем связи, исключая соединения между base и end точками
        node_connections = connections[node_label]
        connection_ids = []
        
        for conn in node_connections.keys():
            # Исключаем соединения между base и end точками
            if (node_label == base_node and conn in end_nodes) or \
               (node_label in end_nodes and conn == base_node):
                print(f"Исключение соединения между base ({node_label}) и end ({conn})")
                continue
            connection_ids.append(node_mapping[conn])
        
        # Формируем строку
        line = f"{node_id} {pos_x} {pos_y} {is_motor} {is_base} {is_end} {' '.join(map(str, connection_ids))}"
        result_lines.append(line)
    
    return "\n".join(result_lines)

def parse_topology_description(topology_text):
    """
    Парсит текстовое описание топологии
    
    Args:
        topology_text: текст с описанием топологии
        
    Returns:
        словарь связей
    """
    # Разбиваем текст на части
    lines = topology_text.strip().split('\n')
    
    # Ищем строку с описанием соединений
    for line in lines:
        if "{'length'" in line:
            connections_text = line.strip()
            connections = eval(connections_text)
            return connections
    
    raise ValueError("Не найдено описание соединений в формате {'A': {'B': {'length': 1.0}, ...}, ...}")

def main():
    # Пример использования
    connections_text = """{"A": {"C": {"length": 1.0}}, "B": {"D": {"length": 1.0}}, "C": {"A": {"length": 1.0}, "D": {"length": 1.0}}, "D": {"B": {"length": 1.0}, "C": {"length": 1.0}}}"""
    
    # Парсим только соединения
    connections = eval(connections_text)
    
    # Для примера: присваиваем произвольную конфигурацию
    base_node = 'A'
    end_nodes = ['D']
    motor_nodes = ['A', 'C']
    
    result = topology_to_mechanism_format(connections, base_node, end_nodes, motor_nodes)
    
    print("Преобразованный формат для mechanism_parser:")
    print(result)
    
    # Сохраняем результат в файл
    with open('mechanism_description.txt', 'w') as f:
        f.write(result)
    print("Результат сохранен в файл 'mechanism_description.txt'")

if __name__ == "__main__":
    main() 