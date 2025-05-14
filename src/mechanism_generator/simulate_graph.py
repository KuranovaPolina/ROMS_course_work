#!/usr/bin/env python3
import os
import tempfile
import argparse
import sys
import mujoco
import mujoco.viewer
import numpy as np

# Добавляем текущую директорию в путь поиска модулей
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from topology_to_mechanism import topology_to_mechanism_format
from mechanism_parser import MechanismParser

def generate_mujoco_xml(connections):
    """
    Генерирует XML модель MuJoCo на основе словаря соединений
    
    Args:
        connections: словарь связей {'A': {'B': {'length': 1.0}, ...}, ...}
    
    Returns:
        XML модель в виде строки
    """
    try:
        # Определяем правила по заданию
        # Первая точка (первая в алфавитном порядке) всегда base и motor
        sorted_nodes = sorted(connections.keys())
        base_node = sorted_nodes[0]
        motor_nodes = [base_node]
        
        # Вторая точка (вторая в алфавитном порядке) всегда end
        end_nodes = [sorted_nodes[1]]
        
        # Формируем описание механизма
        mechanism_description = topology_to_mechanism_format(
            connections, base_node, end_nodes, motor_nodes
        )
        
        # Создаем временный файл с описанием механизма
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as temp_file:
            temp_file.write(mechanism_description)
            temp_file_path = temp_file.name
        
        try:
            # Создаем парсер механизма
            parser = MechanismParser()
            
            # Парсим описание механизма
            parser.parse_file(temp_file_path)
            
            # Строим механизм
            generator = parser.build_mechanism()
            
            # Создаем временный файл для XML
            xml_file = tempfile.NamedTemporaryFile(suffix='.xml', delete=False)
            xml_path = xml_file.name
            xml_file.close()
            
            # Сохраняем механизм в XML файл
            parser.save_mechanism(xml_path)
            
            # Читаем содержимое XML файла
            with open(xml_path, 'r') as f:
                xml_content = f.read()
                
            return xml_content
            
        except Exception as e:
            print(f"Ошибка при создании модели: {str(e)}")
            return None
        
        finally:
            # Удаляем временные файлы
            if os.path.exists(temp_file_path):
                os.remove(temp_file_path)
            
            if os.path.exists(xml_path):
                os.remove(xml_path)
    
    except Exception as e:
        print(f"Ошибка при обработке графа: {str(e)}")
        return None

def simulate_graph(connections):
    """
    Симулирует механизм на основе графового описания
    
    Args:
        connections: словарь связей {'A': {'B': {'length': 1.0}, ...}, ...}
    
    Returns:
        XML модель в виде строки
    """
    # Генерируем XML модель
    xml_content = generate_mujoco_xml(connections)
    
    if xml_content:
        try:
            # Создаем временный файл для XML
            with tempfile.NamedTemporaryFile(suffix='.xml', delete=False) as temp_xml:
                temp_xml.write(xml_content.encode('utf-8'))
                xml_path = temp_xml.name
            
            # Загружаем модель в MuJoCo
            model = mujoco.MjModel.from_xml_path(xml_path)
            data = mujoco.MjData(model)
            
            # Запускаем симуляцию
            print("Запуск симуляции...")
            with mujoco.viewer.launch_passive(model, data) as viewer:
                while viewer.is_running():
                    mujoco.mj_step(model, data)
                    viewer.sync()
            
            # Удаляем временный файл
            os.remove(xml_path)
            
            return xml_content
            
        except Exception as e:
            print(f"Ошибка при симуляции модели: {str(e)}")
            return None
    else:
        return None

def main():
    parser = argparse.ArgumentParser(description='Симуляция механизма на основе графового описания')
    
    parser.add_argument('--connections', '-c', required=True, type=str,
                        help='Словарь соединений в формате Python (например: "{\\"A\\": {\\"B\\": {\\"length\\": 1.0}, ...}, ...}")')
    
    args = parser.parse_args()
    
    try:
        # Парсим словарь соединений
        connections = eval(args.connections)
        
        # Запускаем симуляцию и получаем XML модель
        xml_content = simulate_graph(connections)
        
        if xml_content:
            print("\nСгенерированная XML модель:")
            print(xml_content)
            return 0
        else:
            return 1
        
    except Exception as e:
        print(f"Ошибка: {str(e)}")
        return 1

if __name__ == "__main__":
    sys.exit(main())
