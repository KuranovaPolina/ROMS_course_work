#!/usr/bin/env python3
"""
Тестовый файл для преобразования графа в XML модель и ее симуляции в MuJoCo
"""
import os
import sys
import tempfile
import mujoco
import mujoco.viewer

# Импортируем функцию для генерации XML модели
from simulate_graph import generate_mujoco_xml

def graph_to_mujoco(connections):
    """
    Преобразует граф в XML модель MuJoCo и возвращает эту строку
    
    Args:
        connections: словарь соединений в формате 
                    {'A': {'B': {'length': 1.0}, 'C': {'length': 1.0}}, ...}
    
    Returns:
        XML модель в виде строки
    """
    # Генерируем XML модель
    return generate_mujoco_xml(connections)

def test_with_mujoco(connections):
    """
    Запускает симуляцию MuJoCo с моделью, созданной из графа
    
    Args:
        connections: словарь соединений в формате 
                    {'A': {'B': {'length': 1.0}, 'C': {'length': 1.0}}, ...}
    
    Returns:
        XML модель в виде строки
    """
    # Получаем XML модель
    xml_content = graph_to_mujoco(connections)
    
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
            
        except Exception as e:
            print(f"Ошибка при симуляции модели: {str(e)}")
    
    # Возвращаем сгенерированную XML модель
    return xml_content

def test_simple_graph():
    """
    Тестирует простой четырехзвенный механизм
    """
    # Определяем граф
    connections = {"A": {"C": {"length": 1.0}}, "B": {"D": {"length": 1.0}}, "C": {"A": {"length": 1.0}, "D": {"length": 1.0}}, "D": {"B": {"length": 1.0}, "C": {"length": 1.0}}}
    
    # Запускаем симуляцию и получаем XML
    xml = test_with_mujoco(connections)
    
    # Выводим первые 200 символов XML для проверки
    if xml:
        print("\nНачало сгенерированной XML модели:")
        print(xml[:200] + "...\n")
        print(f"Полная длина XML: {len(xml)} символов")
    else:
        print("Не удалось сгенерировать XML модель")

if __name__ == "__main__":
    # Запускаем тест с простым графом
    test_simple_graph() 