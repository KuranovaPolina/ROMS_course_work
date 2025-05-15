import mujoco
import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
import time

# Генерация целевой траектории
def get_function_trajectory(func_str, x_min=-10, x_max=10, N=1000):
    x = sp.symbols('x')
    func = sp.sympify(func_str)
    f_lambdified = sp.lambdify(x, func, 'numpy')
    x_vals = np.linspace(x_min, x_max, N)
    y_vals = f_lambdified(x_vals)
    target_traj = np.zeros((N, 2))
    target_traj[:, 0] = x_vals
    target_traj[:, 1] = y_vals
    return target_traj

def MSE_func(model_xml, func, tracker_site="site_5"):
    # Загрузка модели
    model = mujoco.MjModel.from_xml_string(model_xml)
    data = mujoco.MjData(model)

    # Параметры симуляции
    T = 10.0  # Время симуляции (секунды)
    dt = model.opt.timestep  # Шаг времени
    N = int(T / dt)  # Количество шагов

    # Функция траектории движения
    target_traj = get_function_trajectory(func, x_min=0, x_max=T, N=N)

    # Массив для хранения реальной траектории
    actual_traj = np.zeros((N, 2))
    
    # Найдем ID трекера для отслеживания
    tracker_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, tracker_site)
    if tracker_id == -1:
        # Если указанный сайт не найден, попробуем найти конечные сайты (site_X)
        for i in range(1, 20):  # Проверим сайты с ID от 1 до 20
            potential_tracker = f"site_{i}"
            potential_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, potential_tracker)
            if potential_id != -1:
                print(f"Сайт '{tracker_site}' не найден. Используем '{potential_tracker}' для отслеживания.")
                tracker_id = potential_id
                break
                
        if tracker_id == -1:
            raise ValueError(f"Не удалось найти подходящий сайт для отслеживания")
    
    print(f"Отслеживаем сайт: {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, tracker_id)}")
    
    # Сохраним все координаты для анализа (x, y, z)
    all_coords = np.zeros((N, 3))

    # Запуск визуализатора
    with mujoco.viewer.launch_passive(model, data) as viewer:
        for i in range(N):
            # Простое управление: синусоидальный сигнал
            # Пробуем найти первый актуатор
            if model.nu > 0:
                data.ctrl[0] = (3.14 / 180) * i

            # Шаг симуляции
            mujoco.mj_step(model, data)

            # Обновляем физику и явно запрашиваем глобальные координаты
            mujoco.mj_forward(model, data)
            
            # site_xpos уже содержит глобальные координаты сайта в мировой системе координат
            # В отличие от pos атрибута, который содержит локальные координаты
            tracker_pos = data.site_xpos[tracker_id].copy()
            all_coords[i] = tracker_pos  # Сохраняем все координаты
            actual_traj[i] = tracker_pos[:2]  # Для расчета MSE берем только X и Y

            # Синхронизация с визуализатором
            viewer.sync()
            time.sleep(dt)

    # Анализ траектории
    print(f"Диапазон X: {np.min(all_coords[:, 0]):.3f} до {np.max(all_coords[:, 0]):.3f}")
    print(f"Диапазон Y: {np.min(all_coords[:, 1]):.3f} до {np.max(all_coords[:, 1]):.3f}")
    print(f"Диапазон Z: {np.min(all_coords[:, 2]):.3f} до {np.max(all_coords[:, 2]):.3f}")
    
    # Проверим, лежат ли точки в одной плоскости
    z_variation = np.max(all_coords[:, 2]) - np.min(all_coords[:, 2])
    if z_variation < 1e-6:
        print("Траектория плоская (все точки имеют одинаковую Z координату)")
    
    # MSE между целевой и реальной траекторией
    mse = np.mean((actual_traj - target_traj) ** 2)

        # Визуализация траекторий
    plt.figure(figsize=(12, 10))
    
    # График XY (вид сверху)
    plt.subplot(2, 1, 1)
    plt.plot(actual_traj[:, 0], actual_traj[:, 1], 'r-', label='Реальная траектория')
    plt.scatter(actual_traj[0, 0], actual_traj[0, 1], c='green', s=100, marker='o', label='Начало')
    plt.scatter(actual_traj[-1, 0], actual_traj[-1, 1], c='red', s=100, marker='x', label='Конец')
    plt.xlabel('X (м)')
    plt.ylabel('Y (м)')
    plt.title('Траектория движения в плоскости XY')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    
    # График изменения координат со временем
    plt.subplot(2, 1, 2)
    t = np.linspace(0, T, N)
    plt.plot(t, all_coords[:, 0], 'r-', label='X координата')
    plt.plot(t, all_coords[:, 1], 'g-', label='Y координата')
    plt.xlabel('Время (с)')
    plt.ylabel('Координата (м)')
    plt.title('Изменение координат со временем')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig('trajectory_analysis.png')
    plt.show()

    return mse