import mujoco
import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
import time
import warnings

# Генерация целевой траектории
def get_function_trajectory(func_str, x_min=0, x_max=3, N=1000):
    x = sp.symbols('x')
    try:
        func = sp.sympify(func_str)
    except sp.SympifyError as e:
        raise ValueError(f"Ошибка парсинга функции: {func_str}. Убедитесь, что строка содержит только математическое выражение, например, '2*sin(x)**2 + 3*cos(x)**3'. Ошибка: {e}")
    f_lambdified = sp.lambdify(x, func, 'numpy')
    x_vals = np.linspace(x_min, x_max, N)
    y_vals = f_lambdified(x_vals)
    target_traj = np.zeros((N, 2))
    target_traj[:, 0] = x_vals - x_vals[0]  # Сдвиг, чтобы x начинался с 0
    target_traj[:, 1] = y_vals - y_vals[0]  # Сдвиг, чтобы y начинался с 0
    return target_traj

def MSE_func(model_xml, func, site, vis_sim=True, vis_plt=True):
    # Загрузка модели
    model = mujoco.MjModel.from_xml_string(model_xml)
    data = mujoco.MjData(model)

    # Проверка количества актуаторов
    print("Количество актуаторов:", model.nu)
    print("Размер data.ctrl:", data.ctrl.shape)

    # Параметры симуляции
    T = 10.0  # Время симуляции (секунды)
    dt = model.opt.timestep  # Шаг времени
    N = int(T / dt)  # Количество шагов

    # Функция траектории движения
    target_traj = get_function_trajectory(func, x_min=0, x_max=3, N=N)

    # Массив для хранения траектории
    actual_traj = np.zeros((N, 2))  # Траектория для X и Y
    all_coords = np.zeros((N, 3))   # Полные 3D-координаты

    # Идентификатор сайта
    site_name = f"site_{site}"
    tracker_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    if tracker_id == -1:
        raise ValueError(f"Сайт {site_name} не найден в модели")

    # Максимальный угол из ctrlrange актуатора
    max_angle = 3.14 * 4  # Соответствует ctrlrange из XML

    # Получаем начальную позицию точки
    mujoco.mj_step(model, data)  # Выполняем один шаг, чтобы инициализировать позиции
    initial_pos = data.site_xpos[tracker_id][:2].copy()  # Начальная позиция основного сайта

    # Запуск симуляции
    if vis_sim:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            for i in range(N):
                # Плавное увеличение угла (как в первом коде)
                current_angle = max_angle * (i / N)
                data.ctrl[0] = current_angle
                if model.nu > 1:
                    data.ctrl[1] = current_angle

                # Шаг симуляции
                mujoco.mj_step(model, data)
                mujoco.mj_forward(model, data)  # Явное обновление физики (как во втором коде)

                # Сохраняем позицию сайта
                tracker_pos = data.site_xpos[tracker_id].copy()
                all_coords[i] = tracker_pos  # Сохраняем все координаты (как во втором коде)
                actual_traj[i] = tracker_pos[:2] - initial_pos  # Сдвиг относительно начальной позиции (как в первом коде)

                viewer.sync()
                time.sleep(dt)
    else:
        for i in range(N):
            # Плавное увеличение угла
            current_angle = max_angle * (i / N)
            data.ctrl[0] = current_angle
            if model.nu > 1:
                data.ctrl[1] = current_angle

            # Шаг симуляции
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)

            # Сохраняем позицию сайта
            tracker_pos = data.site_xpos[tracker_id].copy()
            all_coords[i] = tracker_pos
            actual_traj[i] = tracker_pos[:2] - initial_pos

    # Проверка плоскости траектории (как во втором коде)
    print(f"Диапазон X: {np.min(all_coords[:, 0]):.3f} до {np.max(all_coords[:, 0]):.3f}")
    print(f"Диапазон Y: {np.min(all_coords[:, 1]):.3f} до {np.max(all_coords[:, 1]):.3f}")
    print(f"Диапазон Z: {np.min(all_coords[:, 2]):.3f} до {np.max(all_coords[:, 2]):.3f}")
    z_variation = np.max(all_coords[:, 2]) - np.min(all_coords[:, 2])
    if z_variation < 1e-6:
        print("Траектория плоская (все точки имеют одинаковую Z координату)")

    # Расчет MSE
    mse = np.mean((actual_traj - target_traj) ** 2)

    # Вывод MSE в консоль
    print(f"MSE (site_{site}): {mse}")

    # Визуализация траекторий (как в первом коде)
    if vis_plt:
        plt.figure(figsize=(10, 8))
        plt.plot(target_traj[:, 0], target_traj[:, 1], 'b-', label='Целевая траектория')
        plt.plot(actual_traj[:, 0], actual_traj[:, 1], 'r--', label=f'Реальная траектория (site_{site})')
        plt.xlabel('X (м)')
        plt.ylabel('Y (м)')
        plt.title('Сравнение траекторий')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.savefig('trajectory_comparison.png')
        plt.show()

    return mse