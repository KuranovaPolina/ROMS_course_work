import mujoco
import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
import time
import warnings

# Генерация целевой траектории
def get_function_trajectory(func_str, x_min=-10, x_max=10, N=1000):
    x = sp.symbols('x')
    try:
        func = sp.sympify(func_str)
    except sp.SympifyError as e:
        raise ValueError(f"Ошибка парсинга функции: {func_str}. Убедитесь, что строка содержит только математическое выражение, например, '2*sin(x)**2 + 3*cos(x)**3'. Ошибка: {e}")
    f_lambdified = sp.lambdify(x, func, 'numpy')
    x_vals = np.linspace(x_min, x_max, N)
    y_vals = f_lambdified(x_vals)
    target_traj = np.zeros((N, 2))
    target_traj[:, 0] = x_vals
    target_traj[:, 1] = y_vals
    return target_traj

def MSE_func(model_xml, func, visualize=True):
    # Загрузка модели
    model = mujoco.MjModel.from_xml_string(model_xml)
    data = mujoco.MjData(model)

    # Проверка количества актуаторов
    print("Количество актуаторов:", model.nu)
    print("Размер data.ctrl:", data.ctrl.shape)

    # Параметры симуляции
    T = 20.0  # Время симуляции (секунды)
    dt = model.opt.timestep  # Шаг времени
    N = int(T / dt)  # Количество шагов

    # Функция траектории движения
    target_traj = get_function_trajectory(func, x_min=0, x_max=3, N=N)  # Ограничиваем x_max до 3

    actual_traj = np.zeros((N, 2))

    tracker_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "site_4")
    if tracker_id == -1:
        raise ValueError("Сайт 'site_3' не найден в модели")

    # Максимальный угол из ctrlrange актуатора
    max_angle = 3.14*3 # Исправлено: соответствует ctrlrange из XML

    # Запуск симуляции
    if visualize:
        # Запуск визуализатора
        with mujoco.viewer.launch_passive(model, data) as viewer:
            for i in range(N):
                # Плавное увеличение угла
                current_angle = max_angle * (i / N)  # Линейное увеличение от 0 до max_angle
                data.ctrl[0] = current_angle  # Управление первым мотором
                if model.nu > 1:  # Проверяем, есть ли второй актуатор
                    data.ctrl[1] = current_angle  # Управление вторым мотором

                # Шаг симуляции
                mujoco.mj_step(model, data)

                # Сохраняем позицию site по ID
                tracker_pos = data.site_xpos[tracker_id][:2]
                actual_traj[i] = tracker_pos

                # Синхронизация с визуализатором
                viewer.sync()
                time.sleep(dt)
    else:
        # Симуляция без визуализации
        for i in range(N):
            # Плавное увеличение угла
            current_angle = max_angle * (i / N)  # Линейное увеличение от 0 до max_angle
            data.ctrl[0] = current_angle  # Управление первым мотором
            if model.nu > 1:  # Проверяем, есть ли второй актуатор
                data.ctrl[1] = current_angle  # Управление вторым мотором

            # Шаг симуляции
            mujoco.mj_step(model, data)

            # Сохраняем позицию site по ID
            tracker_pos = data.site_xpos[tracker_id][:2]
            actual_traj[i] = tracker_pos

    # MSE между целевой и реальной траекторией
    mse = np.mean((actual_traj - target_traj) ** 2)

    print(f"MSE: {mse}")

    # Визуализация траекторий
    plt.figure(figsize=(8, 8))
    plt.plot(target_traj[:, 0], target_traj[:, 1], 'b-', label='Целевая траектория')
    plt.plot(actual_traj[:, 0], actual_traj[:, 1], 'r--', label='Реальная траектория')
    plt.xlabel('X (м)')
    plt.ylabel('Y (м)')
    plt.title('Сравнение траекторий')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.savefig('trajectory_comparison.png')
    plt.show()

    return mse