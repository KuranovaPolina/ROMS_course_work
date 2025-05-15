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

def MSE_func(model_xml, func):
    # Загрузка модели
    # model = mujoco.MjModel.from_xml_string(model_xml)
    model = mujoco.MjModel.from_xml_path("/home/kirill/vs_code_pr/ROMS_course_work/parsed_mechanism.xml")
    data = mujoco.MjData(model)

    # Параметры симуляции
    T = 10.0  # Время симуляции (секунды)
    dt = model.opt.timestep  # Шаг времени
    N = int(T / dt)  # Количество шагов

    # Функция траектории движения
    target_traj = get_function_trajectory(func, x_min=0, x_max=T, N=N)

    actual_traj = np.zeros((N, 2))

    tracker_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "site_3")
    if tracker_id == -1:
        raise ValueError("Сайт 'site_3' не найден в модели")

    # Запуск визуализатора
    with mujoco.viewer.launch_passive(model, data) as viewer:
        for i in range(N):
            # Простое управление: синусоидальный сигнал
            data.ctrl[0] = 5.0 * np.sin(2 * np.pi * i * dt)

            # Шаг симуляции
            mujoco.mj_step(model, data)

            # Сохраняем позицию site по ID
            tracker_pos = data.site_xpos[tracker_id][:2]
            actual_traj[i] = tracker_pos

            # Синхронизация с визуализатором
            viewer.sync()
            time.sleep(dt)

    # MSE между целевой и реальной траекторией
    mse = np.mean((actual_traj - target_traj) ** 2)

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