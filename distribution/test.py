import numpy as np
from scipy.optimize import linear_sum_assignment

# Пример: координаты дронов
drones = np.array([[0, 0], [0, 0], [5, 2]])

# Пример: полигоны с несколькими точками
polygons = [
    np.array([[1, 1], [0, 2], [2, 0]]),  # Полигон 0
    np.array([[4, 0], [5, 1], [3, 1]]),  # Полигон 1
    np.array([[6, 3], [5, 4], [7, 2]]),  # Полигон 2
]

n = len(drones)
cost_matrix = np.zeros((n, n))

# Создаем матрицу минимальных расстояний дрон -> полигон
for i in range(n):
    for j in range(n):
        # расстояние дрона до ближайшей точки полигона
        distances = np.linalg.norm(polygons[j] - drones[i], axis=1)
        cost_matrix[i, j] = distances.min()

# Решаем задачу назначения
row_ind, col_ind = linear_sum_assignment(cost_matrix)

# Вывод результатов
total_distance = cost_matrix[row_ind, col_ind].sum()
print("Назначения дронов к полигонам:")
for drone_idx, poly_idx in zip(row_ind, col_ind):
    closest_point = polygons[poly_idx][
        np.argmin(np.linalg.norm(polygons[poly_idx] - drones[drone_idx], axis=1))
    ]
    print(
        f"Дрон {drone_idx} -> Полигон {poly_idx}, ближайшая точка {closest_point}, расстояние = {cost_matrix[drone_idx, poly_idx]:.2f}"
    )
