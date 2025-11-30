import numpy as np
from geometry_msgs.msg import Point32, Polygon

# Данные полигонов
area_first_points = np.array(
    [
        [40.0, 40.0],
        [48.0, 42.0],
        [45.0, 55.0],
        [38.0, 50.0],
    ]
)

area_second_points = np.array(
    [
        [10.0, 10.0],
        [15.0, 12.0],
        [14.0, 18.0],
        [9.0, 17.0],
    ]
)

drone_pose = np.array([10.14, 11.11])


# Функция для создания Polygon из numpy массива
def create_polygon(points: np.ndarray) -> Polygon:
    poly_msg = Polygon()
    poly_msg.points = [Point32(x=p[0], y=p[1], z=0.0) for p in points]
    return poly_msg


# Функция для конвертации Polygon в numpy массив
def poly_msgs_to_np(polygon: Polygon) -> np.ndarray:
    return np.array([[p.x, p.y] for p in polygon.points])


# Создаём полигоны
polygons_msgs = [create_polygon(area_first_points), create_polygon(area_second_points)]

# Конвертируем в numpy и ищем ближайшие точки к дрону
points_and_dist = []

for idx, poly_msg in enumerate(polygons_msgs):
    poly_np = poly_msgs_to_np(poly_msg)
    distances = np.linalg.norm(poly_np - drone_pose, axis=1)

    # Индекс ближайшей точки в полигоне
    min_idx = np.argmin(distances)
    points_and_dist.append((distances[min_idx], poly_np[min_idx], idx))

# Сортировка по расстоянию
points_and_dist.sort(key=lambda x: x[0])

# Вывод результатов
for distance, point, polygon_idx in points_and_dist:
    print(f"Polygon index: {polygon_idx}")
    print(f"Nearest point: {point}")
    print(f"Distance: {distance:.4f}\n")
