import rclpy
from scipy.optimize import linear_sum_assignment
import numpy as np
from geometry_msgs.msg import Polygon
from std_msgs.msg import Int64
from rclpy.node import Node
from distribution_msgs.srv import Distribution


class PolygonDistribution(Node):
    def __init__(self):
        super().__init__("distribution_zones")
        self.srv = self.create_service(
            Distribution, "distribution_zones", self.distribute
        )

    def poly_msgs_to_np(self, polygon: Polygon) -> np.ndarray:
        return np.array([[p.x, p.y] for p in polygon.points])

    def distribute(self, request, response):
        polygons_msg = []
        for msg in request.polygons:
            polygons_msg.append(self.poly_msgs_to_np(msg))

        drone_poses_list = []
        for point in request.drone_start_point:
            drone_poses_list.append([point.x, point.y])
        drone_poses_list = np.array(drone_poses_list)
        n = len(drone_poses_list)
        cost_matrix = np.zeros((n, n))

        for i in range(n):
            for j in range(n):
                distances = np.linalg.norm(
                    polygons_msg[j] - drone_poses_list[i], axis=1
                )
                cost_matrix[i, j] = distances.min()
        _, col_ind = linear_sum_assignment(cost_matrix)
        for point in col_ind:
            response.sorted_by_uavs_polygons.append(point)
        return response


def main():
    rclpy.init()
    node = PolygonDistribution()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
