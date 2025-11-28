import os
import rclpy
from geometry_msgs.msg import Polygon as RosPolygon, Point32
from rclpy.node import Node
from shapely.geometry import Polygon, LineString, GeometryCollection
from shapely.ops import split


class PolygonDistribution(Node):
    def __init__(self):
        super().__init__("distribution_zones")


def main():
    pass


if __name__ == "__main__":
    main()
