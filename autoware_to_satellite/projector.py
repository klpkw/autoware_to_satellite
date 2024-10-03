from abc import ABC, abstractmethod

from geometry_msgs.msg import Pose
from satellite_msgs.msg import Coordinate
from tf_transformations import euler_from_quaternion
from tier4_map_msgs.msg import MapProjectorInfo
from typing import List


class CoordinateConvertor(ABC):
    def __new__(cls, msg: MapProjectorInfo):
        if msg.vertical_datum != MapProjectorInfo.WGS84:
            raise NotImplementedError("Unsupported vertical datum in map_projector_info message")

        if msg.projector_type == MapProjectorInfo.TRANSVERSE_MERCATOR:
            return super().__new__(TranverseMercator)
        else:
            raise NotImplementedError(f"Currently unsupported projector_type {msg.projector_type}")

    @abstractmethod
    def xy_to_lat_lon(self, xy_data: List[Pose]) -> List[Coordinate]:
        ...

class TranverseMercator(CoordinateConvertor):
    def __init__(self, msg: MapProjectorInfo):
        from swri_transform_util.wgs84_transformer import Wgs84Transformer
        from geometry_msgs.msg import PoseStamped

        p = PoseStamped()
        p.pose.position.y = msg.map_origin.latitude
        p.pose.position.x = msg.map_origin.longitude
        p.pose.position.z = msg.map_origin.altitude

        self.z = msg.map_origin.altitude
        self.__projector = Wgs84Transformer(p)

    def xy_to_lat_lon(self, xy_data: List[Pose]) -> List[Coordinate]:
        coordinates = []
        for pose in xy_data:
            transformed_pt = self.__projector.local_xy_to_wgs84([pose.position.x, pose.position.y])
            r,p,y = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            coordinates.append(
                Coordinate(
                    latitude=transformed_pt[0],
                    longitude=transformed_pt[1],
                    altitude=self.z - pose.position.z,
                    azimuth=y
                )
            )
        return coordinates