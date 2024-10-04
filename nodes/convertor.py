#!/usr/bin/env python3
import message_filters
import rclpy
import rclpy.duration
import rclpy.time
import rclpy.qos

from autoware_auto_vehicle_msgs.msg import SteeringReport, GearReport, VelocityReport, TurnIndicatorsReport
from autoware_auto_perception_msgs.msg import PredictedObjects, PredictedObject, ObjectClassification
from autoware_to_satellite.projector import CoordinateConvertor
from geometry_msgs.msg import Pose
from math import sqrt
from rclpy.node import Node
from satellite_msgs.msg import Monitor, Coordinate, Vehicle, Object as sObject
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from tier4_map_msgs.msg import MapProjectorInfo
from typing import List, Optional


def calculate_xor_checksum(data):
    checksum = 0x00
    for byte in data:
        checksum ^= byte
    return checksum


class ConvertorNode(Node):
    __perception_msg = PredictedObjects()
    __steering_msg = SteeringReport()
    __gear_msg = GearReport()
    __velocity_msg = VelocityReport()
    __turn_indicators_msg = TurnIndicatorsReport()
    __projector: Optional[CoordinateConvertor] = None
    def __init__(self):
        super().__init__("autoware_to_satellite")

        self.__map_frame = self.declare_parameter("map_frame_id", "map").value
        self.__vehicle_frame = self.declare_parameter("vehicle_frame", "base_link").value
        self.__max_steering = self.declare_parameter("max_steering",  38.05).value
        target_rate = self.declare_parameter("target_rate", 10).value

        self.__tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=1))
        self.__tf_listener = TransformListener(buffer=self.__tf_buffer, node=self)

        self.__publisher = self.create_publisher(Monitor, "monitor", 10)

        steering_subscriber = message_filters.Subscriber(self, SteeringReport, "/vehicle/status/steering_status")
        gear_subscriber = message_filters.Subscriber(self, GearReport, "/vehicle/status/gear_status")
        velocity_subscriber = message_filters.Subscriber(self, VelocityReport, "/vehicle/status/velocity_status")
        turn_indicators_subscriber = message_filters.Subscriber(self, TurnIndicatorsReport, "/vehicle/status/turn_indicators_status")
        self.vehicle_subscriber = message_filters.ApproximateTimeSynchronizer(
            fs=[
                steering_subscriber,
                gear_subscriber,
                velocity_subscriber,
                turn_indicators_subscriber,
            ],
            queue_size=1,
            slop=0.1,
            allow_headerless=True,
        )
        self.vehicle_subscriber.registerCallback(self.vehicle_cb)

        self.create_subscription(
            MapProjectorInfo,
            "/map/map_projector_info",
            self.map_projector_cb,
            rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, depth=1
            ),
        )

        self.create_subscription(PredictedObjects, "/perception/object_recognition/objects", self.perception_cb, 1)

        self.create_timer(timer_period_sec=1/target_rate, callback=self.timer_cb)

    def vehicle_cb(
        self, steering_msg: SteeringReport, gear_msg: GearReport, velocity_msg: VelocityReport, turn_indicators_msg: TurnIndicatorsReport
    ):
        self.__steering_msg = steering_msg
        self.__gear_msg = gear_msg
        self.__velocity_msg = velocity_msg
        self.__turn_indicators_msg = turn_indicators_msg

    def perception_cb(self, msg: PredictedObjects):
        self.__perception_msg = msg

    def map_projector_cb(self, msg: MapProjectorInfo):
        try:
            self.__projector = CoordinateConvertor(msg)
        except Exception as e:
            print(e)
            pass

    def get_vehicle(
        self, steering_msg: SteeringReport, gear_msg: GearReport, velocity_msg: VelocityReport, turn_indicators_msg: TurnIndicatorsReport
    ) -> Vehicle:
        vehicle = Vehicle()
        vehicle.velocity = velocity_msg.longitudinal_velocity
        vehicle.steering = -steering_msg.steering_tire_angle * self.__max_steering

        # TODO: Need to get values from the raw_vehicle_convertor
        vehicle.accel_target = 0.0
        vehicle.brake_target = 0.0

        if turn_indicators_msg.report == TurnIndicatorsReport.ENABLE_LEFT:
            vehicle.turn_left = True
        else:
            vehicle.turn_left = False

        if turn_indicators_msg.report == TurnIndicatorsReport.ENABLE_RIGHT:
            vehicle.turn_right = True
        else:
            vehicle.turn_right = False

        if gear_msg.report in (GearReport.NEUTRAL, GearReport.PARK):
            vehicle.gear = Vehicle.GEAR_NEUTRAL
        elif gear_msg.report in (GearReport.REVERSE, GearReport.REVERSE_2):
            vehicle.gear = Vehicle.GEAR_REVERSE
        elif gear_msg.report == GearReport.NONE:
            vehicle.gear = Vehicle.GEAR_UNKNOWN
        else:
            vehicle.gear = Vehicle.GEAR_FORWARD

        return vehicle

    def get_vehicle_pose(self) -> Coordinate:
        transform = self.__tf_buffer.lookup_transform(
            target_frame=self.__map_frame, source_frame=self.__vehicle_frame, time=rclpy.time.Time()
        )
        p = Pose()
        p.position.x = transform.transform.translation.x
        p.position.y = transform.transform.translation.y
        p.position.z = transform.transform.translation.z
        p.orientation = transform.transform.rotation

        coordinate_array = self.__projector.xy_to_lat_lon([p])
        return coordinate_array[0]

    def get_satellite_objects(self, msg: PredictedObjects) -> List[sObject]:
        satellite_objects = []

        if msg.header.frame_id != self.__map_frame:
            should_transform = True
        else:
            should_transform = False

        for predicted_obj in msg.objects:
            predicted_obj: PredictedObject
            if should_transform:
                try:
                    transform = self.__tf_buffer.lookup_transform(
                        target_frame=self.__map_frame, source_frame=msg.header.frame_id, time=rclpy.time.Time()
                    )
                    pose = do_transform_pose(predicted_obj.kinematics.initial_pose_with_covariance.pose, transform)
                except Exception as e:
                    self.get_logger().warn(
                        "Failed to transform predicted object from frame {} to {}".format(msg.header.frame_id, self.__map_frame)
                    )
                    continue
            else:
                pose = predicted_obj.kinematics.initial_pose_with_covariance.pose

            coordinate = self.__projector.xy_to_lat_lon([pose])[0]

            classification = sObject.CLASSIFICATION_UNKNOWN

            if len(predicted_obj.classification) > 0:
                if predicted_obj.classification[0].label == ObjectClassification.CAR:
                    classification = sObject.CLASSIFICATION_CAR
                elif predicted_obj.classification[0].label in (
                    ObjectClassification.TRUCK,
                    ObjectClassification.BUS,
                    ObjectClassification.TRAILER,
                ):
                    classification = sObject.CLASSIFICATION_TRUCK
                elif predicted_obj.classification[0].label == ObjectClassification.MOTORCYCLE:
                    classification = sObject.CLASSIFICATION_MOTORCYCLE
                elif predicted_obj.classification[0].label == ObjectClassification.BICYCLE:
                    classification = sObject.CLASSIFICATION_BIKE
                elif predicted_obj.classification[0].label == ObjectClassification.PEDESTRIAN:
                    classification = sObject.CLASSIFICATION_PEDESTRIAN

            velocity = sqrt(
                predicted_obj.kinematics.initial_twist_with_covariance.twist.linear.x**2
                + predicted_obj.kinematics.initial_twist_with_covariance.twist.linear.y**2
                + predicted_obj.kinematics.initial_twist_with_covariance.twist.linear.z**2
            )
            satellite_obj = sObject(
                id=int(calculate_xor_checksum(predicted_obj.object_id.uuid)),
                coordinate=coordinate,
                velocity=velocity,
                lane_id=0,  # TODO calculate lane_id based on vector map
                classification=classification,
            )
            satellite_objects.append(satellite_obj)

        return satellite_objects

    def timer_cb(self):
        if self.__projector is None:
            self.get_logger().warn("Map projector info was not received", throttle_duration_sec=1.)
            return

        try:
            vehicle_coordinates = self.get_vehicle_pose()
        except Exception as e:
            self.get_logger().error("Failed to get vehicle coordinates. Error: {}".format(str(e)))
            return

        satellite_objects = self.get_satellite_objects(self.__perception_msg)
        vehicle = self.get_vehicle(
            steering_msg=self.__steering_msg,
            gear_msg=self.__gear_msg,
            velocity_msg=self.__velocity_msg,
            turn_indicators_msg=self.__turn_indicators_msg,
        )

        msg = Monitor(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id="/wgs84"),
            coordinate=vehicle_coordinates,
            vehicle=vehicle,
            objects=satellite_objects,
            road_line=0,  # TODO: fix
        )

        self.__publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ConvertorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
