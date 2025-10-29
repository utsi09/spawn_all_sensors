#!/usr/bin/env python3
"""
Spawn an 'ego' vehicle and attach all sensors (based on your JSON structure)
All sensors are safely destroyed when script ends.
"""

import carla
import random
import time
import traceback

def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    print(f" Connected to CARLA map: {world.get_map().name}")

    # ============================================================
    # 1️ EGO 차량 찾기 or 새로 스폰
    # ============================================================
    vehicle = None
    for actor in world.get_actors().filter("vehicle.*"):
        if actor.attributes.get("role_name") == "ego":
            vehicle = actor
            break

    if vehicle is None:
        print(" No ego vehicle found, spawning a new one...")
        vehicle_bp = bp_lib.find("vehicle.tesla.model3")
        vehicle_bp.set_attribute("role_name", "ego")
        spawn_point = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    print(f" EGO vehicle ready: {vehicle.type_id} (ID={vehicle.id})")

    # ============================================================
    # 2 센서 부착 함수 정의
    # ============================================================
    sensors = []

    def attach_sensor(sensor_type, transform, attributes=None, name=None):
        bp = bp_lib.find(sensor_type)
        if attributes:
            for k, v in attributes.items():
                bp.set_attribute(k, str(v))
        if name:
            bp.set_attribute("role_name", name)
        sensor = world.spawn_actor(bp, transform, attach_to=vehicle)
        sensors.append(sensor)
        print(f"  - {sensor_type} attached as {name}")
        return sensor

    # ============================================================
    # 3 센서 생성 (JSON 구조 반영)
    # ============================================================

    # RGB Camera (Front)
    cam_front = attach_sensor(
        "sensor.camera.rgb",
        carla.Transform(carla.Location(x=2.0, z=2.0)),
        {"image_size_x": "1024", "image_size_y": "768", "fov": "90"},
        "rgb_front"
    )

    # Rear View Camera
    cam_view = attach_sensor(
        "sensor.camera.rgb",
        carla.Transform(carla.Location(x=-4.5, z=2.8), carla.Rotation(pitch=20.0)),
        {"image_size_x": "1024", "image_size_y": "768", "fov": "90"},
        "rgb_view"
    )

    # LiDAR
    lidar = attach_sensor(
        "sensor.lidar.ray_cast",
        carla.Transform(carla.Location(z=2.4)),
        {
            "range": "50",
            "channels": "32",
            "points_per_second": "320000",
            "rotation_frequency": "20",
            "upper_fov": "2.0",
            "lower_fov": "-26.8"
        },
        "lidar"
    )

    # Semantic LiDAR
    semantic_lidar = attach_sensor(
        "sensor.lidar.ray_cast_semantic",
        carla.Transform(carla.Location(z=2.4)),
        {
            "range": "50",
            "channels": "32",
            "points_per_second": "320000",
            "rotation_frequency": "20",
            "upper_fov": "2.0",
            "lower_fov": "-26.8"
        },
        "semantic_lidar"
    )

    # Radar
    radar = attach_sensor(
        "sensor.other.radar",
        carla.Transform(carla.Location(x=2.0, z=2.0)),
        {
            "horizontal_fov": "30",
            "vertical_fov": "10",
            "points_per_second": "1500",
            "range": "100"
        },
        "radar_front"
    )

    # Semantic Segmentation
    seg = attach_sensor(
        "sensor.camera.semantic_segmentation",
        carla.Transform(carla.Location(x=2.0, z=2.0)),
        {"fov": "90", "image_size_x": "400", "image_size_y": "70"},
        "semantic_segmentation_front"
    )

    # Depth Camera
    depth = attach_sensor(
        "sensor.camera.depth",
        carla.Transform(carla.Location(x=2.0, z=2.0)),
        {"fov": "90", "image_size_x": "400", "image_size_y": "70"},
        "depth_front"
    )

    # DVS
    dvs = attach_sensor(
        "sensor.camera.dvs",
        carla.Transform(carla.Location(x=2.0, z=2.0)),
        {
            "fov": "90",
            "image_size_x": "400",
            "image_size_y": "70",
            "positive_threshold": "0.3",
            "negative_threshold": "0.3",
            "use_log": "true",
            "log_eps": "0.001"
        },
        "dvs_front"
    )

    # GNSS
    gnss = attach_sensor(
        "sensor.other.gnss",
        carla.Transform(carla.Location(x=1.0, z=2.0)),
        {
            "noise_alt_stddev": "0.0",
            "noise_lat_stddev": "0.0",
            "noise_lon_stddev": "0.0",
            "noise_alt_bias": "0.0",
            "noise_lat_bias": "0.0",
            "noise_lon_bias": "0.0"
        },
        "gnss"
    )

    # IMU
    imu = attach_sensor(
        "sensor.other.imu",
        carla.Transform(carla.Location(x=2.0, z=2.0)),
        {
            "noise_accel_stddev_x": "0.0",
            "noise_accel_stddev_y": "0.0",
            "noise_accel_stddev_z": "0.0",
            "noise_gyro_stddev_x": "0.0",
            "noise_gyro_stddev_y": "0.0",
            "noise_gyro_stddev_z": "0.0"
        },
        "imu"
    )   

    # Collision Sensor
    collision = attach_sensor(
        "sensor.other.collision",
        carla.Transform(),
        None,
        "collision"
    )

    # Lane Invasion
    lane_invasion = attach_sensor(
        "sensor.other.lane_invasion",
        carla.Transform(),
        None,
        "lane_invasion"
    )

    # ============================================================
    # 4 콜백 (기본 작동 확인)
    # ============================================================

    def lidar_callback(pc):
        pts = len(pc.raw_data) // 16
        print(f"[LiDAR] Frame {pc.frame}: {pts} points")

    def cam_callback(image):
        print(f"[Camera] {image.frame} - {image.width}x{image.height}")

    lidar.listen(lidar_callback)
    for s in sensors:
        if "camera" in s.type_id:
            s.listen(cam_callback)

    print(" All sensors attached. Streaming started (Ctrl+C to stop).")

    while True:
        world.wait_for_tick()

# ============================================================
# 종료 처리
# ============================================================
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n🧹 Ctrl+C detected — destroying all actors...")
    except Exception:
        traceback.print_exc()
    finally:
        try:
            client = carla.Client("localhost", 2000)
            client.set_timeout(5.0)
            world = client.get_world()
            actors = world.get_actors()
            for actor in actors:
                role = actor.attributes.get("role_name", "")
                if role in [
                    "ego", "lidar", "rgb_front", "rgb_view",
                    "semantic_lidar", "radar_front", "semantic_segmentation_front",
                    "depth_front", "dvs_front", "gnss", "imu",
                    "collision", "lane_invasion"
                ]:
                    actor.destroy()
                    print(f"🗑️ destroyed: {actor.type_id} ({role})")
            print(" All actors cleaned up.")
        except Exception as e:
            print(" Cleanup failed:", e)
