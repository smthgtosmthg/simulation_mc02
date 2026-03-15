#!/usr/bin/env python3
"""
Inject LIDAR 2D and RGB camera sensors into an iris model.sdf.

Usage:
    python3 inject_sensors.py <model.sdf path> <drone instance number>

Called automatically by 06_launch_multi_drones.sh after copying each model.
"""

import sys


SENSOR_TEMPLATE = """
    <!-- ============ LIDAR 2D 360° ============ -->
    <link name="lidar_link">
      <pose>0 0 0.05 0 0 0</pose>
      <inertial>
        <mass>0.01</mass>
        <inertia>
          <ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <sensor name="lidar_2d" type="gpu_lidar">
        <topic>/drone_{instance}/lidar</topic>
        <update_rate>5</update_rate>
        <lidar>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.1</min>
            <max>10.0</max>
          </range>
        </lidar>
        <always_on>true</always_on>
        <visualize>true</visualize>
      </sensor>
    </link>
    <joint name="lidar_joint" type="fixed">
      <parent>iris_with_standoffs::base_link</parent>
      <child>lidar_link</child>
    </joint>

    <!-- ============ CAMERA RGB ============ -->
    <link name="camera_link">
      <pose>0.1 0 0 0 0 0</pose>
      <inertial>
        <mass>0.01</mass>
        <inertia>
          <ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <sensor name="camera_rgb" type="camera">
        <topic>/drone_{instance}/camera</topic>
        <update_rate>10</update_rate>
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
          </image>
          <clip>
            <near>0.1</near>
            <far>50</far>
          </clip>
        </camera>
        <always_on>true</always_on>
        <visualize>true</visualize>
      </sensor>
    </link>
    <joint name="camera_joint" type="fixed">
      <parent>iris_with_standoffs::base_link</parent>
      <child>camera_link</child>
    </joint>
"""


def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <model.sdf> <instance_number>")
        sys.exit(1)

    sdf_path = sys.argv[1]
    instance = int(sys.argv[2])

    sensor_xml = SENSOR_TEMPLATE.replace("{instance}", str(instance))

    with open(sdf_path, 'r') as f:
        content = f.read()

    # Insert sensors before the closing </model> tag
    marker = "</model>"
    if marker not in content:
        print(f"  ERROR: {marker} not found in {sdf_path}")
        sys.exit(1)

    content = content.replace(marker, sensor_xml + "\n  " + marker, 1)

    with open(sdf_path, 'w') as f:
        f.write(content)

    print(f"    + LIDAR + Camera injected (topics: /drone_{instance}/lidar, /drone_{instance}/camera)")


if __name__ == '__main__':
    main()
