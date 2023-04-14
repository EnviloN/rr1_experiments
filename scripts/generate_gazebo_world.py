#!/usr/bin/env python3
import argparse, subprocess, math

PATH = "/home/envilon/dev_ws/src/rr1_experiments/worlds/experiment"

start = """
<sdf version='1.7'>
  <world name='default'>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <contact>
              <collide_bitmask>65535</collide_bitmask>
              <ode/>
            </contact>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>
    <audio>
      <device>default</device>
    </audio>
    <wind/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
"""
static_model = """
    <model name='{}'>
      <static>1</static>
      <link name='link'>
        <pose>0 0 0 0 -0 0</pose>
        <visual name='visual'>
          <geometry>
            <mesh>
              <uri>model://Boat/mesh/boat.dae</uri>
              <scale>0.1 0.1 0.1</scale>
            </mesh>
          </geometry>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose>1.2835 0.898814 0 0 -0 0</pose>
    </model>
"""
dynamic_model = """
    <model name='{}'>
      <link name='link_1'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <pose>0 0 0 0 -0 0</pose>
        <gravity>1</gravity>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
        <enable_wind>0</enable_wind>
        <visual name='visual'>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>/home/envilon/dev_ws/src/rr1_experiments/models/meshes/boat.dae</uri>
              <scale>0.1 0.1 0.1</scale>
            </mesh>
          </geometry>
        </visual>
        <collision name='collision'>
          <laser_retro>0</laser_retro>
          <max_contacts>10</max_contacts>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>/home/envilon/dev_ws/src/rr1_experiments/models/meshes/boat_collider.stl</uri>
              <scale>0.1 0.1 0.1</scale>
            </mesh>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
                <fdir1>0 0 0</fdir1>
                <slip1>0</slip1>
                <slip2>0</slip2>
              </ode>
              <torsional>
                <coefficient>1</coefficient>
                <patch_radius>0</patch_radius>
                <surface_radius>0</surface_radius>
                <use_patch_radius>1</use_patch_radius>
                <ode>
                  <slip>0</slip>
                </ode>
              </torsional>
            </friction>
            <bounce>
              <restitution_coefficient>0</restitution_coefficient>
              <threshold>1e+06</threshold>
            </bounce>
            <contact>
              <collide_without_contact>0</collide_without_contact>
              <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
              <collide_bitmask>1</collide_bitmask>
              <ode>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
                <max_vel>0.01</max_vel>
                <min_depth>0</min_depth>
              </ode>
              <bullet>
                <split_impulse>1</split_impulse>
                <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
              </bullet>
            </contact>
          </surface>
        </collision>
      </link>
      <static>0</static>
      <allow_auto_disable>1</allow_auto_disable>
      <pose>-0.097601 0.623831 0 0 -0 0</pose>
    </model>
"""
middle = """
    <state world_name='default'>
      <sim_time>94 51000000</sim_time>
      <real_time>94 175151430</real_time>
      <wall_time>1680964359 107253009</wall_time>
      <iterations>94051</iterations>
"""
model_state = """
      <model name='{}'>
        <pose>{}</pose>
        <scale>1 1 1</scale>
        <link name='link_1'>
          <pose>{}</pose>
          <velocity>0 0 0 0 0 0</velocity>
          <acceleration>0 0 0 0 0 0</acceleration>
          <wrench>0 0 0 0 0 0</wrench>
        </link>
      </model>
"""
end = """
      <model name='ground_plane'>
        <pose>0 0 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>0 0 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <light name='sun'>
        <pose>0 0 10 0 -0 0</pose>
      </light>
    </state>
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>{}</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
  </world>
</sdf>
"""

static_camera_pose = "-6.5 -1.5 6.4 0 0.54 0.44"
dynamic_camera_pose = "-9 -6 6 0 0.43 0.6"

x_spacing = 1
y_spacing = 0.5
z_spacing = 1

def generate_world(args):
    positions = generate_spawns(args.cnt, args.dynamic)
    with open(PATH, 'w') as f:
        f.write(start)
        for i in range(1, args.cnt + 1):
            name = "Boat_{}".format(i)
            if args.dynamic:
                f.write(dynamic_model.format(name))
            else:
                f.write(static_model.format(name))
        f.write(middle)
        for name, pos in positions:
            f.write(model_state.format(name, pos, pos))
        f.write(end.format(dynamic_camera_pose if args.dynamic else static_camera_pose))

def generate_spawns(cnt, is_dynamic):
    if cnt == 0: return []
    if is_dynamic:
        return generate_dynamic_test_spawns(cnt)
    else:
        return generate_static_test_spawns(cnt)
        
def generate_dynamic_test_spawns(cnt):
    ids = list(range(1, cnt+1))
    names = ["Boat_{}".format(x) for x in ids]

    if cnt >= 120:
      rows, columns = 3,2
    elif cnt >= 80:
      rows, columns = 2,2
    elif cnt >= 20:
      rows, columns = 2,1
    else:
      rows, columns = 1,1

    coordinates = []
    layer = 0
    while (len(coordinates) < cnt):
      for row in range(rows):
          for col in range(columns):
              if len(coordinates) >= cnt:
                  break
              x = col * x_spacing
              y = row * y_spacing
              z = layer * z_spacing + 1
              coordinates.append("{} {} {} 0 0 0".format(x, y, z))
      layer += 1

    return list(zip(names, coordinates))

def generate_static_test_spawns(cnt):
    ids = list(range(1, cnt+1))
    names = ["Boat_{}".format(x) for x in ids]

    num_rows = math.ceil(math.sqrt(cnt))
    num_cols = math.ceil(cnt / num_rows)

    coordinates = []
    for row in range(num_rows):
        for col in range(num_cols):
            if len(coordinates) == cnt:
                break
            x = col * x_spacing
            y = row * y_spacing
            coordinates.append("{} {} 0 0 0 0".format(x, y))

    return list(zip(names, coordinates))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("cnt", type=int, default=10)
    parser.add_argument("--dynamic", action='store_true')
    args = parser.parse_args()

    generate_world(args)


if __name__ == "__main__":
    main()