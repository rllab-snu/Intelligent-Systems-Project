import math
import rospkg
import argparse


#######################################################
#THIS CODE IS FOR VISUALIZING YOUR GENERATED WAYPOINTS#
#######################################################




parser = argparse.ArgumentParser()
parser.add_argument(
    "--track_id",
    type=int,
    required=True,
    help="which track image to construct map",
)




args = parser.parse_args()

def get_length(X, Y):
    return math.sqrt((X[0] - Y[0]) * (X[0] - Y[0]) + (X[1] - Y[1]) * (X[1] - Y[1]))

def get_wall(wall_name, X, Y):
    l = get_length(X,Y)
    C = [(X[0] + Y[0]) / 2, (X[1] + Y[1]) / 2]
    yaw = math.atan2(X[1] - Y[1], X[0] - Y[0])
    return """<link name='%s'>
                <collision name='%s_Collision'>
                <geometry>
                    <box>
                    <size>%lf 0.15 2.5</size>
                    </box>
                </geometry>
                <pose frame=''>0 0 1.25 0 -0 0</pose>
                <max_contacts>10</max_contacts>
                <surface>
                    <contact>
                    <ode/>
                    </contact>
                    <bounce/>
                    <friction>
                    <torsional>
                        <ode/>
                    </torsional>
                    <ode/>
                    </friction>
                </surface>
                </collision>
                <visual name='%s_Visual'>
                <pose frame=''>0 0 1.25 0 -0 0</pose>
                <geometry>
                    <box>
                    <size>%lf 0.15 2.5</size>
                    </box>
                </geometry>
                <material>
                    <script>
                    <uri>file://media/materials/scripts/gazebo.material</uri>
                    <name>Gazebo/Grey</name>
                    </script>
                    <ambient>1 1 1 1</ambient>
                </material>
                <meta>
                    <layer>0</layer>
                </meta>
                </visual>
                <pose frame=''>%lf %lf 0 0 -0 %lf</pose>
                <self_collide>0</self_collide>
                <enable_wind>0</enable_wind>
                <kinematic>0</kinematic>
            </link>
            """ %(wall_name, wall_name, l, wall_name, l, C[0], C[1], yaw)

def export_world_file(file_path, waypoint_path, save_path):
  lines = []
  with open(file_path, "r") as f:
    for line in f:
        lines.append(line.split())
  N = len(lines)

  waypoints = []
  with open(waypoint_path, "r") as f_wpt:
    for line in f_wpt:
        waypoints.append(line.split())
  num_wpt = len(waypoints)

  A = []
  B = []
  for i in range(N):
    A.append([float(lines[i][1]), float(lines[i][0])])
    B.append([float(lines[i][3]), float(lines[i][2])])

  world_file = """<sdf version='1.6'>
    <world name='default'>
      <light name='sun' type='directional'>
        <cast_shadows>1</cast_shadows>
        <pose frame=''>0 0 10 0 -0 0</pose>
        <diffuse>0.8 0.8 0.8 1</diffuse>
        <specular>0.2 0.2 0.2 1</specular>
        <attenuation>
          <range>1000</range>
          <constant>0.9</constant>
          <linear>0.01</linear>
          <quadratic>0.001</quadratic>
        </attenuation>
        <direction>-0.5 0.1 -0.9</direction>
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
              <friction>
                <ode>
                  <mu>100</mu>
                  <mu2>50</mu2>
                </ode>
                <torsional>
                  <ode/>
                </torsional>
              </friction>
              <contact>
                <ode/>
              </contact>
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
      <physics name='default_physics' default='0' type='ode'>
        <max_step_size>0.001</max_step_size>
        <real_time_factor>1</real_time_factor>
        <real_time_update_rate>1000</real_time_update_rate>
      </physics>
      <scene>
        <ambient>0.4 0.4 0.4 1</ambient>
        <background>0.7 0.7 0.7 1</background>
        <shadows>1</shadows>
      </scene>
      <wind/>
      <spherical_coordinates>
        <surface_model>EARTH_WGS84</surface_model>
        <latitude_deg>0</latitude_deg>
        <longitude_deg>0</longitude_deg>
        <elevation>0</elevation>
        <heading_deg>0</heading_deg>
      </spherical_coordinates>
      <model name='track2'>
        <pose frame=''>0 0 0 0 -0 0</pose>
        """
  for i in range(N):
    world_file += get_wall("wall_" + str(i).zfill(3), A[i], B[i])

  world_file += """<static>1</static>
      </model>
      <gui fullscreen='0'>
        <camera name='user_camera'>
          <pose frame=''>-9.88167 -5.87841 72.8661 3.14159 1.57079 3.14159</pose>
          <view_controller>orbit</view_controller>
          <projection_type>perspective</projection_type>
        </camera>
      </gui>
  """
  radius = float(0.05)
  for i in range(num_wpt):
    world_file += """    <model name='unit_sphere_0_%d'>
      <pose frame=''>%lf %lf %lf 0 -0 0</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1</iyy>
            <iyz>0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
        <collision name='collision'>
          <geometry>
            <sphere>
              <radius>%lf</radius>
            </sphere>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <sphere>
              <radius>%lf</radius>
            </sphere>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
        <gravity>1</gravity>
      </link>
    </model>
    """ %(i, float(waypoints[i][0]), float(waypoints[i][1]), radius, radius, radius)
    print(i, float(waypoints[i][0]), float(waypoints[i][1]))
  
  world_file += """    </world>
  </sdf>
"""


  save_file = open(save_path, "w")
  save_file.write(world_file)
  save_file.close()



world_file_name = "track_{}".format(args.track_id)  # Track ID
file_path = rospkg.RosPack().get_path("sim2real") + "/worlds/world_map_scaled/" + world_file_name + "_scaled.txt" # Get the scaled world map file
waypoint_file_name = "track_{}_aug_waypoints".format(args.track_id) 
waypoint_path = rospkg.RosPack().get_path("sim2real") + "/worlds/waypoints_augmented/" + waypoint_file_name + ".txt" # Get augmented waypoint file
world_file_path = rospkg.RosPack().get_path("sim2real") + "/worlds/worlds_with_waypoints/" + world_file_name + "_waypoints.world" # Output directory
export_world_file(file_path, waypoint_path, world_file_path)
