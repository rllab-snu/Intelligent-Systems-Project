import math
import rospkg
import argparse

parser = argparse.ArgumentParser()
parser.add_argument(
    "--track_id",
    type=int,
    required=True,
    help="which track image to construct map",
)
parser.add_argument(
    "--scale",
    type=float,
    required=False,
    default=1.0,
    help="scale for map and waypoints",
)


####################################################
#THIS CODE IS TO AUGMENT YOUR HANDCRAFTED WAYPOINTS#
####################################################




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

def export_world_file(file_path, scaled_wall_path, save_path, scale):
  lines = []
  with open(file_path, "r") as f:
    for line in f:
        lines.append(line.split())
  N = len(lines) # int(lines[0][0])
  A = []
  B = []
  for i in range(N):
    A.append([scale*float(lines[i][1]), scale*float(lines[i][0])])
    B.append([scale*float(lines[i][3]), scale*float(lines[i][2])])
  f_scale = open(scaled_wall_path, 'w')
  for i in range(N-1):
     txt = str(scale*float(lines[i][0])) + " " + str(scale*float(lines[i][1])) + " " + str(scale*float(lines[i][2])) + " " + str(scale*float(lines[i][3])) + "\n"
     f_scale.write(txt)
  for i in range(N-1,N):
     txt = str(scale*float(lines[i][0])) + " " + str(scale*float(lines[i][1])) + " " + str(scale*float(lines[i][2])) + " " + str(scale*float(lines[i][3]))
     f_scale.write(txt)
  f_scale.close()
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
    </world>
  </sdf>
  """
  save_file = open(save_path, "w")
  save_file.write(world_file)
  save_file.close()


scale = args.scale# SINCE THE WALL BUILT USING THE RAW TXT FILE GENERATED FROM PYQT CAN BE TOO SMALL OR TOO BIG, WE MUST RESCALE IT.
world_file_name = "track_{}".format(args.track_id)
file_path = rospkg.RosPack().get_path("sim2real") + "/worlds/world_map/" + world_file_name + ".txt" # GET TXT FROM POINTS GENERATED FROM PYQT
world_file_path = rospkg.RosPack().get_path("sim2real") + "/worlds/worlds/" + world_file_name + ".world" # OUTPUT GENERATED WORLD

scaled_wall_path = rospkg.RosPack().get_path("sim2real") + "/worlds/world_map_scaled/" + world_file_name + "_scaled.txt" # OUTPUT SCALED WORLD MAP DATA 
export_world_file(file_path, scaled_wall_path, world_file_path, scale)
