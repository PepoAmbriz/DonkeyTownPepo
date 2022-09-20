base_sdf = """
<?xml version='1.0'?>
<sdf version='1.5'>
  <model name='aruco_visual_marker_{mid}'>

    <link name='marker'>
      <pose frame=''>0 0 0 0 0 0</pose>
      <visual name='visual'>
        <geometry>
          <box>
            <size>0.14 0.14 1e-04</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>model://aruco_visual_marker_{mid}/materials/scripts</uri>
            <uri>model://aruco_visual_marker_{mid}/materials/textures</uri>
            <name>ArucoVisualMarker{mid}/Marker</name>
          </script>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0 0 0 1</specular>
          <emissive>1 1 1 0</emissive>
          <shader type='vertex'>
            <normal_map>__default__</normal_map>
          </shader>
        </material>
        <pose frame=''>0 0 0 0 -0 0</pose>
        <cast_shadows>1</cast_shadows>
        <transparency>0</transparency>
      </visual>
      <collision name='collision'>
        <laser_retro>0</laser_retro>
        <max_contacts>10</max_contacts>
        <pose frame=''>0 0 0 0 -0 0</pose>
        <geometry>
          <box>
            <size>0.14 0.14 1e-04</size>
          </box>
        </geometry>
      </collision>
    </link>

    <link name='marker_pad'>
      <pose frame='marker'>0 0 -1e-04 0 -0 0</pose>
      <visual name='visual'>
        <geometry>
          <box>
            <size>0.14 0.14 1e-04</size>
          </box>
        </geometry>
        <material>
          <lighting>0</lighting>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/White</name>
          </script>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
          <specular>0.01 0.01 0.01 1</specular>
          <emissive>1 1 1 0</emissive>
          <shader type='vertex'>
            <normal_map>__default__</normal_map>
          </shader>
        </material>
        <cast_shadows>1</cast_shadows>
        <transparency>0</transparency>
      </visual>
      <collision name='collision'>
        <laser_retro>0</laser_retro>
        <max_contacts>10</max_contacts>
        <pose frame=''>0 0 0 0 -0 0</pose>
        <geometry>
          <box>
            <size>0.14 0.14 1e-04</size>
          </box>
        </geometry>
      </collision>
    </link>

    <joint name='marker_JOINT_marker_pad' type='revolute'>
      <parent>marker</parent>
      <child>marker_pad</child>
      <pose frame=''>0 0 0 0 -0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <upper>0</upper>
          <lower>0</lower>
        </limit>
      </axis>
    </joint>

    <static>1</static>
    <allow_auto_disable>1</allow_auto_disable>

  </model>
</sdf>
"""

base_config = """
<?xml version="1.0" ?>
<model>
    <name>aruco_visual_marker_{mid}</name>
    <version>1.0</version>
    <sdf version="1.6">model.sdf</sdf>
    <author>
        <name></name>
        <email></email>
    </author>
    <description></description>
</model>
"""

mscript = """ 
material ArucoVisualMarker{mid}/Marker
{{
  technique
  {{
    pass
    {{
      texture_unit
      {{
        texture aruco_mark_{mid}.png
      }}
    }}
  }}
}}
"""
base_world = """
<?xml version="1.0"?>
<sdf version="1.4">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    {models}
  </world>
</sdf>
"""

world_mark_parser = """
    <include>
        <uri>model://aruco_visual_marker_{mid}</uri>
        <name>aruco_marker_{mid}</name>
        <pose>{cord[0]} {cord[1]} 0 0 0 0</pose>
    </include>
"""