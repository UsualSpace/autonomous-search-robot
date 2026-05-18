#!/usr/bin/env python3

#Filename: level_generator.py
#Programmer: Abdurrahman Alyajouri
#Date: 5/10/2026
#Purpose: To provide a simple interface to procedurally generate levels with rooms for robot simulation in gazebosim.

PI = 3.14159265359
world_template = """
<?xml version="1.0" ?>
<sdf version="1.8">
        <world name="RG1">
                <physics name="1ms" type="ignored">
                        <max_step_size>0.001</max_step_size>
                        <real_time_factor>1.0</real_time_factor>
                </physics>
                <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
                <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
                <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
                <plugin filename="MinimalScene" name="3D View">
                        <gz-gui>
                                <title>3D View</title>
                                <property type="bool" key="showTitleBar">false</property>
                                <property type="string" key="state">docked</property>
                        </gz-gui>
                        <engine>ogre2</engine>
                        <scene>scene</scene>
                        <ambient_light>0.4 0.4 0.4</ambient_light>
                        <background_color>0.8 0.8 0.8</background_color>
                        <camera_pose>-6 0 6 0 0.5 0</camera_pose>
                        <camera_clip>
                                <near>0.25</near>
                                <far>25000</far>
                        </camera_clip>
                </plugin>
                <plugin filename="GzSceneManager" name="Scene Manager">
                        <gz-gui>
                                <property key="resizable" type="bool">false</property>
                                <property key="width" type="double">5</property>
                                <property key="height" type="double">5</property>
                                <property key="state" type="string">floating</property>
                                <property key="showTitleBar" type="bool">false</property>
                        </gz-gui>
                </plugin>

                
                [INSERT]
                

        </world>
</sdf>

"""

#NOTE: Input angles are defined in degrees for this script, and are converted to radians internally.
#NOTE: Positions and scales are in meters.

class Object:
    def __init__(self, name, position, rotation, scale):
        self.position = position
        to_rads = PI / 180.0
        self.rotation = (rotation[0] * to_rads, rotation[1] * to_rads, rotation[2] * to_rads)
        self.scale = scale
        self.name = name

    def poseXMLStr(self):
        return f"""
            <pose> 
                {self.position[0]} {self.position[1]} {self.position[2]} {self.rotation[0]} {self.rotation[1]} {self.rotation[2]}
            </pose>
        """

    def scaleXMLStr(self, is_primitive):
        if is_primitive:
            return f"<size> {self.scale[0]} {self.scale[1]} {self.scale[2]} </size>"
        else:
            return f"<scale> {self.scale[0]} {self.scale[1]} {self.scale[2]} </scale>"

    def toXMLStr(self):
        pass

class Model(Object):
    count = 0 #To provide each model in the gazebo simulation a unique name.

    def __init__(self, name="model", position=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1), uri=""):
        self.count += 1
        name = f"{name}_{self.count}"
        super().__init__(name, position, rotation, scale)
        self.uri = uri

    def toXMLStr(self):
        return f"""
            <include>
                <name> {self.name} </name>
                {self.poseXMLStr()}
                {self.scaleXMLStr(is_primitive=False)}
                <uri> {self.uri} </uri>
            </include>
        """

class Wall(Object):
    count = 0 #To provide each wall in the gazebo simulation a unique name.

    def __init__(self, position(0,0,0), aligned_y=False, wall_width=1.0, wall_height=1.0, wall_thickness=1.0):
        self.count += 1
        name = f"wall_{self.count}"
        scale = (wall_width, wall_thickness, wall_height) #Convention is to scale on xz axis and rotate around z.
        rotation = (0, 0, 90 if aligned_y else 0)
        super().__init__(name, position, rotation, scale)

    def toXMLStr(self):
        return f"""
            <model name="{self.name}">
                <link name="{self.name + "-base-link"}">
                    {self.poseXMLStr()}
                    <visual name="{self.name + "-visual"}">
                        <geometry>
                            <box>
                                {self.scaleXMLStr(is_primitive=True)}
                            </box>
                        </geometry>
                    </visual>
                </link>
            </model>
        """


class Floor(Object):
    count = 0 #To provide each floor in the gazebo simulation a unique name.

    def __init__(self, position=(0,0,0), floor_size=1.0):
        self.count += 1
        name = f"floor_{self.count}"
        scale = (floor_size, floor_size, 1.0)
        super().__init__(name, position, (0,0,0), scale)

    def toXMLStr(self):
        return f"""
            <model name="{self.name}">
                <link name="{self.name + "-base-link"}">
                    {self.poseXMLStr()}
                    <visual name="{self.name + "-visual"}">
                        <geometry>
                            <box>
                                {self.scaleXMLStr(is_primitive=True)}
                            </box>
                        </geometry>
                    </visual>
                </link>
            </model>
        """

class Ceiling(Object):
    count = 0 #To provide each ceiling in the gazebo simulation a unique name.

    def __init__(self, position=(0,0,0), ceiling_size=1.0):
        self.count += 1
        name = f"ceiling_{self.count}"
        scale=(ceiling_size, ceiling_size, 1.0)
        super().__init__(name, position, (0,0,0), scale)

    def toXMLStr(self):
        return f"""
            <model name="{self.name}">
                <link name="{self.name + "-base-link"}">
                    {self.poseXMLStr()}
                    <visual name="{self.name + "-visual"}">
                        <geometry>
                            <box>
                                {self.scaleXMLStr(is_primitive=True)}
                            </box>
                        </geometry>
                    </visual>
                </link>
                <light type="spot" name="{self.name + "-light"}">
                    <cast_shadows>true</cast_shadows>
                    <diffuse>0.8 0.8 0.8 1</diffuse>
                    <specular>0.2 0.2 0.2 1</specular>
                    <attenuation>
                        <range>1000</range>
                        <constant>0.9</constant>
                        <linear>0.01</linear>
                        <quadratic>0.001</quadratic>
                    </attenuation>
                    <direction>0 0 -1</direction>
                </light>
            </model>
        """


def compileSDF(objects):
    objects_str = ""
    for object in objects:
        objects_str += object.toXMLStr()
    return world_template.replace("[INSERT]", objects_str)

def generateLevel(configuration, seed):
    rows = configuration["max_cell_rows"]
    columns = configuration["max_cell_columns"]
    room = [[0 for _ in range(columns)] for _ in range(rows)]
    objects = []

    if configuration["random_room_shape"]:
        pass
    else:
        for row in range(len(room)):
            for column in range(len(room[0])):
                room[row][column] = 1

    #Wrap room in walls, ceilings, and floors. Condition is if neighbor cell is nonexistent or invalid.
    for row in range(len(room)):
        for column in range(len(room[0])):
            if room[row][column] == 0:
                #Nonexistent room, skip processing.
                continue
           

            room_center = ro

            #Now this is a valid room, but we need to check neighbor existence/validity for placing walls.
            nc = [(-1, 0), (1, 0), (0, 1), (0, -1)] #neighbor coordinates, North, South, East, West.
            direction = True
            for c in nc:
                neighbor = (row + c[0], column + c[1])
                if not (0 < neighbor[0] < rows) or not (0 < neighbor[1] < columns) or room[neighbor[0]][neighbor[1]] == 0:
                    wall = Wall(
      

#MAIN CODE
import sys, json
def main():
    help_message = "Try: ./level_generator.py [config file path (.json)] [destination_path (.sdf)] [seed (integer, 1 by default, so this argument is optional)]"

    if len(sys.argv) < 3: 
        print("Bad arguments. " + help_message)
        return
    
    configuration_path = sys.argv[1]
    destination_path = sys.argv[2]

    #In case seed was user defined, try to convert to an integer.
    try:
        seed = 1 if len(sys.argv) < 4 else int(sys.argv[3])
    except:
        print("Seed argument must be an integer. " + help_message)
        return

    if not configuration_path.endswith(".json"):
        print("Configuration path must be a json file with the .json extension. " + help_message)
        return

    #I wonder if this is poor design...
    if not destination_path.endswith(".sdf"):
        destination_path = destination_path + ".sdf"
   
    #Obtain the configuration as a python dictionary.
    with open(configuration_path, "r") as f:
        json_str = f.read()
        configuration = json.loads(json_str)
    
    #Generate the level, convert to sdf format, and then write to the destination path.
    level_sdf = compileSDF(generateLevel(configuration, seed))
    with open(destination_path, "w") as f:
        f.write(level_sdf)

if __name__ == "__main__":
    main()


