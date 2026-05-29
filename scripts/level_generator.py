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
        Model.count += 1
        name = f"{name}_{Model.count}"
        super().__init__(name, position, rotation, scale)
        self.uri = uri

    def toXMLStr(self):
        return f"""
            <include>
                <static> true </static>
                <name> {self.name} </name>
                {self.poseXMLStr()}
                {self.scaleXMLStr(is_primitive=False)}
                <uri> {self.uri} </uri>
            </include>
        """

class Wall(Object):
    count = 0 #To provide each wall in the gazebo simulation a unique name.

    def __init__(self, position=(0,0,0), aligned_y=False, wall_width=1.0, wall_height=1.0, wall_thickness=1.0):
        Wall.count += 1
        name = f"wall_{Wall.count}"
        scale = (wall_width, wall_thickness, wall_height) #Convention is to scale on xz axis and rotate around z.
        rotation = (0, 0, 90 if aligned_y else 0)
        super().__init__(name, position, rotation, scale)

    def toXMLStr(self):
        return f"""
            <model name="{self.name}">
                <static> true </static>
                {self.poseXMLStr()}
                <link name="{self.name + "-base-link"}">
                    <visual name="{self.name + "-visual"}">
                        <geometry>
                            <box>
                                {self.scaleXMLStr(is_primitive=True)}
                            </box>
                        </geometry>
                        <material>
                          <diffuse>0.2 0.2 0.2 1</diffuse>
                          <specular>0.1 0.1 0.1 1</specular>
                        </material>
                    </visual>
                </link>
            </model>
        """

class Floor(Object):
    count = 0 #To provide each floor in the gazebo simulation a unique name.

    def __init__(self, position=(0,0,0), floor_size=1.0):
        Floor.count += 1
        name = f"floor_{Floor.count}"
        scale = (floor_size, floor_size, 1.0)
        super().__init__(name, position, (0,0,0), scale)

    def toXMLStr(self):
        return f"""
            <model name="{self.name}">
                <static> true </static>
                {self.poseXMLStr()}
                <link name="{self.name + "-base-link"}">
                    <visual name="{self.name + "-visual"}">
                        <geometry>
                            <box>
                                {self.scaleXMLStr(is_primitive=True)}
                            </box>
                        </geometry>
                        <material>
                          <diffuse>0.2 0.2 0.2 1</diffuse>
                          <specular>0.1 0.1 0.1 1</specular>
                        </material>
                    </visual>
                </link>
            </model>
        """

class Ceiling(Object):
    count = 0 #To provide each ceiling in the gazebo simulation a unique name.

    def __init__(self, position=(0,0,0), ceiling_size=1.0):
        Ceiling.count += 1
        name = f"ceiling_{Ceiling.count}"
        scale=(ceiling_size, ceiling_size, 1.0)
        super().__init__(name, position, (0,0,0), scale)

    def toXMLStr(self):
        return f"""
            <model name="{self.name}">
                <static> true </static>
                {self.poseXMLStr()}
                <link name="{self.name + "-base-link"}">
                    <visual name="{self.name + "-visual"}">
                        <geometry>
                            <box>
                                {self.scaleXMLStr(is_primitive=True)}
                            </box>
                        </geometry>
                        <material>
                          <diffuse>0.2 0.2 0.2 1</diffuse>
                          <specular>0.1 0.1 0.1 1</specular>
                        </material>
                    </visual>
                    <light type="point" name="{self.name + "-light"}">
                        <pose> 0 0 -1 0 0 0 </pose>
                        <cast_shadows>false</cast_shadows>
                        <diffuse>0.8 0.8 0.8 1</diffuse>
                        <attenuation>
                            <range>10</range>
                            <constant>0.8</constant>
                            <linear>0.05</linear>
                            <quadratic>0.01</quadratic>
                        </attenuation> 
                    </light>
                </link>
            </model>
        """

class Box(Object):
    count = 0 #To provide each floor in the gazebo simulation a unique name.

    def __init__(self, position=(0,0,0), scale=(1,1,1)):
        Box.count += 1
        name = f"box_{Box.count}"
        super().__init__(name, position, (0,0,0), scale)

    def toXMLStr(self):
        return f"""
            <model name="{self.name}">
                <static> true </static>
                {self.poseXMLStr()}
                <link name="{self.name + "-base-link"}">
                    <visual name="{self.name + "-visual"}">
                        <geometry>
                            <box>
                                {self.scaleXMLStr(is_primitive=True)}
                            </box>
                        </geometry>
                        <material>
                          <diffuse>0.2 0.2 0.2 1</diffuse>
                          <specular>0.1 0.1 0.1 1</specular>
                        </material>
                    </visual>
                </link>
            </model>
        """

class AABB:
    def __init__(self, points):
        self.points = points
        self.min_point = np.min(points, axis=0)
        self.max_point = np.max(points, axis=0)
        self.translated_min = self.min_point
        self.translated_max = self.max_point

    def intersects(self, other):
        if self.min_point.shape != other.min_point.shape:
            raise ValueError("Dimension mismatch")
        return np.all(
            (self.translated_min <= other.translated_max) &
            (self.translated_max >= other.translated_min)
        )

    def setPosition(self, position):
        position = np.array(position)
        self.translated_min = self.min_point + position
        self.translated_max = self.max_point + position

    def setRotation(rotation):
        pass

    def getScale(self):
        return (self.max_point[0] - self.min_point[0], self.max_point[1] - self.min_point[1], self.max_point[2] - self.min_point[2])

import requests
import numpy as np
def loadVertices(configuration):
    models = configuration["floor_models"] + configuration["wall_models"]
    model_vertices = {}

    #Fetch the data from gazebo fuel model library and switch logic 
    #based on file type (only .obj and .dae files expected for now)
    for model in models:
        print(model)
        uri = model["uri"] + model["mesh_path"]
        data = requests.get(uri).content.decode()
        vertices = []
        if uri.endswith(".obj"):
            #Interpret data as an obj file.
            lines = data.splitlines()
            for line in lines:
                if line.startswith("v "):
                    _, x, y, z = line.split()
                    vertices.append(np.array([float(x), float(y), float(z)]))
            vertices = np.array(vertices)
        elif uri.endswith(".dae"):
            #Interpret data as a dae file.
            #TODO: handle .dae files.
            pass

        model_vertices.update({model["name"]: vertices})

    return model_vertices

def compileSDF(objects):
    objects_str = ""
    for object in objects:
        objects_str += object.toXMLStr()
    return world_template.replace("[INSERT]", objects_str)

import random
def generateFloorPlan(rows, columns, random_room_shape, growth_steps, seed): 
    room = [[0 for _ in range(columns)] for _ in range(rows)]

    if random_room_shape:
        #Idea: randomly stitch together square and rectangular cell groups to form a random room shape,
        #effectively "growing" the room.
        growth_patterns = [(2, 2), (2, 3), (3, 2), (1, 4)]
        growth_start_coords = (rows // 2, columns // 2)
        active_cell_coords = [growth_start_coords]
        chosen_cell_coords = []
        #Begin random growth.
        for i in range(growth_steps):
            pattern = random.choice(growth_patterns)
            random_active_cell = random.choice(active_cell_coords)
            chosen_cell_coords.append(random_active_cell)
            for row in range(pattern[0]):
                for column in range(pattern[1]):
                    true_row = random_active_cell[0] + row
                    true_column = random_active_cell[1] + column
                    
                    if (0 <= true_row < rows) and (0 <= true_column < columns):
                        room[true_row][true_column] = 1
                        active_cell_coords.append((true_row, true_column))

    else:
        for row in range(rows):
            for column in range(columns):
                room[row][column] = 1

    return room

def placeModels(configuration, room, model_vertices):
    #NOTE: Idea, map multiple rectangular kernels to random parts of the room and assign model groups to them for spawning.
    #Each mapped kernel can have either a uniform grid item layout or a randomly scattered item layout.
    region_kernels = [(2, 3), (2,2), (3, 2)]
    iterations = 10
   
    #To store placed models.
    model_objects = []

    #Obtain set of cells that are open.
    rows = configuration["max_cell_rows"]
    columns = configuration["max_cell_columns"]
    open_cell_coords = []
    for row in range(rows):
        for column in range(columns):
            if room[row][column]:
                open_cell_coords.append((row, column))

    region_AABBs = []
    cell_size = configuration["cell_size"]
    ceiling_height = configuration["ceiling_height"]
    for i in range(iterations):
        chosen_cell_coord = random.choice(open_cell_coords)
        chosen_kernel = random.choice(region_kernels)

        #TODO:Kernel must be clipped if it breaches walls.


        #Convert to world space coordinates.
        chosen_cell_coord = ((chosen_cell_coord[0] - 0.5 * rows) * cell_size, (chosen_cell_coord[1] - 0.5 * columns) * cell_size, 0)
        chosen_kernel = (chosen_kernel[0] * cell_size, chosen_kernel[1] * cell_size)
        region_corners = [
                chosen_cell_coord,
                (
                    chosen_cell_coord[0] + chosen_kernel[0], 
                    chosen_cell_coord[1] + chosen_kernel[1],
                    ceiling_height
                )
        ]
        region_AABB = AABB(region_corners)

        #Check if it intersects any previously defined regions.
        flag = False
        for aabb in region_AABBs:
            if region_AABB.intersects(aabb): 
                flag = True
                break
       
        #if flag: continue

        region_AABBs.append(region_AABB)

        #Select either a uniform or random layout within this particular region. 
        #TODO: add support for uniform layout

        density = 0.3
        max_models_per_region = 10
        temp_model = configuration["floor_models"][0]
        region_model_objects = []
        region_model_AABBs = []
        for j in range(max_models_per_region):
            rand_angle = random.uniform(0, 360)
            rand_position = (
                chosen_cell_coord[0] + random.uniform(0, chosen_kernel[0]),
                chosen_cell_coord[1] + random.uniform(0, chosen_kernel[1]),
                0
            )
            
            #Check if model intersects region bounds or bounds of any other models in the region
            model_AABB = AABB(model_vertices[temp_model["name"]])
            rand_position = (rand_position[0], rand_position[1], model_AABB.getScale()[2] / 2 + 0.0001)
            model_AABB.setPosition(rand_position)            
            
            #if model_AABB.intersects(region_AABB): continue

            flag = False
            for aabb in region_model_AABBs:
                if model_AABB.intersects(aabb):
                    flag = True
                    print("Couldnt place object")
                    break

            if flag: continue

            region_model_AABBs.append(model_AABB)

            #Place model in the scene.
            #Getting scale of object to place it ontop of floor
            region_model_objects.append(Box(position=rand_position, scale=model_AABB.getScale()))
            #region_model_objects.append(Model(temp_model["name"], position=rand_position, uri=temp_model["uri"]))
        #Now dump all region level models into the global models list.
        model_objects += region_model_objects

    return model_objects

def generateLevel(configuration, seed, model_vertices):
    rows = configuration["max_cell_rows"]
    columns = configuration["max_cell_columns"]
    cell_size = configuration["cell_size"]
    ceiling_height = configuration["ceiling_height"]
    wall_thickness = configuration["wall_thickness"]
   
    random.seed(seed)
    room = generateFloorPlan(rows, columns, configuration["random_room_shape"], configuration["random_growth_steps"], seed)
    
    objects = []

    #Wrap room in walls, ceilings, and floors. Condition is if neighbor cell is nonexistent or invalid.
    for row in range(len(room)):
        for column in range(len(room[0])):
            if room[row][column] == 0:
                #Nonexistent room, skip processing.
                continue
           
            #Place floor and ceiling for this cell in the room.
            cell_center = ((row - 0.5 * rows) * cell_size, (column - 0.5 * columns) * cell_size, 0)
            floor_position = (cell_center[0], cell_center[1], -0.5)
            floor = Floor(position=floor_position, floor_size=cell_size)
            ceiling_position = (cell_center[0], cell_center[1], ceiling_height + 0.5)
            ceiling = Ceiling(position=ceiling_position, ceiling_size=cell_size) 

            objects.append(floor)
            objects.append(ceiling)

            #Wall pass. 
            nc = [(-1, 0), (1, 0), (0, 1), (0, -1)] #neighbor coordinate offsets, North, South, East, West.
            for c in nc:
                neighbor = (row + c[0], column + c[1])
                if not (0 <= neighbor[0] < rows) or not (0 <= neighbor[1] < columns) or room[neighbor[0]][neighbor[1]] == 0:
                    #Walls are placed at half a cell size plus the half-thickness of the wall. 
                             
                    wall_position = (
                        cell_center[0] + c[0] * cell_size * 0.5 + c[0] * wall_thickness * 0.5,
                        cell_center[1] + c[1] * cell_size * 0.5 + c[1] * wall_thickness * 0.5,
                        ceiling_height * 0.5
                    )

                    wall = Wall(
                        position=wall_position, 
                        aligned_y=not (c[0] == 0), 
                        wall_width=cell_size, 
                        wall_height=ceiling_height, 
                        wall_thickness=wall_thickness
                    )

                    objects.append(wall) 
    
    objects += placeModels(configuration, room, model_vertices)
    return objects
      

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
   
    #Load all model mesh vertices that will be needed during generation.
    model_vertices = loadVertices(configuration)

    #Generate the level, convert to sdf format, and then write to the destination path.
    level_sdf = compileSDF(generateLevel(configuration, seed, model_vertices))
    with open(destination_path, "w") as f:
        f.write(level_sdf)

if __name__ == "__main__":
    main()


