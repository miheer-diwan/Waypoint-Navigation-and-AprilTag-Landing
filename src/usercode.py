import numpy as np
import cv2
import bpy
import os
import copy
import apriltag

class state_machine:
    def __init__(self):
        # User code executes every dt time in seconds
        self.dt = 0.050

        # location of each tag
        self.waypoints = [
            [-1.7,      0.8,    1.5],
            [-1.2,     -1.6,      1.5],
            [0.9,     0.7,    1.5]
        ]

        # Current waypoint
        self.currentWP = [0,    -0.2,    1.5]


        # Current waypoint index
        self.current_wp_index = 1

        # Set a distance threshold for waypoint reaching
        self.distance_threshold = 0.1

        # Initialize Apriltag detector
        self.detector = apriltag.Detector()

        # Landing and takeoff heights
        self.landing_height = 0.1
        self.takeoff_height = 1.5

        # Variable to track landing state
        self.landing = False
        self.index = 0

        self.landed = False


    def step(self, time, currpos):
        """
        Input: time, current position in blender frame (x, y, z)
        Output: desired position
            Si unit unless specified otherwise

            FILL YOUR CODE HERE!
        """

        if self.landing == False:
            
            
            xyz_desired = self.waypoints[self.index]

            distance_to_wp = np.linalg.norm(np.array(xyz_desired) - np.array(currpos))

            if distance_to_wp < 0.05 and self.index == 0:

                image = self.fetchLatestImage()
                tags = self.detector.detect(image)

                # if len(tags) > 0:
                #     print("Apriltag detected:", tags[0].tag_id)

                if tags[0].tag_id == 4:
                    self.landing = True

            elif distance_to_wp < 0.05 and self.index == 1:
                self.index = 2

            # self.index = (self.index + 1)

        else:

            if self.landed == False:
                xyz_desired = [currpos[0],currpos[1],0.1]
                dist = np.linalg.norm(np.array(xyz_desired) - np.array(currpos))

                if dist < 0.05:
                    self.landed = True

                # self.landed = False
            
            elif self.landed == True:
                xyz_desired = [currpos[0],currpos[1],1.5]

                dist = np.linalg.norm(np.array(xyz_desired) - np.array(currpos))

                if dist < 0.05:
                    self.landed = False
                    self.landing = False
                    self.index += 1
                                    

        return xyz_desired
            

    def fetchLatestImage(self):
        # Fetch image - renders the camera, saves the rendered image to a file and reads from it. 
        path_dir = bpy.data.scenes["Scene"].node_tree.nodes["File Output"].base_path

        # Render Drone Camera
        cam = bpy.data.objects['DownCam']    
        bpy.context.scene.camera = cam
        bpy.context.scene.render.filepath = os.path.join(path_dir, 'DownCam_latest.png')
        bpy.ops.render.render(write_still=True)

        return cv2.imread(bpy.context.scene.render.filepath,cv2.IMREAD_GRAYSCALE)
    
    def executeLanding(self, currpos):
        # Land by maintaining the XY location and setting Z location to landing height
        landing_waypoint = [currpos[0], currpos[1], self.landing_height]
        return landing_waypoint