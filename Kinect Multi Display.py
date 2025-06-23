from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

import ctypes
import _ctypes
import pygame
import sys
import serial
from time import sleep
import threading
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.backends.backend_agg as agg

matplotlib.use("Agg")


if sys.hexversion >= 0x03000000:
    import _thread as thread
else:
    import thread

# colors for drawing different bodies 
SKELETON_COLORS = [pygame.color.THECOLORS["red"], 
                  pygame.color.THECOLORS["blue"], 
                  pygame.color.THECOLORS["green"], 
                  pygame.color.THECOLORS["orange"], 
                  pygame.color.THECOLORS["purple"], 
                  pygame.color.THECOLORS["yellow"], 
                  pygame.color.THECOLORS["violet"]]

def ser_conn(name): #connect to the arduino over serial. runs once at the beginning
    ser = serial.Serial(name, 9600, timeout=0.01, write_timeout=0.1)
    ser.reset_input_buffer()
    sleep(3)
    return ser

'''
|=================================|
|                                 |
|    COM Port Number Goes Here    |
|       (e.g. port = "COM3")      |
|                                 |
|=================================|
'''
port = None
#port = "COM3"
if port:
    serial_connection = ser_conn(port)
else: 
    serial_connection = None

class BodyGameRuntime(object):
    def __init__(self):
        pygame.init()

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Set the width and height of the screen [width, height]
        self._infoObject = pygame.display.Info()
        self._screen = pygame.display.set_mode((self._infoObject.current_w >> 1, self._infoObject.current_h >> 1), 
                                               pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE, 32)

        pygame.display.set_caption("Kinect Visualization")

        # Loop until the user clicks the close button.
        self._done = False

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Kinect runtime object, we want only color and body frames 
        self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body | PyKinectV2.FrameSourceTypes_Depth | PyKinectV2.FrameSourceTypes_Infrared)

        # back buffer surface for getting Kinect color frames, 32bit color, width and height equal to the Kinect color frame size
        self._frame_surface = pygame.Surface((self._kinect.color_frame_desc.Width, self._kinect.color_frame_desc.Height), pygame.SRCALPHA, 32)

        self._ir_frame_surface = pygame.Surface((self._kinect.infrared_frame_desc.Width, self._kinect.infrared_frame_desc.Height), 0, 24)

        self._depth_frame_surface = pygame.Surface((self._kinect.depth_frame_desc.Width, self._kinect.depth_frame_desc.Height), 0, 16)

        # here we will store skeleton data 
        self._bodies = None

        self._fig = plt.figure()
        self._ax = plt.axes([0,0,1,1],frameon=False)
        
        self.peoplecount = 0

        global serial_connection
        self.serial = serial_connection


    def draw_body_bone(self, joints, jointPoints, color, joint0, joint1):
        joint0State = joints[joint0].TrackingState;
        joint1State = joints[joint1].TrackingState;
        thickness = 8

        # both joints are not tracked
        if (joint0State == PyKinectV2.TrackingState_NotTracked) or (joint1State == PyKinectV2.TrackingState_NotTracked): 
            return

        # both joints are not *really* tracked
        if (joint0State == PyKinectV2.TrackingState_Inferred) and (joint1State == PyKinectV2.TrackingState_Inferred):
            return

        # only one joint is sus
        if (joint0State == PyKinectV2.TrackingState_Inferred) or (joint1State == PyKinectV2.TrackingState_Inferred):
            color = pygame.color.THECOLORS['grey']
            thickness = 4

        # ok, at least one is good 
        start = (jointPoints[joint0].x+70, jointPoints[joint0].y+100)
        end = (jointPoints[joint1].x+70, jointPoints[joint1].y+100)

        try:
            pygame.draw.line(self._frame_surface, color, start, end, thickness)
        except: # need to catch it due to possible invalid positions (with inf)
            pass


    def draw_depth(self, depth):
        depth = depth[::3,::3]
        ysize, xsize = np.shape(depth)
        X, Y = np.meshgrid(range(xsize),range(ysize))
        #print(*[np.shape(thingy) for thingy in [X,Y,depth]])
        self._ax.clear()
        self._ax.pcolormesh(X,-Y,depth,vmin=0, vmax=4000, cmap=matplotlib.colormaps['jet'],shading='nearest')
        plt.axis('off')

        canvas = agg.FigureCanvasAgg(self._fig)
        canvas.draw()
        renderer = canvas.get_renderer()
        raw_data = renderer.buffer_rgba()
        size = canvas.get_width_height()
        self._depth_frame_surface = pygame.image.frombuffer(raw_data, size, "RGBA")

    '''
    def draw_depth(self, depth, xstep=8, ystep=8):
        cmap = matplotlib.colormaps['gist_rainbow']
        sizey, sizex = np.shape(depth)
        depth = depth[::ystep,::xstep]
        #print(np.shape(depth))
        frame_ht = self._frame_surface.get_height()
        frame_wd = self._frame_surface.get_width()
        #print(frame_wd, frame_ht)
        #ysize, xsize = np.shape(depth)
        for iy, row in enumerate(depth):
            for ix, dep in enumerate(row):
                xcoord = (((ix*xstep/sizex)-0.5)*frame_ht*sizex/sizey*1.15)+frame_wd/2+30
                ycoord = (((iy*ystep/sizey)-0.5)*frame_ht)+frame_ht/2
                pygame.draw.circle(self._frame_surface, pygame.Color([i * 255 for i in cmap(dep/2000)]), (xcoord, ycoord), 3)
    '''

    def draw_body(self, joints, jointPoints, color, hands=(0,0)):
        #print(hands)
        hands = (0,0)
        handcolors = [None, None,(255,0,0,50),(0,255,0,50),(0,0,255,50)]
        #print("bones!")
        # Torso
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Head, PyKinectV2.JointType_Neck);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Neck, PyKinectV2.JointType_SpineShoulder);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_SpineMid);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineMid, PyKinectV2.JointType_SpineBase);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_ShoulderRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_ShoulderLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipLeft);
    
        # Right Arm    
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderRight, PyKinectV2.JointType_ElbowRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowRight, PyKinectV2.JointType_WristRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_HandRight);
        if hands[1] > 1:
            pygame.draw.circle(self._frame_surface, handcolors[hands[1]], (jointPoints[PyKinectV2.JointType_HandRight].x,jointPoints[PyKinectV2.JointType_HandRight].y), 80)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandRight, PyKinectV2.JointType_HandTipRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_ThumbRight);

        # Left Arm
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderLeft, PyKinectV2.JointType_ElbowLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowLeft, PyKinectV2.JointType_WristLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_HandLeft);
        if hands[0] > 1:
            pygame.draw.circle(self._frame_surface, handcolors[hands[0]], (jointPoints[PyKinectV2.JointType_HandLeft].x,jointPoints[PyKinectV2.JointType_HandLeft].y), 80)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandLeft, PyKinectV2.JointType_HandTipLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_ThumbLeft);

        # Right Leg
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HipRight, PyKinectV2.JointType_KneeRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_KneeRight, PyKinectV2.JointType_AnkleRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_AnkleRight, PyKinectV2.JointType_FootRight);

        # Left Leg
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HipLeft, PyKinectV2.JointType_KneeLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_KneeLeft, PyKinectV2.JointType_AnkleLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_AnkleLeft, PyKinectV2.JointType_FootLeft);


    def draw_color_frame(self, frame, target_surface):
        target_surface.lock()
        address = self._kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame.ctypes.data, frame.size)
        del address
        target_surface.unlock()

    def draw_infrared_frame(self, frame, target_surface):
        if frame is None:  # some usb hub do not provide the infrared image. it works with Kinect studio though
            return
        target_surface.lock()
        f8=np.uint8(frame.clip(1,4000)/16.)
        frame8bit=np.dstack((f8,f8,f8))
        address = self._kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame8bit.ctypes.data, frame8bit.size)
        del address
        target_surface.unlock()

    def run(self):
        # -------- Main Program Loop -----------
        while not self._done:
            # --- Main event loop
            for event in pygame.event.get(): # User did something
                if event.type == pygame.QUIT: # If user clicked close
                    self._done = True # Flag that we are done so we exit this loop

                elif event.type == pygame.VIDEORESIZE: # window resized
                    self._screen = pygame.display.set_mode(event.dict['size'], 
                                               pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE, 32)
                    
            # --- Game logic should go here
            #print(self.peoplecount)
            #print('attempting serial write')
            if self.serial:
                try:
                    self.serial.write(['a','b','c','d','e','f'][self.peoplecount].encode('utf-8'))
                except:
                    print('serial bad times')
                    try:
                        self.serial.close()
                    except:
                        print('failed to close')
                    global port
                    if port:
                        self.serial = ser_conn(port)
                    else: 
                        self.serial = None
                    pass
                
            # --- Getting frames and drawing  
            # --- Woohoo! We've got a color frame! Let's fill out back buffer surface with frame's data 
            
            #print('attempting to fetch color frames')
            if self._kinect.has_new_color_frame():
                #print('has color frame')
                frame = self._kinect.get_last_color_frame()
                self.draw_color_frame(frame, self._frame_surface)
                frame = None

            if self._kinect.has_new_infrared_frame():
                #print('has color frame')
                frame = self._kinect.get_last_infrared_frame()
                self.draw_infrared_frame(frame, self._ir_frame_surface)
                frame = None
            
            if self._kinect.has_new_depth_frame():
                depth = self._kinect.get_last_depth_frame()
                depth = np.reshape(depth, (424, 512))
                self.draw_depth(depth)
                
            # --- Cool! We have a body frame, so can get skeletons
            #print('attempting to fetch body frames')
            if self._kinect.has_new_body_frame(): 
                #print("bones!")
                self._bodies = self._kinect.get_last_body_frame()

            # --- draw skeletons to _frame_surface
            if self._bodies is not None:
                
                self.peoplecount = 0 
                for i in range(0, self._kinect.max_body_count):
                    body = self._bodies.bodies[i]
                    if not body.is_tracked: 
                        continue 
                    
                    self.peoplecount += 1
                    
                    joints = body.joints
                    hands = (body.hand_left_state, body.hand_right_state)
                    # convert joint coordinates to color space 
                    joint_points = self._kinect.body_joints_to_color_space(joints)
                    self.draw_body(joints, joint_points, SKELETON_COLORS[i], hands)
            

            # --- copy back buffer surface pixels to the screen, resize it if needed and keep aspect ratio
            # --- (screen size may be different from Kinect's color frame size) 
            h_to_w = float(self._frame_surface.get_height()) / self._frame_surface.get_width()
            target_height = int(h_to_w * self._screen.get_width() * 0.7)
            surface_to_draw = pygame.transform.scale(self._frame_surface, (self._screen.get_width() * 0.7, target_height))
            self._screen.blit(surface_to_draw, (0,0))
            surface_to_draw = None

            h_to_w = float(self._ir_frame_surface.get_height()) / self._ir_frame_surface.get_width()
            ir_target_height = int(h_to_w * self._screen.get_width() * 0.3)
            surface_to_draw = pygame.transform.scale(self._ir_frame_surface, (self._screen.get_width() * 0.3, ir_target_height))
            self._screen.blit(surface_to_draw, (self._screen.get_width() * 0.7,0))
            surface_to_draw = None

            h_to_w = float(self._depth_frame_surface.get_height()) / self._depth_frame_surface.get_width()
            depth_target_height = int(h_to_w * self._screen.get_width() * 0.3)
            surface_to_draw = pygame.transform.scale(self._depth_frame_surface, (self._screen.get_width() * 0.3, depth_target_height))
            self._screen.blit(surface_to_draw, (self._screen.get_width() * 0.7,ir_target_height))
            surface_to_draw = None

            pygame.display.update()

            # --- Go ahead and update the screen with what we've drawn.
            pygame.display.flip()

            # --- Limit to 60 frames per second
            self._clock.tick(60)

        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()
        pygame.quit()


__main__ = "Kinect v2 Body Game"
game = BodyGameRuntime();
game.run();

