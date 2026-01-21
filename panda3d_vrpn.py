from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.vrpn import VrpnClient
from panda3d.core import Quat, Camera, TrackerNode, ButtonNode, AnalogNode, Transform2SG, loadPrcFileData, PerspectiveLens, LVecBase3, Lens, ColorAttrib

loadPrcFileData("", "fullscreen 1")
loadPrcFileData("", "win-size 1920 1080")
loadPrcFileData("", "framebuffer-stereo 1")

class PandaDavis(ShowBase):

    IOD_METERS = 0.064
    LEDWALL_WIDTH_METERS = 4.6
    LEDWALL_HEIGHT_METERS = 2.6
    Z_CALIBRATION_METERS = 3.0
    TRANSFORMATION_BUTTON_NUMBER = 0
    TRANSFORMATION_SPEED = 2
    NEAR_PLANE = 0.1
    FAR_PLANE = 1000

    def __init__(self):
        ShowBase.__init__(self)

        self.setup_vrpn()
        self.setup_stereo()

        # COVISE-style navigation
        self.transformation_started = False
        self.reference_pos = LVecBase3(0, 0, 0)
        self.reference_quat = Quat()

        self.environment = self.loader.loadModel("models/environment")
        self.environment.reparentTo(render)
        self.environment.setScale(0.25, 0.25, 0.25)
        self.environment.setPos(-8, 42, 0)
    
        cube = self.loader.loadModel("models/box")
        cube.reparentTo(render)
        cube.setPos(0, PandaDavis.Z_CALIBRATION_METERS, 0)
        cube.setScale(1)


    def create_pointer(self, parent, length=10.0, radius=0.02):
        pointer = parent.attachNewNode("pointer")
        beam = loader.loadModel("models/box")
        beam.reparentTo(pointer)
        beam.setScale(radius, radius, -length)
        beam.setTwoSided(True)

        beam.setLightOff()
        beam.clearTexture()
        beam.setColor(0.2, 0.4, 1.0, 1.0)
        beam.setAttrib(ColorAttrib.makeFlat((0.2, 0.4, 1.0, 1.0)))

        return pointer


    def setup_vrpn(self):
        # connect to vrpn server
        self.vrpnclient = VrpnClient("localhost")
        
        # register update method used to poll new data from the VRPN server via the VrpnClient
        taskMgr.add(self.poll_vrpn_server, "poll_vrpn_server")
        
        # CAVE parent rig, moves according to tracking and covise-style scene navigation
        self.rig = self.render.attachNewNode("rig")

        # VRPN trackers:
        
        # In Panda3D, input is handled via the data graph, a tree that lives separately to the scene graph.
        # Therefore, attaching a tracker node input (data graph) to a scene graph node will not work.
        # Instead we need to create a node in the data graph for the input, a node in the scene graph for the actual scene, and convert
        # from the data graph node to the scene graph node using a Transform2SG object ("Transform to Scene Graph").

        # head glasses tracker (sensor #1)
        self.head_tracker = TrackerNode(self.vrpnclient, "DTrack:2")        # in DATA graph: create a tracker node using the VRPN name (TrackerName:SensorNumber)
        self.dataRoot.node().addChild(self.head_tracker)                    # add tracker to DATA graph
        self.head = render.attachNewNode("head")                            # create a corresponding node for the SCENE graph
        d2s_head = Transform2SG("d2s_head")                                 # create a transform object to transform from DATA graph to SCENE graph (SG)
        self.head_tracker.addChild(d2s_head)                                # add the transform object as a child to the DATA graph node
        d2s_head.setNode(self.head.node())                                  # set the output node to the SCENE graph node
        self.head.reparentTo(self.rig)

        # attach left and righ eye trackers (only used for projection matrix calculation)
        self.left = self.head.attachNewNode("left")
        self.left.setPos(-PandaDavis.IOD_METERS / 2.0, 0.0, 0.0)
        self.right = self.head.attachNewNode("right")
        self.right.setPos(+PandaDavis.IOD_METERS / 2.0, 0.0, 0.0)

        # Flystick 8 tracker (sensor #8)
        self.flystick8_tracker = TrackerNode(self.vrpnclient, "DTrack:8")   # in DATA graph: create a tracker node using the VRPN name (TrackerName:SensorNumber)
        self.dataRoot.node().addChild(self.flystick8_tracker)               # add tracker to DATA graph
        self.flystick8 = render.attachNewNode("flystick8")                  # create a corresponding node for the SCENE graph
        d2s_flystick8 = Transform2SG("d2s_flystick8")                       # create a transform object to transform from DATA graph to SCENE graph (SG)
        self.flystick8_tracker.addChild(d2s_flystick8)                      # add the transform object as a child to the DATA graph node
        d2s_flystick8.setNode(self.flystick8.node())                        # set the output node to the SCENE graph node
        self.flystick8.reparentTo(self.rig)

        # Flystick 9 tracker (sensor #9)
        self.flystick9_tracker = TrackerNode(self.vrpnclient, "DTrack:9")   # in DATA graph: create a tracker node using the VRPN name (TrackerName:SensorNumber)
        self.dataRoot.node().addChild(self.flystick9_tracker)               # add tracker to DATA graph
        self.flystick9 = render.attachNewNode("flystick9")                  # create a corresponding node for the SCENE graph
        d2s_flystick9 = Transform2SG("d2s_flystick9")                       # create a transform object to transform from DATA graph to SCENE graph (SG)
        self.flystick9_tracker.addChild(d2s_flystick9)                      # add the transform object as a child to the DATA graph node
        d2s_flystick9.setNode(self.flystick9.node())                        # set the output node to the SCENE graph node
        self.flystick9.reparentTo(self.rig)

        self.flystick8_pointer = self.create_pointer(self.flystick8)
        self.flystick9_pointer = self.create_pointer(self.flystick9)
        
        # VRPN buttons:
        self.buttons = ButtonNode(self.vrpnclient, "DTrack")
        taskMgr.add(self.handle_vrpn_buttons, "handle_vrpn_buttons")
        
        # VRPN analogs:
        self.analogs = AnalogNode(self.vrpnclient, "DTrack")
        taskMgr.add(self.handle_vrpn_analogs, "handle_vrpn_analogs")


    def setup_stereo(self):

        # disable default cam
        base.camNode.setActive(False)
        base.cam.node().setActive(False)

        # make_display_region automatically returns a StereoDisplayRegion if framebuffer-stereo is set to 1 in config
        # both left and right display region share the default camera initially
        self.stereo_display_region = base.win.make_display_region()
        left_display_region = self.stereo_display_region.getLeftEye()
        right_display_region = self.stereo_display_region.getRightEye()
        
        # create and configure left and right eye camera:

        self.left_cam = self.left.attachNewNode(Camera("left_cam"))
        self.right_cam = self.right.attachNewNode(Camera("right_cam"))

        screen_z = PandaDavis.Z_CALIBRATION_METERS
        screen_left = -PandaDavis.LEDWALL_WIDTH_METERS / 2.0
        screen_right = PandaDavis.LEDWALL_WIDTH_METERS / 2.0
        screen_top = PandaDavis.LEDWALL_HEIGHT_METERS / 2.0
        screen_bottom = -PandaDavis.LEDWALL_HEIGHT_METERS / 2.0

        self.screen_ul = LVecBase3(screen_left, screen_z, screen_top)
        self.screen_ur = LVecBase3(screen_right, screen_z, screen_top)
        self.screen_ll = LVecBase3(screen_left, screen_z, screen_bottom)
        self.screen_lr = LVecBase3(screen_right, screen_z, screen_bottom)

        left_lens = PerspectiveLens()
        left_lens.setNear(PandaDavis.NEAR_PLANE)
        left_lens.setFar(PandaDavis.FAR_PLANE)

        right_lens = PerspectiveLens()
        right_lens.setNear(PandaDavis.NEAR_PLANE)
        right_lens.setFar(PandaDavis.FAR_PLANE)

        self.left_cam.node().setLens(left_lens)
        self.right_cam.node().setLens(right_lens)

        left_display_region.setCamera(self.left_cam)
        right_display_region.setCamera(self.right_cam)

        taskMgr.add(self.update_projection_matrix, "update_projection_matrix")

    
    def update_projection_matrix(self, t):

        ul_l = self.left.getRelativePoint(self.rig, self.screen_ul)
        ur_l = self.left.getRelativePoint(self.rig, self.screen_ur)
        ll_l = self.left.getRelativePoint(self.rig, self.screen_ll)
        lr_l = self.left.getRelativePoint(self.rig, self.screen_lr)

        ul_r = self.right.getRelativePoint(self.rig, self.screen_ul)
        ur_r = self.right.getRelativePoint(self.rig, self.screen_ur)
        ll_r = self.right.getRelativePoint(self.rig, self.screen_ll)
        lr_r = self.right.getRelativePoint(self.rig, self.screen_lr)

        flags = Lens.FC_off_axis

        self.left_cam.node().getLens().setFrustumFromCorners(
            ul_l, ur_l, ll_l, lr_l, flags
        )

        self.right_cam.node().getLens().setFrustumFromCorners(
            ul_r, ur_r, ll_r, lr_r, flags
        )

        return Task.cont
    

    def poll_vrpn_server(self, t):
        self.vrpnclient.poll()
        self.head.setQuat(Quat()) # discard tracked orientation (cannot face away from wall)
        return Task.cont
        
    
    # DTrack exposes a single button device for all Flystick buttons, that means:
    # Buttons 0-8 are for Flystick 9 and buttons 8-16 are for Flystick 8.
    def handle_vrpn_buttons(self, t):

        # Default COVISE-style scene navigation:

        tbtn_pressed = self.buttons.getButtonState(PandaDavis.TRANSFORMATION_BUTTON_NUMBER)

        if not self.transformation_started and tbtn_pressed:
            self.transformation_started = True

        if not tbtn_pressed:
            self.transformation_started = False
            self.reference_pos = LVecBase3(self.flystick9.getPos())
            self.reference_quat = Quat(self.flystick9.getQuat())       

        if self.transformation_started:
            dt = globalClock.getDt()
            
            # translation
            self.rig.setPos(self.rig, (self.flystick9.getPos() - self.reference_pos) * PandaDavis.TRANSFORMATION_SPEED * dt)

            # rotation
            q_old_inv = Quat(self.reference_quat)
            q_old_inv.invertInPlace()
            q_diff = q_old_inv * self.flystick9.getQuat()
            h, _, _ = q_diff.getHpr()
            q_yaw_only = Quat()
            q_yaw_only.setHpr((h * PandaDavis.TRANSFORMATION_SPEED * dt, 0, 0))
            self.rig.setQuat(self.rig.getQuat() * q_yaw_only)

        # handle your Flystick button presses here
        # example:
        #if self.buttons.getButtonState(0): # trigger Flystick 9
            #print("Flystick ID 9 button 0 pressed")
        return Task.cont
        

    
    # DTrack exposes a single analog device for all Flystick joysticks, that means:
        # Flystick 9 x-Axis: 0
        # Flystick 9 y-Axis: 1
        # Flystick 8 x-Axis: 2
        # Flystick 8 y-Axis: 3
    def handle_vrpn_analogs(self, t):
        
        # handle your Flystick joystick inputs here
        # examples:

        x8 = self.analogs.getControlState(2)
        y8 = self.analogs.getControlState(3)
        #print(f"Flystick ID 8 joystick: {x8}|{y8}")
        
        x9 = self.analogs.getControlState(0)
        y9 = self.analogs.getControlState(1)
        #print(f"Flystick ID 9 joystick: {x9}|{y9}")
        
        return Task.cont
        

template = PandaDavis()
template.run()
