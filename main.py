#!/usr/bin/env python3

from datetime import datetime, timezone
from direct.showbase.ShowBase import ShowBase
from panda3d.core import (
    loadPrcFile,
    Vec3,
    Shader,
    DirectionalLight,
    AmbientLight,
    GeomVertexFormat, GeomVertexData, GeomVertexWriter,
    Geom, GeomPoints, GeomNode,
    GraphicsPipe, GraphicsOutput, Texture, WindowProperties, FrameBufferProperties,
    ShaderAttrib, RenderState,
    LColor, BitMask32
)
from sgp4 import omm
from sgp4.propagation import gstime
from sgp4.api import Satrec, SatrecArray
from sgp4.conveniences import jday_datetime
from math import sin, radians, degrees
import numpy as np
import logging
import os
import sys

from panda3d import core as p3core

# Constants
SCALE = 0.001  # 1 unit = 1000 km
EARTH_RADIUS = 6371.0
EARTH_TEXTURE_OFFSET = 160 # 148 TODO fix with better texture

# Shader
ENABLE_SHADER = True
VERT_SHADER = "shader/satellites.vert"
FRAG_SHADER = "shader/satellites.frag"
GL_PROGRAM_POINT_SIZE = 0x8642

# Data source
OMM_FILE = "all.csv"
if len(sys.argv) > 1:
    OMM_FILE = sys.argv[1]
OMM_PATH = os.path.join("res", OMM_FILE)
print(f"Loading omm resource file: {OMM_FILE}")

# Logging
np.set_printoptions(precision=2)
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Panda Config
loadPrcFile("config/Config.prc")


class SatelliteVisualizer(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        # self.disableMouse()
        self.setBackgroundColor(Vec3(0,0,0))

        self.init_gmst()
        self.setup_camera()
        self.setup_light() # TODO fix rotation
        self.setup_earth()

        self.load_omm_model()

        self.setup_shader()
        self.setup_picking()

        self.taskMgr.add(self.render_satellites, "render")


    def init_gmst(self):
        start_time = datetime.now(timezone.utc)
        jd, fr = jday_datetime(start_time)
        self.gstime = gstime(jd + fr)


    def setup_camera(self):
        self.camera.setPos(100,50,50)
        self.camera.lookAt(0,0,0)
    

    def setup_light(self):
        time = datetime.now(timezone.utc)
        day_of_year = time.timetuple().tm_yday
        sun_declination = 23.44 * sin(radians((360/365)*(day_of_year - 81)))
        sun_angle = degrees(self.gstime)

        sun_light = DirectionalLight("sun_light")
        sun_light.setColor(Vec3(1, 1, 0.9))
        sun_node = self.render.attachNewNode(sun_light)
        sun_node.setHpr(-90 + sun_angle, -sun_declination, 0)
        self.render.setLight(sun_node)

        ambient_light = AmbientLight("ambient_light")
        ambient_light.setColor(Vec3(0.2, 0.2, 0.3))
        ambient_node = self.render.attachNewNode(ambient_light)
        self.render.setLight(ambient_node)

    def setup_background(self):
        self.stars = self.loader.loadModel("models/solar_sky_sphere")
        self.stars_tex = self.loader.loadTexture("textures/2k_stars.jpg")
        self.stars.setTexture(self.stars_tex, 1)
        self.stars.setScale(10000)
        self.stars.reparentTo(self.render)


    def setup_earth(self):
        time = datetime.now(timezone.utc)
        hours = time.hour + time.minute/60 + time.second/3600
        earth_angle = np.fmod(hours * 15.0, 360.0)
        earth_angle = degrees(self.gstime)
        earth_angle = 0

        self.earth = self.loader.loadModel("models/planet_sphere")
        self.earth_tex = self.loader.loadTexture("textures/earth_1k_tex.jpg")
        self.earth.setTexture(self.earth_tex, 1)
        self.earth.setScale(EARTH_RADIUS * SCALE)
        self.earth.setHpr(EARTH_TEXTURE_OFFSET + earth_angle, 0, 0)
        self.earth.reparentTo(self.render)


    def load_omm_model(self):
        self.satellite_infos: list[dict[str,str]] = []
        self.satellite_orbits: list[Satrec] = []
        with open(OMM_PATH) as f:
            parsed = omm.parse_csv(f)
            for fields in parsed:
                sat = Satrec()
                omm.initialize(sat, fields)
                self.satellite_infos.append(fields)
                self.satellite_orbits.append(sat)


        vformat = GeomVertexFormat.getV3()
        self.vertex_data = GeomVertexData("points", vformat, Geom.UHDynamic)
        self.vertex_data.setNumRows(len(self.satellite_orbits))

        points = GeomPoints(Geom.UHDynamic)
        points.addNextVertices(len(self.satellite_orbits))
        points.closePrimitive()

        geom = Geom(self.vertex_data)
        geom.addPrimitive(points)

        node = GeomNode("points")
        node.addGeom(geom)
        self.points_np = self.render.attachNewNode(node)

    
    def setup_shader(self):
        if not ENABLE_SHADER:
            self.points_np.setLightOff()
            self.points_np.setRenderModeThickness(0.025)
            self.points_np.setRenderModePerspective(True)
            self.points_np.setColor(1, 1, 1, 1)

        shader = Shader.load(
            Shader.SL_GLSL,
            vertex=VERT_SHADER,
            fragment=FRAG_SHADER,
        )

        attrib = p3core.ShaderAttrib.make(shader)
        attrib = attrib.setFlag(p3core.ShaderAttrib.F_shader_point_size, True)

        self.points_np.setShader(shader)
        self.points_np.setAttrib(attrib)
        self.points_np.setShaderInputs(
            point_size=100,
            border_size=0.05,
            point_color=(1,1,1,1),
            border_color=(0,0,0,1),
        )


    def setup_picking(self):
        # self.index_tex = Texture()
        # self.index_tex.setup_2d_texture(
        #     self.win.get_x_size(), 
        #     self.win.get_y_size(), 
        #     Texture.T_unsigned_byte, 
        #     Texture.F_rgba8      
        # )
        # 
        # props = FrameBufferProperties()
        # props.set_rgb_color(True)
        # buffer = self.graphics_engine.make_output(
        #     self.pipe,
        #     "picking_buffer",
        #     -1,
        #     props,
        #     WindowProperties.get_default(),
        #     GraphicsPipe.BF_refuse_window,
        #     self.win.get_gsg(),
        #     self.win
        # )
        # buffer.set_active(True)
        # buffer.add_render_texture(self.index_tex, GraphicsOutput.RTM_copy_ram, GraphicsOutput.RTP_color)

        # picker_dr = buffer.make_display_region()
        # picker_cam = self.make_camera(buffer)
        # picker_cam.reparent_to(self.cam)

        # picking_shader = Shader.load(Shader.SL_GLSL, "shader/picking.vert", "shader/picking.frag")
        # picker_cam.node().set_initial_state(RenderState.make(ShaderAttrib.make(picking_shader)))

        win_props = WindowProperties.size(self.win.getXSize(), self.win.getYSize())
        fb_props = FrameBufferProperties()
        fb_props.setRgbColor(True)
        fb_props.setRgbaBits(8, 8, 8, 8) # 32-bit Red channel for large integer IDs
        fb_props.setDepthBits(24) # Need depth to handle occlusion correctly
        
        self.buffer = self.graphicsEngine.makeOutput(
            self.pipe, "index_buffer", -1,
            fb_props, win_props,
            GraphicsPipe.BFRefuseWindow,
            self.win.getGsg(), self.win
        )
        
        # 2. Create a texture to store the IDs
        self.id_tex = Texture()
        self.buffer.addRenderTexture(self.id_tex, GraphicsOutput.RTMCopyRam, GraphicsOutput.RTPColor)
        
        # 3. Create a camera for this buffer
        self.cam = self.makeCamera(self.buffer)
        self.cam.node().setLens(self.camLens) # Match the main camera lens
        self.cam.reparentTo(self.camera) # Move with the main camera
        
        # 4. Apply the ID Shader to everything this camera sees
        # We use a bitmask so this camera only sees the point cloud, not UI or other fluff
        mask = BitMask32.bit(1)
        self.points_np.hide(mask) # Hide from main cam if you want (optional)
        self.cam.node().setCameraMask(mask)
        self.points_np.show(mask) # Ensure point cloud is visible to this cam
        
        # Load the shader (code provided below)
        id_shader = Shader.load(Shader.SL_GLSL, "shader/picking.vert", "shader/picking.frag")
        self.points_np.setShader(id_shader)
        self.point_cloud.setTransparency(TransparencyAttrib.MNone, 1)
        
        # 5. Handle Mouse Click
        self.accept("mouse1", self.pick_vertex)


    def pick_vertex(self):
        if not self.mouseWatcherNode.hasMouse():
            return

        # Get mouse position in pixels
        mpos = self.mouseWatcherNode.getMouse()
        x = int((mpos.getX() + 1) / 2 * self.win.getXSize())
        y = int((mpos.getY() + 1) / 2 * self.win.getYSize())
        
        # Read the texture memory
        # Note: In a real app, optimize this to not read the whole texture every frame
        # or use base.graphicsEngine.extractTextureData() for a specific region.
        
        # A simpler way for single-click is creating a strictly 1x1 pixel texture 
        # peeker, but for now assuming we access the full texture:
        peeker = self.id_tex.peek()
        if peeker:
            # Read vector (r, g, b, a)
            val = LColor()
            peeker.fetchPixel(val, x, y) 
            # The ID is in the Red channel (assuming 32-bit float texture setup)
            # Depending on texture setup, you might need to decode standard RGB bytes
            r = int(val[0] * 255.0)
            g = int(val[1] * 255.0)
            b = int(val[2] * 255.0)

            picked_id = (r) + (g << 8) + (b << 16) - 1

            print(f"Selected id: {picked_id}")
            if picked_id >= 0:
                print(f"Selected Vertex: {self.satellite_infos[picked_id]['OBJECT_NAME']}")
            else:
                print("No point selected")


    def render_satellites(self, task):
        if task.frame % 30 != 0:

            # if self.mouseWatcherNode.hasMouse():
            #     x = self.mouseWatcherNode.getMouseX()
            #     y = self.mouseWatcherNode.getMouseY()
            #     # print(self.get_hovered_vertex(x, y), x, y)
            #     ram_image = self.index_tex.get_ram_image()
            #     if ram_image:
            #         print(ram_image[0])

            return task.cont

        time = datetime.now(timezone.utc)
        jd, fr = jday_datetime(time)
        orbits = SatrecArray(self.satellite_orbits).sgp4(np.array([jd]), np.array([fr]))
        _, positions, _ = orbits

        vertex_writer = GeomVertexWriter(self.vertex_data, "vertex")
        vertex_writer.setRow(0)

        scaled_positions = (positions * SCALE).astype(np.float32)
        num_rows = len(scaled_positions)
        self.vertex_data.setNumRows(num_rows)
        array_handle = self.vertex_data.modifyArray(0)
        memory_view = memoryview(array_handle)
        np_buffer = np.frombuffer(memory_view, dtype=np.float32)
        np_buffer[:] = scaled_positions.flatten()

        self.points_np.node().mark_bounds_stale()
        self.points_np.force_recompute_bounds()

        return task.cont


SatelliteVisualizer().run()
