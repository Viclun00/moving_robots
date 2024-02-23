import os
import omni
import carb,time
import numpy as np
from omni.isaac.kit import SimulationApp
import asyncio                                                  
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core.utils.nucleus import get_assets_root_path, is_file
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.world import World
from omni.isaac.core.utils.prims import get_prim_at_path
import omni.kit.commands
from omni.isaac.wheeled_robots.robots import WheeledRobot
from pxr import Gf, Usd, PhysicsSchemaTools, UsdGeom, UsdPhysics       
from omni.isaac.core.utils.stage import is_stage_loading
from omni.physx.scripts import utils
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.wheeled_robots.controllers import WheelBasePoseController
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.types import ArticulationAction

from Mir100 import MIR100

from omni.isaac.range_sensor import _range_sensor   

async def get_lidar_param(lidarPath):                                    # Function to retrieve data from the LIDAR

    await omni.kit.app.get_app().next_update_async()            # wait one frame for data

    timeline.pause()                                            # Pause the simulation to populate the LIDAR's depth buffers

    depth = lidarInterface.get_linear_depth_data("/World"+lidarPath)
    
    print(depth)                                       # Print the data
    print(type(depth))





LOG = False
#goal_position = np.array([5.0,5.0,0.4])

project_path = os.getcwd()
warehouse_path = project_path + "/USD/warehouse.usd"
mir_path = project_path + "/USD/MIRURDF.usd"

wheel_radius = 6.45e-2
wheel_base = 45e-2

timeline = omni.timeline.get_timeline_interface()               
lidarInterface = _range_sensor.acquire_lidar_sensor_interface()

my_world = World(stage_units_in_meters=1, physics_prim_path="/physicsScene")
stage = omni.usd.get_context().get_stage()



omni.kit.commands.execute('SetLightingMenuModeCommand', lighting_mode='stage', usd_context_name='')
physx_scene = get_prim_at_path("/physicsScene")
physx_scene.GetAttribute("physxScene:enableGPUDynamics").Set(True)
add_reference_to_stage(usd_path=warehouse_path, prim_path="/World/Scene")

omni.kit.commands.execute('AddGroundPlaneCommand',
    stage=stage,
    planePath='/GroundPlane',
    axis='Z',
    size=25.0,
    position=Gf.Vec3f(0.0, 0.0, 0.0),
    color=Gf.Vec3f(0.5, 0.5, 0.5))

my_mir = my_world.scene.add(
    MIR100(
        prim_path="/World/mir100",
        name="mir100",
        wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
        wheel_dof_indices=[1,2],
        create_robot=True,
        usd_path=mir_path,
        position=np.array([0, -5.0, 0.3]),
    )
)



lidarPath = "/mir100/base_link/visuals/Lidar"




my_mir = my_world.scene.get_object("mir100")


my_diff_controller = DifferentialController(name="simple_control",wheel_radius=wheel_radius, wheel_base=wheel_base)
my_controller = WheelBasePoseController(name="pose_control", open_loop_wheel_controller=my_diff_controller,is_holonomic=False)

my_mir.set_joints_default_state(my_mir.get_joints_state())

#Warehouse [-25,25],[-30,0]

end_x = np.random.rand()*40 - 25
end_y = np.random.rand()*-20 

print(end_x)
print(end_y)

angular = 10
my_world.reset()


while simulation_app.is_running():
    
    my_world.step(render=True)
    timeline.play()                                                 # Start the Simulation
    asyncio.ensure_future(get_lidar_param(lidarPath=lidarPath))   

    if my_world.is_playing():
        if my_world.current_time_step_index == 0:

            my_diff_controller.reset()
        else:
            position, orientation = my_mir.get_world_pose()
            
            wheel_action = my_controller.forward(start_position = position, start_orientation = orientation, goal_position = [end_x , end_y])
            
            my_mir.apply_joint_actions(wheel_action)

            if np.abs(position[0] - end_x) < 0.2 and np.abs(position[1] - end_y) < 0.2:
                print(position)
                
                print('Reached')
                end_x = np.random.rand()*40 - 25
                end_y = np.random.rand()*- 20 
                print('Going to')
                print(end_x)
                print(end_y)



               


simulation_app.close()




