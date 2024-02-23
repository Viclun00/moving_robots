from omni.isaac.core.robots.robot import Robot
from typing import Optional, Tuple, Sequence
from omni.isaac.core.materials.visual_material import VisualMaterial
from omni.isaac.core.prims import GeometryPrim, XFormPrim
from omni.isaac.core.utils.types import XFormPrimState, ArticulationAction
from omni.isaac.core.utils.string import find_unique_string_name
from pxr import Gf, PhysicsSchemaTools, Usd
from omni.isaac.core.materials import PhysicsMaterial
from omni.isaac.core.materials import PreviewSurface
from omni.isaac.core.utils.prims import (
    get_prim_path,
    get_prim_at_path,
    define_prim,
    is_prim_path_valid,
    get_first_matching_child_prim,
    get_prim_type_name,
)
from omni.isaac.core.utils.stage import get_current_stage, get_stage_units
import numpy as np
import carb



class MIR100(Robot):
    """[summary]

        Args:
            prim_path (str): [description]
            name (str): [description]
            wheel_dof_names ([str, str]): name of the wheels, [left,right].
            wheel_dof_indices: ([int, int]): indices of the wheels, [left, right]
            usd_path (str, optional): [description]
            create_robot (bool): create robot at prim_path if no robot exist at said path. Defaults to False
            position (Optional[np.ndarray], optional): [description]. Defaults to None.
            orientation (Optional[np.ndarray], optional): [description]. Defaults to None.
        """

    def __init__(
        self,
        prim_path: str,
        wheel_dof_names: Optional[str] = None,
        wheel_dof_indices: Optional[int] = None,
        name: str = "agv_robot",
        usd_path: Optional[str] = None,
        create_robot: Optional[bool] = False,
        position: Optional[np.ndarray] = None,
        translation: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        prim = get_prim_at_path(prim_path)
        if not prim.IsValid():
            if create_robot:
                prim = define_prim(prim_path, "Xform")
                if usd_path:
                    prim.GetReferences().AddReference(usd_path)
                else:
                    carb.log_error("no valid usd path defined to create new robot")
            else:
                carb.log_error("no prim at path %s", prim_path)
        super().__init__(
            prim_path=prim_path, name=name, position=position, orientation=orientation, articulation_controller=None, translation=translation,
        )
        self._wheel_dof_names = wheel_dof_names
        self._wheel_dof_indices = wheel_dof_indices
        # TODO: check the default state and how to reset
        return

    @property
    def wheel_dof_indices(self):
        return self._wheel_dof_indices
    
    def get_wheel_positions(self):
        full_dofs_positions = self.get_joint_positions()
        wheel_joint_positions = [full_dofs_positions[i] for i in self._wheel_dof_indices]
        return wheel_joint_positions



    def set_wheel_positions(self, positions) -> None:
        full_dofs_positions = [None] * self.num_dof
        for i in range(self._num_wheel_dof):
            full_dofs_positions[self._wheel_dof_indices[i]] = positions[i]
        self.set_joint_positions(positions=np.array(full_dofs_positions))
        return
    

    def get_wheel_velocities(self):
        full_dofs_velocities = self.get_joint_velocities()
        wheel_dof_velocities = [full_dofs_velocities[i] for i in self._wheel_dof_indices]
        return wheel_dof_velocities
    


    def set_wheel_velocities(self, velocities) -> None:
        full_dofs_velocities = [None] * self.num_dof
        for i in range(self._num_wheel_dof):
            full_dofs_velocities[self._wheel_dof_indices[i]] = velocities[i]
        self.set_joint_velocities(velocities=np.array(full_dofs_velocities))
        return
    
    
    
    def apply_joint_actions(self, wheel_action: ArticulationAction) -> None:
        actions_length = wheel_action.get_length()
        if actions_length is not None and actions_length != (self._num_wheel_dof):
            raise Exception("ArticulationAction passed should be the same length as the number of wheels")
        joint_actions = ArticulationAction()

        if wheel_action.joint_positions is not None:
            joint_actions.joint_positions = np.zeros(self.num_dof)
      
        if wheel_action.joint_positions is not None:
            for i in range(self._num_wheel_dof):
                joint_actions.joint_positions[self._wheel_dof_indices[i]] = wheel_action.joint_positions[i]

        if wheel_action.joint_velocities is not None:
            joint_actions.joint_velocities = np.zeros(self.num_dof)
   
        if wheel_action.joint_velocities is not None:
            for i in range(self._num_wheel_dof):
                joint_actions.joint_velocities[self._wheel_dof_indices[i]] = wheel_action.joint_velocities[i]

        if wheel_action.joint_efforts is not None:
            joint_actions.joint_efforts = np.zeros(self.num_dof)
        
        if wheel_action.joint_efforts is not None:
            for i in range(self._num_wheel_dof):
                joint_actions.joint_efforts[self._wheel_dof_indices[i]] = wheel_action.joint_efforts[i]
                
        self.apply_action(control_actions=joint_actions)
        return
        

    def initialize(self, physics_sim_view=None) -> None:
        super().initialize(physics_sim_view=physics_sim_view)
        if self._wheel_dof_names is not None :
            self._wheel_dof_indices = [
                self.get_dof_index(self._wheel_dof_names[i]) for i in range(len(self._wheel_dof_names))
            ]
            
        else:
            if self._wheel_dof_indices is None:
                carb.log_error("need to have either wheel names or wheel indices")
            
            

        self._num_wheel_dof = len(self._wheel_dof_indices)
        
        return

    def post_reset(self) -> None:
        super().post_reset()
        self._articulation_controller.switch_control_mode(mode="velocity")
        return

    def get_articulation_controller_properties(self):
        return self._wheel_dof_names, self._wheel_dof_indices