from src.configurations.physics_confs import PhysicsSceneConf

from omni.isaac.core.physics_context.physics_context import PhysicsContext


class PhysicsSceneManager:
    def __init__(self, settings: PhysicsSceneConf) -> None:
        self.settings = settings
        self.physics_context = PhysicsContext(sim_params=self.settings.physics_scene_args)
        if self.settings.enable_ccd:
            self.physics_context.enable_ccd(True)
