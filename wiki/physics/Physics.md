# Physics Configuration

> [!Warning]
> This page is under construction.

Attributes:
- `dt`: `(float)`, The time step for each physics simulation update. Defines the amount of time that passes per frame for physics calculations.
- `gravity`: `(Tuple[float, float, float])`, The gravitational force vector applied in the simulation, defined as `(x, y, z)` where each axis represents a directional force in meters per second squared.
- `substeps`: `(int)`, The number of substeps for the physics solver. This helps with simulation accuracy, especially in high-speed or complex interactions.
- `use_gpu_pipeline`: `(bool)`, Whether to enable the GPU pipeline for physics processing. When `True`, computations are offloaded to the GPU, improving performance for large simulations.
- `worker_thread_count`: `(int)`, The number of threads allocated for physics processing. A higher count can improve performance but may increase CPU usage.
- `use_fabric`: `(bool)`, TODO
- `enable_scene_query_support`: `(bool)`, TODO
- `gpu_max_rigid_contact_count`: `(int)`, The maximum number of rigid body contacts that the GPU can handle per frame.
- `gpu_max_rigid_patch_contact_count`: `(int)`, The maximum number of patch contacts between rigid bodies that the GPU can process per frame.
- `gpu_found_lost_pairs_capacity`: `(int)`, The capacity for handling found and lost contact pairs in the GPU pipeline. Determines how many pairs can be processed before overflow occurs.
- `gpu_total_aggregate_pairs_capacity`: `(int)`, The total capacity for aggregate pairs (grouped objects) that can be processed by the GPU.
- `gpu_max_soft_body_contacts`: `(int)`, The maximum number of soft body contacts the GPU can process. Soft bodies include deformable objects such as cloth or elastic materials.
- `gpu_max_particle_contacts`: `(int)`, The maximum number of particle contacts the GPU can process. This typically applies to particle-based simulations such as fluids or granular materials.
- `gpu_heap_capacity`: `(int)`, The heap memory capacity reserved for GPU physics processing.
- `gpu_temp_buffer_capacity`: `(int)`, The temporary buffer size for the GPU pipeline during physics computations.
- `gpu_max_num_partions`: `(int)`, The maximum number of partitions the GPU can handle, related to splitting the physics scene into smaller sections for processing.
- `gpu_collision_stack_size`: `(int)`, The stack size allocated for handling collisions on the GPU.
- `solver_type`: `(str)`, The type of solver used for physics simulation. Common values include `PBD` (Position-Based Dynamics) or `TGS` (Temporal Gauss-Seidel).
- `enable_stabilization`: `(bool)`, Whether to enable stabilization for the solver, which helps reduce jitter and improve the stability of simulations involving contact or collision.
- `bounce_threshold_velocity`: `(float)`, The velocity threshold for bounce calculations. Objects with a velocity below this threshold will not bounce upon impact.
- `friction_offset_threshold`: `(float)`, The offset distance used when applying friction between objects in contact. Affects how friction is calculated based on the distance between objects.
- `friction_correlation_distance`: `(float)`, The correlation distance for friction calculation. Defines the area over which friction is applied between two surfaces.
- `enable_ccd`: `(bool)`, Whether to enable Continuous Collision Detection (CCD). CCD prevents fast-moving objects from passing through one another by performing more frequent collision checks.

> [!Note]
> - GPU-specific settings are important for optimizing large simulations, especially when dealing with rigid bodies, soft bodies, and particles.
> - The number of substeps and solver type significantly impact the accuracy and stability of the physics simulation.
> - Using the GPU pipeline and adjusting thread count allows for performance improvements, particularly in complex scenes.
> - The stabilization options and collision detection settings like CCD help avoid errors in physics calculations, such as objects passing through one another or jittering at rest.


Example:
```yaml
physics_scene:
  dt: 0.16666
  gravity: [0.0, 0.0, -1.62]
  enable_ccd: true
  enable_stabilization: false
  solver_type: PGS
  use_gpu_pipeline: false
```
