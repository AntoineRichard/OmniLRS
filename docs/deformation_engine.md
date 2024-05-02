## DeformationEngine

Here, the docs gives a brief explanation of deformation engine plugin. \
In hydra cfg file (see `cfg/environment/**.yaml`), you have specific lines for terrain deformation. 

```yaml
deformation_engine:
    enable: True
    render_deform_inv: 10
    terrain_width: ${....lunaryard_settings.lab_width}
    terrain_height: ${....lunaryard_settings.lab_length}
    terrain_resolution: ${....lunaryard_settings.resolution}
    gravity: [0, 0, -49.0] #mg
    force_depth_slope: 0.00014
    force_depth_intercept: 0.008
    wheel_params:
        wheel_width: 0.09
        wheel_radius: 0.1
    deform_constraint:
        deform_offset: 0.0
        deform_decay_ratio: 0.01
    force_distribution: 
        distribution: sinusoidal
        wave_frequency: 2.0
    boundary_distribution: 
        distribution: trapezoidal
        angle_of_repose: 1.047
```

### Wheel parameters

You must specify the dimension of your wheels in meter. 
```yaml
wheel_params:
    wheel_width: 0.09
    wheel_radius: 0.1
```

### Deformation Constraint
Here you will specify parameter for constraint. \
`deform_offsest` is the distance between wheel origin and the center of deformation profile. \
`deform_decay_ratio` is decaying parameter of deformation. \
This is to limit deformation depth as rover traverse the same place many times. 
```yaml
deform_constraint:
    deform_offset: 0.0
    deform_decay_ratio: 0.01
```

### Force Distribution 
Controls distribution of force over wheel footprint.
You have two options for this. 

```yaml
force_distribution: 
    distribution: uniform
```

```yaml
force_distribution: 
    distribution: sinusoidal
    wave_frequency: 2.0
```

### Boundary Distribution
Controls boundary shape (y axis vs z axis with FLU convention). 

```yaml
boundary_distribution: 
    distribution: uniform
```

```yaml
boundary_distribution: 
    distribution: parabolic
```

```yaml
boundary_distribution: 
    distribution: trapezoidal
    angle_of_repose: 1.047
```