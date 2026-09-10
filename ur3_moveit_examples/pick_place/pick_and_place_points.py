"""Joint-taught positions for the fixed UR3 pick-and-place scenario."""


# Joint order:
# shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
HOME_JOINTS = (
    -0.118306,
    -0.487397,
    -1.822889,
    -0.814682,
    1.524297,
    -0.027445,
)

POINT1_JOINTS = (
    0.277823,
    -2.372771,
    -2.013256,
    -0.262646,
    1.725142,
    0.979218,
)

POINT2_JOINTS = (
    1.700911,
    -2.384013,
    -1.866491,
    -0.445122,
    1.504914,
    0.979218,
)

# TCP poses measured from base_link to tool0 at the taught joint positions.
# Order: x, y, z, qx, qy, qz, qw.
HOME_POSE = (
    0.003823,
    0.115193,
    0.510116,
    -0.527867,
    -0.462017,
    0.455855,
    0.547808,
)

POINT1_POSE = (
    -0.347673,
    0.003441,
    0.043215,
    0.903446,
    0.420082,
    -0.043600,
    0.073586,
)

POINT2_POSE = (
    -0.069039,
    -0.370382,
    0.047135,
    0.413547,
    0.909958,
    -0.022513,
    -0.021190,
)
