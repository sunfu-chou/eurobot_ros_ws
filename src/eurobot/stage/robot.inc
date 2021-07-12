define laser ranger
(
  sensor
  (
    range_max 6.5
    fov 360
    samples 1440
  )
  color "black"
  size [ 0.06 0.15 0.03 ]
  interval_sim 100
  noise [1 10 10]
)

define diff_drive_robot position
(
  odom_error [0.3 0.3 0.00 0.3]

  size [ 0.25 0.25 0.40 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  color "red"

  # kinematics
  drive "diff"

  # spawn sensors
  laser(
    pose [ 0.0 0.0 0.6 0.0 ]
    noise [1 10 10]
  )
)
