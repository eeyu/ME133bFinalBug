define kinect ranger
(
  sensor
  (
    range_max 4
    fov 360
    samples 720
    range [0 2]
  )
  # generic model properties
  color "blue"
  size [ 0.06 0.15 0.03 ]
  gui_nose 1
)

define turtlebot position
(
  pose [ 0 0 0.0 0.0 ]

  odom_error [0.00 0.00 999999 999999 999999 0.02]

  size [ 0.1552 0.1552 0.40 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "green"
  
  kinect(pose [ -0.1 0.0 -0.11 0.0 ])
)
