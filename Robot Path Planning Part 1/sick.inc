
define sicksensor sensor
(				
  # factory settings for LMS200	
  range [ 0.0 8.0 ]
  pose [ 0 0 0.1 0 ]
  fov 180
  samples 180
  #samples 90 # still useful but much faster to compute
  color_rgba [0 0 1 0.15]
)

define sickcommon ranger
(
  color "blue"
  size [ 2 2 1] # dimensions from LMS200 data sheet
 )

define sickbase sickcommon
(
block( points 4
       point[0] [0 0]
       point[1] [0 1]
       point[2] [1 1]
       point[3] [1 0]
       z [0 0.21]
      )
)

define sicklaser sickbase ( sicksensor() )

