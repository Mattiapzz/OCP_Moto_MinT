include Mechatronix

mechatronix do |data|

  # R O A D
  data.Road = {
    :MeshGridSize => 1,
    :RoadWidth    => 12.0, 
    :Theta0       => 0,
    :S0           => 0,
    :X0           => 0,
    :Y0           => 0,
    :IsSAE        => true,
    :Segments     => []
  }
  
  data.Road[:Segments] << {:length => 20.0,          :curvature => 0.00     , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 0
  data.Road[:Segments] << {:length => 366.933546-20, :curvature => 0.0      , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 1
  data.Road[:Segments] << {:length => 36.395335,     :curvature => 0.005014 , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 2
  data.Road[:Segments] << {:length => 17.508761,     :curvature => 0.000000 , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 3
  data.Road[:Segments] << {:length => 26.719483,     :curvature => 0.058793 , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 4
  data.Road[:Segments] << {:length => 62.995808,     :curvature => 0.018212 , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 5
  data.Road[:Segments] << {:length => 392.057882,    :curvature => 0.0      , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 6
  data.Road[:Segments] << {:length => 27.236252,     :curvature => -0.010979, :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 7
  data.Road[:Segments] << {:length => 51.298757,     :curvature => 0.0      , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 8
  data.Road[:Segments] << {:length => 41.608797,     :curvature => 0.052652 , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 9 
  data.Road[:Segments] << {:length => 97.007909,     :curvature => 0.016655 , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 10
  data.Road[:Segments] << {:length => 74.636919,     :curvature => -0.004980, :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 11
  data.Road[:Segments] << {:length => 208.502649,    :curvature => 0.0      , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 12
  data.Road[:Segments] << {:length => 43.563564,     :curvature => -0.058555, :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 13
  data.Road[:Segments] << {:length => 27.095868,     :curvature => -0.015751, :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 14
  data.Road[:Segments] << {:length => 226.951522,    :curvature => 0.0      , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 15
  data.Road[:Segments] << {:length => 3.891352,      :curvature => -0.020000, :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 16
  data.Road[:Segments] << {:length => 85.376172,     :curvature => 0.0      , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 17
  data.Road[:Segments] << {:length => 23.941104,     :curvature => -0.055556, :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 18
  data.Road[:Segments] << {:length => 3.713811,      :curvature => 0.0      , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 19
  data.Road[:Segments] << {:length => 38.191425,     :curvature => 0.035746 , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 20
  data.Road[:Segments] << {:length => 5.983022,      :curvature => -0.010000, :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 21
  data.Road[:Segments] << {:length => 2.368430,      :curvature => 0.0      , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 22
  data.Road[:Segments] << {:length => 29.104257,     :curvature => 0.035714 , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 23
  data.Road[:Segments] << {:length => 97.139903,     :curvature => 0.0      , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 24
  data.Road[:Segments] << {:length => 48.274475,     :curvature => 0.031250 , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 25
  data.Road[:Segments] << {:length => 127.426317,    :curvature => 0.0      , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 26
  data.Road[:Segments] << {:length => 44.496183,     :curvature => -0.010417, :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 27
  data.Road[:Segments] << {:length => 29.768515,     :curvature => 0.0      , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 28
  data.Road[:Segments] << {:length => 46.942398,     :curvature => 0.043478 , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 29
  data.Road[:Segments] << {:length => 14.669384,     :curvature => 0.0      , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 30
  data.Road[:Segments] << {:length => 54.032989,     :curvature => 0.031250 , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 31
  data.Road[:Segments] << {:length => 143.257805,    :curvature => 0.0      , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 32
  data.Road[:Segments] << {:length => 74.854041,     :curvature => -0.035713, :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 33
  data.Road[:Segments] << {:length => 82.764839,     :curvature => 0.0      , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 34
  data.Road[:Segments] << {:length => 59.341977,     :curvature => 0.003122 , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 35
  # data.Road[:Segments] << {:length => 80 ,           :curvature => 0.0      , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 36a
  # data.Road[:Segments] << {:length => 20,            :curvature => 0.0      , :gridSize   => 0.5, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 36b
  
end 
#EOF

