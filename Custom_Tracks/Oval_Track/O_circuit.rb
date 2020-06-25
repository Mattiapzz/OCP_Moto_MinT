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
  
  data.Road[:Segments] << {:length => 150.0,          :curvature => 0.00     , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 0
  data.Road[:Segments] << {:length => 75*Math::PI,    :curvature => 1/75.00  , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 1
  data.Road[:Segments] << {:length => 150.0,          :curvature => 0.00     , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 2
  data.Road[:Segments] << {:length => 150.0,          :curvature => 0.00     , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 3
  data.Road[:Segments] << {:length => 75*Math::PI,    :curvature => 1/75.00  , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 4
  data.Road[:Segments] << {:length => 150.0,          :curvature => 0.00     , :gridSize   => 1.0, :leftWidth  => 6.0, :rightWidth  => 6.0 }  # 5
  
  
end 
#EOF

