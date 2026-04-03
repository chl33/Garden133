// Copyright (c) 2026 Chris Lee and contributors.
// Licensed under the MIT license. See LICENSE file in the project root for details.

epsilon = 0.001;

module soilpot() {
  dims = [40, 25, 12];
  wall = 1;
  sensor_z = 1.5;

  inner_dims = dims - wall*[1, 2, 1];

  difference() {
    cube(dims);
    translate([-epsilon, wall, wall+epsilon]) cube(inner_dims);
    angle = atan((dims[2]-sensor_z)/dims[0]);
    translate([-epsilon, -epsilon, sensor_z]) rotate([0, -angle, 0]) cube(dims + [1,1,1]);
  }
}

soilpot();
