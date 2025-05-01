epsilon = 0.001;

module soilpot() {
  dims = [36, 25, 10];
  // wire_cutout = [7, 5];
  wall = 1;
  sensor_z = 1.5;

  inner_dims = dims - wall*[1, 2, 1];

  difference() {
    cube(dims);
    translate([-epsilon, wall, wall+epsilon]) cube(inner_dims);
    angle = atan((dims[2]-sensor_z)/dims[0]);
    translate([-epsilon, -epsilon, sensor_z]) rotate([0, -angle, 0]) cube(dims + [1,1,1]);
    // translate([dims[0]-wall-1, (dims[1]-wire_cutout[0])/2, dims[2]-wire_cutout[1]+epsilon])
    //   cube([wall+2, wire_cutout[0], wire_cutout[1]+1]);
  }
}

soilpot();
