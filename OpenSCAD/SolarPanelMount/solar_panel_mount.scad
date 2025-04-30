// Copyright (c) 2025 Chris Lee and contibuters.
// Licensed under the MIT license. See LICENSE file in the project root for details.

include <MCAD/units.scad>

panel_mount_tube_diam = 24;
panel_mount_tube_gap = 0.5;
panel_mount_tube_wall = 2;
panel_holder_wall = 2;
panel_mount_tube_depth = 30;
panel_width = 130;
panel_thickness = 2;
panel_holder_gap = 1.5;
panel_holder_tooth_len = 5;

module SolarPanelMount() {
  $fn = 80;
  twall = panel_mount_tube_wall;
  hwall = panel_holder_wall;
  td = panel_mount_tube_depth;
  pw = panel_width;
  // inner tube radius
  itr = (panel_mount_tube_diam + panel_mount_tube_gap)/2;
  // outer tube radius
  otr = itr + panel_mount_tube_wall;
  mnt_dims = [pw+2*hwall+panel_holder_gap, 2*otr, hwall];
  union() {
    // Tube stem
    difference() {
      cylinder(td, otr, otr);
      translate(-Z*epsilon) cylinder(td+1, itr, itr);
    }
    translate([-mnt_dims[0]/2, -otr, td-epsilon]) {
      cube(mnt_dims);
      translate(Z*(hwall-epsilon)) {
	cube([hwall, mnt_dims[1], panel_thickness]);
	translate(Z*(panel_thickness-epsilon)) rotate(45, Y)
	  cube([hwall, mnt_dims[1], panel_holder_tooth_len]);
      }
    }
    translate([mnt_dims[0]/2-hwall, -otr, td-epsilon]) {
      translate(Z*(hwall-epsilon)) {
	cube([hwall, mnt_dims[1], panel_thickness]);
	translate([hwall, mnt_dims[1], panel_thickness-epsilon])
	  rotate(180, Z) rotate(45, Y)
	  cube([hwall, mnt_dims[1], panel_holder_tooth_len]);
      }
    }
  }
}


SolarPanelMount();
