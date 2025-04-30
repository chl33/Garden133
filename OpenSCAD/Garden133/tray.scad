// Copyright (c) 2025 Chris Lee and contibuters.
// Licensed under the MIT license. See LICENSE file in the project root for details.

//include <board.scad>
include <box.scad>
include <gui.scad>
include <board.scad>

wall = 1;
space = 0.2;
battery_dims = [44, 25, 10];
battery_space_inner = battery_dims + space*[2, 2, 2];
battery_space_outer = battery_space_inner + [2*wall, 2*wall, 2];

battery_offset = [4, outer_dims[1], 0];

desiccant_bin_offset = battery_offset+[battery_space_outer[0]-wall, 0, 0];

descicant_bin_outer = [outer_dims[0]-4-desiccant_bin_offset[0],
		       battery_space_outer[1], battery_space_outer[2]];
descicant_bin_inner = descicant_bin_outer - wall*[2, 2, 1];

module tray() {
  Garden133_box(top=false);

  translate([battery_offset[0], 2, 0])
    cube([outer_dims[0]-8, battery_offset[1]+battery_space_outer[1]-2, wall]);

  translate(battery_offset) {
    // battery bin
    difference() {
      cube(battery_space_outer);
      translate(wall*[1, 1, 0]) cube(battery_space_inner+Z*10);
      translate([wall, -1, 5]) cube([3, 2+wall, battery_space_outer[2]]);
    }
    if (show_vitamins) {
      translate((wall+space)*[1, 1, 0] + Z*space) color("green") cube(battery_dims);
    }
  }

  translate(desiccant_bin_offset) {
    // desiccant bin
    difference() {
      cube(descicant_bin_outer);
      translate(wall*[1, 1, 0]) cube(descicant_bin_inner+Z*10);
    }
    if (show_vitamins) {
      translate((wall+space)*[1, 1, 0] + Z*space) color("green") cube(battery_dims);
    }
  }
}

tray();
