// Copyright (c) 2025 Chris Lee and contibuters.
// Licensed under the MIT license. See LICENSE file in the project root for details.

include <ProjectBox/mounts.scad>
include <ProjectBox/project_box.scad>
include <ProjectBox/shtc3_window.scad>
include <board.scad>

ones = [1, 1, 1];

wall_thickness = 1;
gap = 0.2;
corner_radius = 2;
mount_offset = pad_space;

space_above_board = 3;
space_below_board = 5;
inner_dims = (board_dims
	      + Z*(space_above_board+space_below_board)
	      + 2*gap*ones);
outer_dims = (inner_dims
	      + 2*ones*wall_thickness
	      + [2, 2, 0] * corner_radius);

shtc3_loc = [51.8, 1.4];

buttons_loc = [30, 15];
buttons_size = [13, 7];

conn_loc = [65, 7];
conn_size  = [7.6, 29.5];

top_cutouts = [[buttons_loc, buttons_size], [conn_loc, conn_size]];

usb_cutout = [[17.75, wall_thickness+space_below_board+board_thickness-1], [9.5, 3.5]];
ym_cutouts = [usb_cutout];

// Location for "USB DBG" text to carve above buttons
text_loc = [25, 25];

antenna_cutout = [[40, -1], [5, wall_thickness+space_below_board+board_thickness+4.4]];
// antenna_cutout = [[5, wall_thickness+space_below_board+board_thickness+1], [10, 4]];
yp_cutouts = [antenna_cutout];

module in_Garden133_board_frame(board_height=false) {
  zoffset = wall_thickness + (board_height ? space_below_board : 0);
  in_board_frame(outer_dims, board_dims, zoffset) children();
}

module Garden133_box(top) {
  wall = wall_thickness;

  difference() {
    union() {
      project_box(outer_dims,
		  wall_thickness=wall_thickness,
		  gap=gap,
		  snaps_on_sides=true,
		  corner_radius=corner_radius,
		  top=top,
		  top_cutouts=top_cutouts,
		  ym_cutouts=ym_cutouts,
		  yp_cutouts=yp_cutouts);
      if (top) {
	translate([text_loc[0]-2, text_loc[1]-2, outer_dims[2]-2*wall+epsilon]) cube([30, 8, wall]);
	in_Garden133_board_frame(board_height=true)
	  shtc3_window(shtc3_loc, space_above_board+2*wall, wall, false, z_gap=1.5);
	// Bumps on top etc.
	// translate(b1o) rounded_box(b1d, corner_radius);
      } else {
	// Stuff to add on bottom.
	// Stuff to add on bottom.
	in_Garden133_board_frame() {
	  at_corners(board_dims+1.0*X, mount_offset, x_extra=-1.2, y_extra=-0.2)
	    screw_mount(space_below_board, wall, 2.5/2);
	}
	
      }	
    }
    // Cut outs.
    if (top) {
      in_Garden133_board_frame(board_height=true)
	shtc3_window(shtc3_loc, space_above_board+wall+1, wall, true, z_gap=1);
      
      // Negative space of bumps.
      // translate(b1o+[wall, wall, -epsilon]) rounded_box(b1d-wall*[2, 2, 1], corner_radius);
      // translate(b2o+[wall, wall, -epsilon]) rounded_box(b2d-wall*[2, 2, 1], corner_radius);

      translate([text_loc[0], text_loc[1], outer_dims[2]-0.5+epsilon])
	linear_extrude(0.5) text("USB DBG", size=4);
    }
  }
}
