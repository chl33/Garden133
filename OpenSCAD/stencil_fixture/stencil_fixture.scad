// Copyright (c) 2026 Chris Lee and contributors.
// Licensed under the MIT license. See LICENSE file in the project root for details.

show_vitamins = true;

// This is a fixture for holding a PCB and stencil in alignment
module stencil(board_dims_xy, dscrew, screw_offset, board_stl_file,
	       board_thickness = 1.6, outside_border = 2,
               base_thickness = 1, pin_mask=0xF, extra_pins=[]) {

  board_dims = [ board_dims_xy[0], board_dims_xy[1], board_thickness ];
  outside_dims = board_dims + [ 2 * outside_border, 2 * outside_border, base_thickness ];

  // Optionally display the PCBA.
  if (board_stl_file) {
    translate([ board_dims[0] / 2, board_dims[1] / 2, board_dims[2] / 2 + base_thickness ])
        color("white") import(file = board_stl_file, convexity = 3);
  }

  // Tolerance for fitting board and pegs
  space = 0.2;

  module peg(offset) { translate(offset) cylinder(board_thickness + 1, (dscrew - space)/ 2,
						  (dscrew - space)/ 2); }

  translate([ -outside_border, -outside_border, 0 ]) {
    difference() {
      // The outside of the stencil.
      cube(outside_dims);
      // Cut-out for the board.
      translate([ outside_border - space/2, outside_border - space/2, base_thickness + 0.001 ]) {
        cube(board_dims + [ space, space, 0 ]);
      }
      // Cut-out at the bottom of the stencil for to make it easy to remove the board.
      // This must be inside the pegs which hold the board and stencil in place.
      inside_offset = outside_border + screw_offset + dscrew / 2 + outside_border;
      translate([inside_offset, inside_offset, -0.01 ]) {
        cube([
          outside_dims[0] - 2 * inside_offset, outside_dims[1] - 2 * inside_offset,
          base_thickness + 1
        ]);
      }
    }
  }
  translate([0, 0, base_thickness - 0.01 ]) {
    $fn = 20;
    offset = screw_offset + space;
    if (pin_mask & 0x1) { // bottom left
      peg([offset, offset, 0 ]);
    }
    if (pin_mask & 0x2) { // bottom right
      peg([board_dims[0] - offset, offset, 0 ]);
    }
    if (pin_mask & 0x4) { // top right
      peg([board_dims[0] - offset, board_dims[1] - offset, 0 ]);
    }
    if (pin_mask & 0x8) { // top_left
      peg([offset, board_dims[1] - screw_offset, 0 ]);
    }
    for (pin_offset = extra_pins) {
      peg([pin_offset[0], pin_offset[1], 0 ]);
    }
  }
}

// Screw diameter (M2.5)
dscrew = 2.5;
// x/y offset between edge of board and middle of peg to align board and stencil.
screw_offset = 2.35;

// pin_mask bits:
//   0x1: bottom left
//   0x2: bottom right
//   0x4: top right
//   0x8: top_left

pin_mask = 0x1 | 0x4;

board_stl_file = show_vitamins ? "../Garden133/Garden133.stl" : undef;
board_xy = [ 71.12, 39.37];
pad = 2.54;
stencil(board_xy, dscrew, screw_offset, board_stl_file = board_stl_file,
	pin_mask=pin_mask);
