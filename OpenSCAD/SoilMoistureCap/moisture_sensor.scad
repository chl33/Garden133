// Copyright (c) 2024 Chris Lee and contibuters.
// Licensed under the MIT license. See LICENSE file in the project root for details.

include <MCAD/units.scad>
include <ProjectBox/headers.scad>

moisture_sensor_dims = [22.5, 92, 1.5];

pin_offset = [6, 4, 0];
pin_size = [10, 2, 2];

cut_out_y1 = 18;
cut_out_dy1 = 1;
cut_out_dy2 = 3;
cut_out_dy = 2 * cut_out_dy1 + cut_out_dy2;
cut_out_dx = 1.75;

moisture_sensor_conn_dims = connector_nx1_dims(3);
moisture_sensor_conn_offset = 
  -header_offset
  + [moisture_sensor_dims[0] / 2,
     moisture_sensor_dims[2] - 1,
     moisture_sensor_dims[2] + moisture_sensor_conn_dims[1] / 2];


module moisture_sensor(space=false) {
  dims = moisture_sensor_dims + Z * (space ? 0.5 : 0);

  offset_val = space ? 0.5 : 0;

  msdx = moisture_sensor_dims[0];
  points = [[0, 0],      // fl
	    [0, cut_out_y1],     // l cut-out fl
	    [cut_out_dx, cut_out_y1 + cut_out_dy1],   // l cut-out fr
	    [cut_out_dx, cut_out_y1 + cut_out_dy1 + cut_out_dy2],   // l cut-out hr
	    [0, cut_out_y1 + cut_out_dy],     // l cut-out hl
	    [0, 28],     // l mid
	    [0, 85],     // hl
	    [msdx/2, moisture_sensor_dims[1]],    // point
	    [msdx, 85],    // hr
	    [msdx, 28],    // r mid
	    [msdx, cut_out_y1 + 2*cut_out_dy1 + cut_out_dy2],    // r cut-out hr
	    [msdx-cut_out_dx, cut_out_y1 + cut_out_dy1 + cut_out_dy2],  // r cut-out hl
	    [msdx-cut_out_dx, cut_out_dy],  // r cut-out fl
	    [msdx, cut_out_y1],    // r cut-out rr
	    [msdx, 0]];    // fr

  translate([1, 1, -1]*offset_val) {
    color("green") {
      linear_extrude(dims[2]+2*offset_val) {
	offset(offset_val)
	  polygon(points = points, paths = [[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]]);
      }
      linear_extrude(2*dims[2]+2*offset_val) {
	offset(offset_val)
	  polygon(points = points, paths = [[0, 1, 2, 3, 4, 5, 9, 10, 11, 12, 13, 14]]);
      }

      component_z = 1.5;
      translate([1, 3, dims[2]-epsilon]) cube([20, 15, component_z]);
      translate([2, 18-epsilon, dims[2]-epsilon]) cube([18, 5, component_z]);
    }
    conn_dims = moisture_sensor_conn_dims;
    if (space) {
      // Reserve simple space for connector when using difference to make space for sensor.
      gap = 0.3;
      space_dims = gap*[2,1,2] + conn_dims;
      color("white") {
	translate([(dims[0]-space_dims[0])/2, -6, dims[2]-0.6-1.4])
	  cube([space_dims[0], space_dims[2]+4.5, space_dims[1]+4]);
      }
    } else {
      translate(moisture_sensor_conn_offset) {
	rotate(180, Y) rotate(90, X) connector_nx1(3, pin_extend=false);
      }
    }
    // Location of pins on the back
    color("white")
      translate([pin_offset[0], pin_offset[1], pin_offset[2] - pin_size[2] + epsilon])
      cube(pin_size);
  }
}
