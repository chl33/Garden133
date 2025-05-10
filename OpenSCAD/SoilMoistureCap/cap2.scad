include <moisture_sensor.scad>
include <ProjectBox/headers.scad>

// moisture_sensor_dims = [22.5, 92, 1.5];

sensor_cap_length = 23;
sensor_cap_top = 5;

screw_diameter = 3;

gap = 0.4;
wall = 2;

back_pins_length = 2;

connector_dims = connector_nx1_dims(3);

conn_space = 1;
conn_y_extra = 8;

wire_diameter = 3;

// TODO: Add space for screw holes
side_thickness =  wall - cut_out_dx + screw_diameter;
top_thickness = 2 * wall + connector_dims[2] + 1;
bottom_thickness = wall + back_pins_length;
back_thickness = wall + 2 * conn_space + conn_y_extra;

inner_dims = [moisture_sensor_dims[0] + 2 * gap,
	      sensor_cap_length,
	      moisture_sensor_dims[2] + 2 * gap];

outer_dims = [inner_dims[0] + 2 * side_thickness,
	      inner_dims[1] + 2 * back_thickness,
	      inner_dims[2] + top_thickness + bottom_thickness];

module sensor_space() {
  union() {
    cube(inner_dims);
    translate(gap*[1,1,1]) cube(moisture_sensor_dims);
    component_z = 1.5;
    translate([0, 0, moisture_sensor_dims[2]])
      cube([moisture_sensor_dims[0]+gap*2, 30, component_z+2*gap]);
    // Space for bottom pins.
    translate([pin_offset[0], pin_offset[1], pin_offset[2] - pin_size[2] + epsilon])
      cube(pin_size + gap*[2,2,2]);
  }
}


sensor_offset = [side_thickness + gap, back_thickness + gap, bottom_thickness + gap];

module cap2(top) {
  $fn = 20;

  slice_z = bottom_thickness + gap + moisture_sensor_dims[2]/2;

  module lip(top) {
    eps = top ? 1e-4 : -1e-4;
    lip_height = 1;
    translate([-epsilon, wall, slice_z - lip_height + epsilon])
      cube([wall, outer_dims[1]-wall+epsilon, lip_height + eps]);
    translate([outer_dims[0]-wall-eps, wall, slice_z - lip_height + epsilon])
      cube([wall, outer_dims[1]-wall+epsilon, lip_height + eps]);
  }

  module screw_hole() {
    cylinder(outer_dims[2]-1, screw_diameter/2, screw_diameter/2);
  }

  difference() {
    cube(outer_dims);
    translate([side_thickness, back_thickness, bottom_thickness]) {
      sensor_space();
    }
    // Screw holes near top (away from point) of sensor
    translate([wall + 1 + 2*gap, bottom_thickness + wall, -1]) screw_hole();
    translate([outer_dims[0] - (wall + 1 + 2*gap), bottom_thickness + wall, -1]) screw_hole();
    // Screw holes at sensor cut-out.
    translate([sensor_offset[0] - (gap + screw_diameter/2 - cut_out_dx),
	       sensor_offset[1] + cut_out_y1 + cut_out_dy / 2,
	       -1])
      screw_hole();
    translate([sensor_offset[0] + moisture_sensor_dims[0] + (gap + screw_diameter/2 - cut_out_dx),
	       sensor_offset[1] + cut_out_y1 + cut_out_dy / 2,
	       -1])
      screw_hole();
    // Space for connector
    translate([moisture_sensor_conn_offset[0] + 1,
	       -1 + back_thickness,
	       moisture_sensor_conn_offset[2]+wall]
	      + [-moisture_sensor_conn_dims[0] / 2 - conn_space,
		 -conn_space - conn_y_extra,
		 -conn_space]) {
      cube(moisture_sensor_conn_dims
	   + [conn_space*2 - 1, conn_space*2 + conn_y_extra + gap, conn_space*2 + 1]);
      translate([(moisture_sensor_conn_dims[0] + conn_space*2)/2, 2,
		 moisture_sensor_conn_dims[2]/2 + conn_space])
	rotate([90, 0, 0])
	cylinder(20, wire_diameter/2, wire_diameter/2);
    }
    wire_z = (moisture_sensor_conn_offset[2] + wall + moisture_sensor_conn_dims[2]/2);
    if (!top) {
      translate([-1, wall+epsilon, slice_z]) cube(outer_dims + [10, 10, 10]);
      translate([-1, -1, wire_z]) cube(outer_dims + [10, 1+wall+epsilon, 10]);
      lip(top);
    } else {
      translate([-1, wall+epsilon, -1])
	cube([outer_dims[0] + 2,
	      outer_dims[1] + 2,
	      slice_z + 1]);
      translate([-1, -1, -1])
	cube([outer_dims[0] + 2,
	      1 + wall + 2*epsilon,
	      wire_z + 1]);
    }
  }
  if (top) {
    lip(top);
  }
}
