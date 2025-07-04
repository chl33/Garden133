include <moisture_sensor.scad>
include <ProjectBox/headers.scad>

// moisture_sensor_dims = [22.5, 92, 1.5];

sensor_cap_length = 20;
sensor_cap_top = 5;

screw_diameter = 3;

gap = 0.4;
wall = 1;

back_pins_length = 2;

connector_dims = connector_nx1_dims(3);

conn_space = 1;
conn_y_extra = 2;

wire_diameter = 3;

// TODO: Add space for screw holes
side_thickness =  wall - cut_out_dx + screw_diameter;
top_thickness = wall + connector_dims[2];
bottom_thickness = wall + back_pins_length;
back_thickness = wall + 2 * conn_space + conn_y_extra;

inner_dims = [moisture_sensor_dims[0] + 2 * gap,
	      sensor_cap_length,
	      moisture_sensor_dims[2] + 2 * gap];

outer_dims = [inner_dims[0] + 2 * side_thickness,
	      inner_dims[1] + 2 * back_thickness,
	      inner_dims[2] + top_thickness + bottom_thickness + 2 * gap];

module sensor_space() {
  union() {
    cube(inner_dims);
    translate([gap, gap, gap]) cube(moisture_sensor_dims);
    component_z = 1.5;
    translate([0, 0, moisture_sensor_dims[2]])
      cube([moisture_sensor_dims[0]+gap*2, 30, component_z+2*gap]);
    translate([pin_offset[0], pin_offset[1], pin_offset[2] - pin_size[2] + epsilon])
      cube(pin_size + gap*[2,2,2]);
    // Space for bottom pins.
  }
}


sensor_offset = [side_thickness + gap, back_thickness + gap, bottom_thickness + gap];


module cap2(top, bottom) {
  $fn = 20;
  difference() {
    cube(outer_dims);
    translate([side_thickness, back_thickness, bottom_thickness]) {
      sensor_space();
    }
    // Screw holes at sensor cut-out.
    translate([sensor_offset[0] - (gap + screw_diameter/2 - cut_out_dx),
	       sensor_offset[1] + cut_out_y1 + cut_out_dy / 2,
	       -1])
      cylinder(2 + outer_dims[2], screw_diameter/2, screw_diameter/2);
    translate([sensor_offset[0] + moisture_sensor_dims[0] + (gap + screw_diameter/2 - cut_out_dx),
	       sensor_offset[1] + cut_out_y1 + cut_out_dy / 2,
	       -1])
      cylinder(2 + outer_dims[2], screw_diameter/2, screw_diameter/2);
    // Space for connector
    translate([moisture_sensor_conn_offset[0], -1 + back_thickness, moisture_sensor_conn_offset[2]+1]
	      + [-moisture_sensor_conn_dims[0] / 2 - conn_space,
		 -conn_space - conn_y_extra,
		 -conn_space]) {
      cube(moisture_sensor_conn_dims + [conn_space*2, conn_space*2 + conn_y_extra, conn_space*2]);
      translate([(moisture_sensor_conn_dims[0] + conn_space*2)/2, 2,
		 moisture_sensor_conn_dims[2]/2 + conn_space])
	rotate([90, 0, 0])
	cylinder(20, wire_diameter/2, wire_diameter/2);
    }
    slice_z = bottom_thickness + gap + moisture_sensor_dims[2]/2;
    if (!top) {
	translate([-1, -1, slice_z]) cube(outer_dims + [10, 10, 10]);
    }
    if (!bottom) {
      translate([-1, -1, -1])
	cube([outer_dims[0] + 2,
	      outer_dims[1] + 2,
	      slice_z + 1]);
    }
  }
}
