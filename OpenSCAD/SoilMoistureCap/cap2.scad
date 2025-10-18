include<moisture_sensor.scad>
include<ProjectBox/headers.scad>

sensor_cap_length = 40;
sensor_cap_top = 5;

screw_diameter = 3;

gap = 0.4;
wall = 1;

back_pins_length = 2;

connector_dims = connector_nx1_dims(3);

conn_space = 1;
conn_y_extra = 6;

wire_diameter = 3;

// TODO: Add space for screw holes
side_thickness = wall - cut_out_dx + screw_diameter;
top_thickness = wall + connector_dims[2];
bottom_thickness = wall + back_pins_length;
back_thickness = 10;

inner_dims =
    [ moisture_sensor_dims[0] + 2 * gap, sensor_cap_length, moisture_sensor_dims[2] + 2 * gap ];

outer_dims = [
  inner_dims[0] + 2 * side_thickness, inner_dims[1] + 2 * back_thickness,
  inner_dims[2] + top_thickness + bottom_thickness + 2 *
  gap
];

module sensor_space() {
  union() {
    cube(inner_dims);
    translate([ gap, gap, gap ]) cube(moisture_sensor_dims);
    component_z = 1.5;
    translate([ 0, 0, moisture_sensor_dims[2] ])
        cube([ moisture_sensor_dims[0] + gap * 2, 30, component_z + 2 * gap ]);
    translate([ pin_offset[0], pin_offset[1], pin_offset[2] - pin_size[2] + epsilon ])
        cube(pin_size + gap * [ 2, 2, 2 ]);
    // Space for bottom pins.
  }
}

sensor_offset = [ side_thickness + gap, back_thickness + gap, bottom_thickness + gap ];

module cap2(top) {
  $fn = 20;

  connector_space_offset = [
    moisture_sensor_conn_offset[0] - moisture_sensor_conn_dims[0] / 2 - conn_space,
    -1 + back_thickness - conn_space - conn_y_extra, moisture_sensor_conn_offset[2] + 1 -
    conn_space
  ];

  slice_z = bottom_thickness + gap + moisture_sensor_dims[2] / 2;

  module screw_hole() { cylinder(2 + outer_dims[2], screw_diameter / 2, screw_diameter / 2); }

  difference() {
    union() {
      difference() {
        cube(outer_dims);
        translate([ side_thickness, back_thickness, bottom_thickness ]) { sensor_space(); }
        // Screw holes at sensor cut-out.
        translate([
          sensor_offset[0] - (gap + screw_diameter / 2 - cut_out_dx),
          sensor_offset[1] + cut_out_y1 + cut_out_dy / 2, -1
        ]) screw_hole();
        translate([
          sensor_offset[0] + moisture_sensor_dims[0] + (gap + screw_diameter / 2 - cut_out_dx),
          sensor_offset[1] + cut_out_y1 + cut_out_dy / 2, -1
        ]) screw_hole();
        // Screw holes near the top
        translate([ sensor_offset[0] - (gap + screw_diameter / 2 - 3), 4, -1 ]) screw_hole();
        translate(
            [ sensor_offset[0] + moisture_sensor_dims[0] + (gap + screw_diameter / 2 - 3), 4, -1 ])
            screw_hole();

        // Space for connector
        translate(connector_space_offset) {
          cube(moisture_sensor_conn_dims +
               [ conn_space * 2, conn_space * 2 + conn_y_extra, conn_space * 2 ]);
        }
        // Cut-off either the top or the bottom.
        if (top) {
          translate([ -1, -1, -1 ]) cube([ outer_dims[0] + 2, outer_dims[1] + 2, slice_z + 1 ]);
        } else {
          translate([ -1, -1, slice_z ]) cube(outer_dims + [ 10, 10, 10 ]);
        }
        if (top) {
          translate([ connector_space_offset[0], -1, slice_z - 1 ])
              cube([ moisture_sensor_conn_dims[0] + conn_space * 2, 8, 4 ]);
        }
      }
      // Adding some material to stop the connector from disconnecting.
      if (top) {
        translate(connector_space_offset - [ 0, 0, 4 + epsilon - moisture_sensor_conn_dims[2] ]) {
          cube([ 2, 4, 6 ]);
        }
        translate(connector_space_offset - [ 0, 0, 4 + epsilon - moisture_sensor_conn_dims[2] ] +
                  [ moisture_sensor_conn_dims[0], 0, 0 ]) {
          cube([ 2, 4, 6 ]);
        }
      }
      if (!top) {
        translate([ connector_space_offset[0], -epsilon, slice_z - 1 ])
            cube([ moisture_sensor_conn_dims[0] + conn_space * 2, 2, 4 ]);
      }
    }
    // Hole for cable at head.
    translate(connector_space_offset + [
      (moisture_sensor_conn_dims[0] + conn_space * 2) / 2, 2, moisture_sensor_conn_dims[2] / 2 +
      conn_space
    ]) rotate([ 90, 0, 0 ]) cylinder(20, wire_diameter / 2, wire_diameter / 2);
  }
}
