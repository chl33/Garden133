epsilon = 0.001;

board_thickness = 1.6;
stencil_dims = [155, 100, 1];
outside = 3;
inside_lip = 10;

module stencil_frame() {
  frame_dims = stencil_dims + [2*outside, 2*outside, board_thickness];
  
  difference() {
    // Frame body
    cube(frame_dims);
    // Cutout for board
    translate([inside_lip, inside_lip, -epsilon]) {
      cube([frame_dims[0]-2*inside_lip, frame_dims[1]-2*inside_lip, frame_dims[2]+1]);
    }
    // Cutout for stencil
    translate([outside, outside,board_thickness]) {
      cube([frame_dims[0]-2*outside, frame_dims[1]-2*outside, frame_dims[2]+1]);
    }
  }
}

stencil_frame();
