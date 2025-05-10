include <cap2.scad>

/* [Customization] */
// Whether to cut-off the top of the box
top = false;
bottom = false;
show_vitamins = false;

if (top) {
  cap2(top=true);
}
if( bottom) {
  cap2(top=false);
}
if (show_vitamins) {
  translate(sensor_offset) moisture_sensor();
}
