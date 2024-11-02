#include "helper_func.hpp"

// If the input voltage is higher or lower than 
double clamp(double input, double min, double max){
  if( input > max ) return(max);
  if(input < min) return(min);
  return(input);
}

double radToDeg(double rad){
  return rad * 180 / M_PI;
}

double degToRad(double deg){
  return deg * M_PI / 180;
}

float to_volt(float percent){
  return(percent*12.0/100.0);
}

float reduce_0_to_360(float angle) {
  while(!(angle >= 0 && angle < 360)) {
    if( angle < 0 ) { angle += 360; }
    if(angle >= 360) { angle -= 360; }
  }
  return(angle);
}

float reduce_negative_180_to_180(float angle) {
  while(!(angle >= -180 && angle < 180)) {
    if( angle < -180 ) { angle += 360; }
    if(angle >= 180) { angle -= 360; }
  }
  return(angle);
}