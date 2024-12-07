#include "helper_func.hpp"

// If the input voltage is higher or lower than 
double clamp(double input, double min, double max){
  if( input > max ) return(max);
  if(input < min) return(min);
  return(input);
}

/// @brief 
/// @param rad 
/// @return 
double radToDeg(double rad){ return rad * 180 / M_PI; }

/// @brief 
/// @param deg 
/// @return 
double degToRad(double deg){ return deg * M_PI / 180; }

/// @brief 
/// @param percent 
/// @return 
float to_volt(float percent){ return(percent*12.0/100.0); }

/// @brief 
/// @param angle 
/// @return 
float reduce_0_to_360(float angle) {
  while(!(angle >= 0 && angle < 360)) {
    if( angle < 0 ) { angle += 360; }
    if(angle >= 360) { angle -= 360; }
  }
  return(angle);
}


/// @brief 
/// @param angle 
/// @return 
float reduce_negative_180_to_180(float angle) {
  while(!(angle >= -180 && angle < 180)) {
    if( angle < -180 ) { angle += 360; }
    if(angle >= 180) { angle -= 360; }
  }
  return(angle);
}

/// @brief 
/// @param inches 
/// @return 
double inchesToCm(double inches){ return inches*2.54; }

/// @brief 
/// @param cm 
/// @return 
double cmToInches(double cm){ return cm/2.54; }

/// @brief 
/// @param angle 
/// @return 
double getHeading(double angle){ return reduce_0_to_360(angle); }