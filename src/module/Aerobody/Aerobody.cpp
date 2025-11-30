#include "Aerobody.h"
#include <cmath>

ApparentWindVector Aerobody::get_wind(const VelocityVector& reported_wind, const VelocityVector& car_velocity) {
     
    VelocityVector true_wind = VelocityVector::from_cartesian_components(
        -reported_wind.get_north_south(),
        -reported_wind.get_east_west()
    );
    
     
    VelocityVector apparent_wind_velocity = VelocityVector::from_cartesian_components(
        true_wind.get_north_south() - car_velocity.get_north_south(),
        true_wind.get_east_west() - car_velocity.get_east_west()
    );
    
     
     
    VelocityVector apparent_wind_direction = VelocityVector::from_cartesian_components(
        -apparent_wind_velocity.get_north_south(),
        -apparent_wind_velocity.get_east_west()
    );
    
     
    double yaw = car_velocity.angle_between(apparent_wind_direction);
    
    return ApparentWindVector{
        .speed = apparent_wind_velocity.get_magnitude(),
        .yaw = yaw
    };
}

double Aerobody::aerodynamic_drag(const ApparentWindVector& apparent_wind, double air_density) const {
     
     
    double wind_component = apparent_wind.speed * std::cos(apparent_wind.yaw);
    return 0.5 * air_density * wind_component * wind_component * drag_coefficient * frontal_area;
}