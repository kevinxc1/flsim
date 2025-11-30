#include "Aerobody.h"
#include <cmath>

ApparentWindVector Aerobody::get_wind(const VelocityVector& reported_wind, const VelocityVector& car_velocity) {
    // Convert reported wind (where wind comes from) to true wind (where wind goes to)
    VelocityVector true_wind = VelocityVector::from_cartesian_components(
        -reported_wind.get_north_south(),
        -reported_wind.get_east_west()
    );
    
    // Calculate apparent wind velocity relative to the car
    VelocityVector apparent_wind_velocity = VelocityVector::from_cartesian_components(
        true_wind.get_north_south() - car_velocity.get_north_south(),
        true_wind.get_east_west() - car_velocity.get_east_west()
    );
    
    // The apparent wind direction (where wind is coming from, relative to car)
    // is opposite to the apparent wind velocity
    VelocityVector apparent_wind_direction = VelocityVector::from_cartesian_components(
        -apparent_wind_velocity.get_north_south(),
        -apparent_wind_velocity.get_east_west()
    );
    
    // Calculate yaw angle from car's heading to apparent wind direction
    double yaw = car_velocity.angle_between(apparent_wind_direction);
    
    return ApparentWindVector{
        .speed = apparent_wind_velocity.get_magnitude(),
        .yaw = yaw
    };
}

double Aerobody::aerodynamic_drag(const ApparentWindVector& apparent_wind, double air_density) const {
    // Drag force formula: F = 0.5 * ρ * v² * Cd * A * cos²(yaw)
    // cos(yaw) gives the component of wind opposing the car's motion
    double wind_component = apparent_wind.speed * std::cos(apparent_wind.yaw);
    return 0.5 * air_density * wind_component * wind_component * drag_coefficient * frontal_area;
}