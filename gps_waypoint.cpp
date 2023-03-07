#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <dronecode_sdk/dronecode_sdk.h>

using namespace dronecode_sdk;

int main(int argc, char** argv) {
  // Initialize Dronecode SDK
  DronecodeSDK dc;
  ConnectionResult connection_result = dc.add_udp_connection("localhost");
  if (connection_result != ConnectionResult::SUCCESS) {
    std::cerr << "Connection failed: " << connection_result_str(connection_result) << std::endl;
    return 1;
  }

  // Wait for system to connect
  std::cout << "Waiting for system connection..." << std::endl;
  while (!dc.is_connected()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  // Get system and telemetry objects
  System &system = dc.system();
  Telemetry &telemetry = system.telemetry();

  // Wait for GPS fix
  std::cout << "Waiting for GPS fix..." << std::endl;
  while (telemetry.health_all_ok() && !telemetry.gps_fix()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  // Set target GPS location
  Telemetry::Position target_position;
  target_position.latitude_deg = 47.617444; // Target latitude in degrees
  target_position.longitude_deg = -122.201208; // Target longitude in degrees
  target_position.absolute_altitude_m = 50.0; // Target altitude in meters above sea level

  // Calculate direction and distance to target location
  Telemetry::Position current_position = telemetry.position();
  double distance_to_target = std::sqrt(std::pow(target_position.latitude_deg - current_position.latitude_deg, 2) +
                                        std::pow(target_position.longitude_deg - current_position.longitude_deg, 2));
  double bearing_to_target = std::atan2(target_position.longitude_deg - current_position.longitude_deg,
                                        target_position.latitude_deg - current_position.latitude_deg) * 180.0 / M_PI;
  if (bearing_to_target < 0) {
    bearing_to_target += 360.0;
  }

  // Set max speed to 5 m/s
  const float max_speed_m_s = 5.0f;

  // Start moving towards target location
  while (distance_to_target > 1.0) {
    // Calculate direction and distance to target location
    current_position = telemetry.position();
    distance_to_target = std::sqrt(std::pow(target_position.latitude_deg - current_position.latitude_deg, 2) +
                                   std::pow(target_position.longitude_deg - current_position.longitude_deg, 2));
    bearing_to_target = std::atan2(target_position.longitude_deg - current_position.longitude_deg,
                                   target_position.latitude_deg - current_position.latitude_deg) * 180.0 / M_PI;
    if (bearing_to_target < 0) {
      bearing_to_target += 360.0;
    }

    // Calculate desired velocity vector
    float speed_m_s = std::min(distance_to_target, max_speed_m_s);
    Telemetry::VelocityNED velocity;
    velocity.north_m_s = speed_m_s * std::sin(bearing_to_target * M_PI / 180.0);
    velocity.east_m_s = speed_m_s * std::cos(bearing_to_target * M_PI / 180.0);   // Send velocity command to quadcopter
    const float yaw_deg = 0.0f; // Keep yaw angle constant
    const float yaw_rate_deg_s = 0.0f; // Keep yaw rate zero
    const bool is_yaw_rate = false; // Set to false for yaw angle control
    const bool is_horizontal_velocity = true; // Set to true for horizontal velocity control
    const bool is_vertical_velocity = false; // Set to false for altitude control
    const uint64_t timeout_ms = 100; // Set timeout for command to 100 ms
    const Telemetry::VelocityBodyYawspeed command(velocity, yaw_deg, yaw_rate_deg_s, is_yaw_rate,
                                                  is_horizontal_velocity, is_vertical_velocity);
    telemetry.set_velocity_body_yawspeed(command);

    // Sleep for 100 ms before checking position again
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

// Stop moving and land
  const uint64_t land_timeout_ms = 30000; // Set timeout for landing to 30 seconds
  telemetry.land_async(Telemetry::LandingGear::DOWN_ONLY, land_timeout_ms);

// Wait for landing to complete
  while (telemetry.in_air()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

// Disarm motors
  system.action().disarm();

// Cleanup Dronecode SDK
  dc.remove_udp_connection();
  return 0;
}