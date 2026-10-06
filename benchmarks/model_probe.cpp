#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>

#include "motion_fixture.hpp"
#include "swerve_mppi/common/config_profile.hpp"
using namespace swerve_mppi;
namespace
{
const char * status_name(MotionStatus status)
{
  switch (status) {
    case MotionStatus::Invalid:
      return "invalid";
    case MotionStatus::Missing:
      return "missing";
    case MotionStatus::Unsynchronized:
      return "unsynchronized";
    case MotionStatus::CorrelatedSource:
      return "correlated_source";
    case MotionStatus::NominalAgreement:
      return "nominal_agreement";
    case MotionStatus::BoundedDisagreement:
      return "bounded_disagreement";
    case MotionStatus::EnvelopeExceeded:
      return "envelope_exceeded";
  }
  throw std::runtime_error("Unknown motion status");
}
const char * mode_name(DriveMode mode)
{
  return mode == DriveMode::DualAckermann ? "ackermann" : mode == DriveMode::Crab ? "crab" : "spin";
}
}  // namespace
int main(int argc, char ** argv)
{
  try {
    Config c;
    bool print_config = false;
    bool config_seen = false;
    for (int i = 1; i < argc; ++i) {
      const std::string option = argv[i];
      if (option == "--print-config" && !print_config) {
        print_config = true;
      } else if (option == "--config" && !config_seen && i + 1 < argc) {
        config_seen = true;
        std::ifstream stream(argv[++i]);
        if (!stream) {
          throw std::runtime_error("Cannot read profile");
        }
        std::string profile(65537, '\0');
        stream.read(profile.data(), static_cast<std::streamsize>(profile.size()));
        if (stream.bad()) {
          throw std::runtime_error("Cannot read profile");
        }
        profile.resize(static_cast<std::size_t>(stream.gcount()));
        c = parse_config_profile(profile);
      } else {
        throw std::invalid_argument(
          "Usage: swerve_mppi_model_probe [--config file] [--print-config]");
      }
    }
    std::cout.imbue(std::locale::classic());
    std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);
    if (print_config) {
      std::cout << write_config_profile(c);
    } else {
      const auto rows = test::run_motion_probe(c);
      std::cout
        << "schema_version,mode,perturbation,tick,phase,encoder_stamp_ns,body_stamp_ns,"
           "position_error_m,yaw_error_rad,encoder_vx,encoder_vy,encoder_wz,physical_vx,physical_"
           "vy,"
           "physical_wz,observed_vx,observed_vy,observed_wz,linear_bound_mps,angular_bound_radps,"
           "linear_residual_mps,angular_residual_radps,linear_residual_upper_mps,angular_residual_"
           "upper_radps,"
           "body_speed_upper_mps,body_rate_upper_radps,status,body_stationary,encoder_stationary,"
           "encoded_model_valid,observed_model_valid\n";
      for (const auto & row : rows) {
        const auto & a = row.assessment;
        const auto & o = row.observed;
        std::cout << 1 << ',' << mode_name(row.mode) << ',' << row.perturbation << ',' << row.tick
                  << ',' << (row.braking ? "brake" : "drive") << ',' << row.encoder_stamp_ns << ','
                  << o.stamp_ns << ',' << row.prediction_position_error_m << ','
                  << row.prediction_yaw_error_rad << ',' << row.encoded.velocity.vx << ','
                  << row.encoded.velocity.vy << ',' << row.encoded.velocity.wz << ','
                  << row.physical_velocity.vx << ',' << row.physical_velocity.vy << ','
                  << row.physical_velocity.wz << ',' << o.velocity.vx << ',' << o.velocity.vy << ','
                  << o.velocity.wz << ',' << o.linear_error_bound_mps << ','
                  << o.angular_error_bound_radps << ',' << a.linear_residual_mps << ','
                  << a.angular_residual_radps << ',' << a.linear_residual_upper_mps << ','
                  << a.angular_residual_upper_radps << ',' << a.body_speed_upper_mps << ','
                  << a.body_rate_upper_radps << ',' << status_name(a.status) << ','
                  << a.body_stationary << ',' << a.encoder_stationary << ','
                  << row.encoded_model_valid << ',' << row.observed_model_valid << '\n';
      }
    }
    if (!std::cout) {
      throw std::runtime_error("Cannot write probe output");
    }
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
