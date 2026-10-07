#include <charconv>
#include <cmath>
#include <fstream>
#include <iostream>
#include <locale>
#include <map>
#include <sstream>
#include <stdexcept>

#include "swerve_mppi/integration/adapter_contract.hpp"

// Offline tooling only: no transport, runtime arming or simulator dependencies.
namespace
{
using namespace swerve_mppi;
std::string read(const char * path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::invalid_argument(std::string("cannot read: ") + path);
  }
  std::string result;
  for (char c; input.get(c);) {
    if (result.size() == 65536) {
      throw std::invalid_argument("input exceeds 64 KiB");
    }
    result += c;
  }
  if (!input.eof()) {
    throw std::invalid_argument("input read failed");
  }
  return result;
}
std::string trim(std::string text)
{
  const auto first = text.find_first_not_of(" \t\r\n");
  return first == std::string::npos
           ? ""
           : text.substr(first, text.find_last_not_of(" \t\r\n") - first + 1);
}
std::uint64_t integer(const std::string & text)
{
  std::uint64_t value = 0;
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
  if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size()) {
    throw std::invalid_argument("invalid unsigned decimal integer: " + text);
  }
  return value;
}
double real(const std::string & text)
{
  double value = 0;
  std::istringstream input(text);
  input.imbue(std::locale::classic());
  input >> std::noskipws >> value;
  if (!input || !input.eof() || !std::isfinite(value)) {
    throw std::invalid_argument("invalid finite real: " + text);
  }
  return value;
}
AdapterMetadata metadata(const char * path)
{
  std::map<std::string, std::string> fields;
  std::istringstream input(read(path));
  for (std::string line; std::getline(input, line);) {
    line = trim(line.substr(0, line.find('#')));
    if (line.empty()) {
      continue;
    }
    const auto equal = line.find('=');
    if (
      equal == std::string::npos || line.find('=', equal + 1) != std::string::npos ||
      !fields.emplace(trim(line.substr(0, equal)), trim(line.substr(equal + 1))).second) {
      throw std::invalid_argument("invalid or duplicate metadata field");
    }
  }
  const auto take = [&](const char * name) {
    const auto found = fields.find(name);
    if (found == fields.end() || found->second.empty()) {
      throw std::invalid_argument(std::string("missing metadata: ") + name);
    }
    const auto value = found->second;
    fields.erase(found);
    return value;
  };
  AdapterMetadata m;
  // Validate before narrowing, so e.g. 2^32+1 cannot masquerade as schema 1.
  if (integer(take("schema_version")) != 1) {
    throw std::invalid_argument("unsupported metadata schema");
  }
  m.schema_version = 1;
  m.world_frame = take("world_frame");
  m.body_frame = take("body_frame");
  m.clock = {take("clock_domain"), integer(take("clock_epoch"))};
  const auto policy = take("motion_policy");
  if (policy == "NominalEncoderOnly") {
    m.motion_policy = MotionEvidencePolicy::NominalEncoderOnly;
  } else if (policy == "IndependentNominal") {
    m.motion_policy = MotionEvidencePolicy::IndependentNominal;
  } else {
    throw std::invalid_argument("unsupported motion_policy");
  }
  m.timing = TimingLimits{
    real(take("max_feedback_age_s")), real(take("max_command_age_s")),
    real(take("period_tolerance_ratio"))};
  m.watchdog_s = real(take("watchdog_s"));
  if (!fields.empty()) {
    throw std::invalid_argument("unknown metadata: " + fields.begin()->first);
  }
  return m;
}
}  // namespace
int main(int argc, char ** argv)
{
  try {
    const std::string operation = argc > 1 ? argv[1] : "";
    if (operation == "schema" && argc == 2) {
      std::cout << "{\"tool_version\":\"" << SWERVE_MPPI_TOOL_VERSION << "\",\"fields\":[";
      bool first = true;
      for (const auto & field : swerve_mppi::config_schema()) {
        std::cout << (first ? "" : ",") << "{\"name\":\"" << field.name << "\",\"unit\":\""
                  << field.unit << "\",\"scope\":\""
                  << (field.scope == swerve_mppi::ConfigScope::Execution ? "execution" : "planning")
                  << "\",\"type\":\""
                  << (field.type == swerve_mppi::ConfigValueType::Real ? "real"
                                                                       : "unsigned_integer")
                  << "\"}";
        first = false;
      }
      std::cout << "]}\n";
    } else if (operation == "resolve" && argc == 4) {
      const auto base = swerve_mppi::parse_resolved_config_profile(read(argv[2]));
      const auto config = swerve_mppi::parse_config_profile(read(argv[3]), base);
      swerve_mppi::validate_live_config(config);
      std::cout << swerve_mppi::write_config_profile(config);
    } else if (operation == "compare" && argc == 6) {
      const auto left_metadata = metadata(argv[3]);
      const auto right_metadata = metadata(argv[5]);
      const swerve_mppi::AdapterContract left(read(argv[2]), left_metadata);
      const swerve_mppi::AdapterContract right(read(argv[4]), right_metadata);
      swerve_mppi::require_adapter_compatible(left, right);
      std::cout << "compatible startup candidates; no runtime or physical acceptance\n";
    } else {
      throw std::invalid_argument(
        "usage: config_preflight schema | resolve BASE OVERRIDES | "
        "compare LEFT_PROFILE LEFT_METADATA RIGHT_PROFILE RIGHT_METADATA");
    }
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "preflight rejected: " << e.what() << '\n';
    return 1;
  }
}
