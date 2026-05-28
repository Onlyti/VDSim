#include "vdsim/version.hpp"

#include <sstream>

namespace vdsim {

std::string version_string() {
    std::ostringstream ss;
    ss << VERSION_MAJOR << '.' << VERSION_MINOR << '.' << VERSION_PATCH;
    return ss.str();
}

}  // namespace vdsim
