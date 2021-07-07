#include "processor.h"

namespace Seiscomp {
namespace detect {

Processor::Processor(const std::string &id) : id_{id} {}

Processor::~Processor() {}

const std::string &Processor::id() const { return id_; }

}  // namespace detect
}  // namespace Seiscomp
