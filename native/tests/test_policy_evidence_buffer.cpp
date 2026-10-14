#include "../src/policy_evidence_buffer.hpp"
#include <cassert>
#include <sstream>
int main() {
  sprite_policy::EvidenceBuffer buffer(4);
  std::ostream stream(&buffer);
  stream.write("1234",4);
  assert(stream.good());
  stream.put('5');
  assert(stream.fail());
  std::ostringstream saved;
  buffer.save(saved);
  assert(saved.str()=="1234");
  bool rejected=false;
  saved.setstate(std::ios::badbit);
  try {buffer.save(saved);} catch(const std::runtime_error&) {rejected=true;}
  assert(rejected);
}
