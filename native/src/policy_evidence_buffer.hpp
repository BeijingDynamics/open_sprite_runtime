#pragma once
#include <ostream>
#include <streambuf>
#include <vector>
#include <stdexcept>
namespace sprite_policy {
// Allocated before IPC startup. Overflow fails instead of growing during control.
class EvidenceBuffer : public std::streambuf {
  std::vector<char> bytes_;
 public:
  explicit EvidenceBuffer(std::size_t size) : bytes_(size) {
    if(!size) throw std::runtime_error("empty evidence buffer");
    setp(bytes_.data(),bytes_.data()+bytes_.size());
  }
  void save(std::ostream& output) {
    if(!output.write(pbase(),pptr()-pbase())) throw std::runtime_error("evidence save failed");
  }
};
}
