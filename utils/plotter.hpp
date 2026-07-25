#ifndef UTILS__PLOTTER_HPP
#define UTILS__PLOTTER_HPP

#include <mutex>
#include <string>

#include <netinet/in.h>  // sockaddr_in
#include <nlohmann/json.hpp>

namespace tools {
class Plotter {
 public:
  Plotter(std::string host = "127.0.0.1", uint16_t port = 9870);

  ~Plotter();

  void plot(const nlohmann::json& json);

 private:
  int socket_;
  sockaddr_in destination_;
  std::mutex mutex_;
};

}  // namespace tools

#endif  // UTILS__PLOTTER_HPP
