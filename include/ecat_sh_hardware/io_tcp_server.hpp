#ifndef ECAT_SH_HARDWARE_IO_TCP_SERVER_HPP
#define ECAT_SH_HARDWARE_IO_TCP_SERVER_HPP

#include <string>
#include <vector>
#include <map>
#include <optional>
#include <chrono>
#include <atomic>
#include <tuple>
#include <string_view>
#include <nlohmann/json.hpp>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <queue>
#include <mutex>
#include <condition_variable>

#include <iostream>

namespace ecat_sh_hardware {

struct IoRequest
{
  public:

  enum class RequestType {
    READ,
    WRITE
  };

  using Request = std::tuple<int, RequestType, std::optional<int>>;

  std::chrono::time_point<std::chrono::system_clock> timestamp;
  std::vector<Request> requests; 

  static std::string toJsonStr(const IoRequest& data) {
    nlohmann::json j;
    j["timestamp"] = std::chrono::system_clock::to_time_t(data.timestamp);
    for (const auto& [key, type, value] : data.requests) {
      nlohmann::json req;
      req["key"] = key;
      req["type"] = type == RequestType::READ ? "READ" : "WRITE";
      if (value) {
        req["value"] = *value;
      }
      j["requests"].push_back(req);
    }
    return j.dump();
  }

  static std::optional<IoRequest> fromStr(const std::string& str) {
    nlohmann::json j = nlohmann::json::parse(str);
    if(j.empty())
    {
      return std::nullopt;
    }

    IoRequest data;

    data.timestamp = std::chrono::system_clock::from_time_t(j["timestamp"].get<std::time_t>());
    for (const auto& req : j["requests"]) {
      RequestType type = req["type"] == "READ" ? RequestType::READ : RequestType::WRITE;
      std::optional<int> value;
      if (req.find("value") != req.end()) {
        value = req["value"].get<int>();
      }
      data.requests.push_back({req["key"].get<int>(), type, value});
    }
    return data;
  }

};

struct IoResponse
{

  public:
  std::chrono::time_point<std::chrono::system_clock> timestamp;
  using Response = std::tuple<int, int>;
  std::vector<Response> responses;

  static std::string toJsonStr(const IoResponse& data) {
    nlohmann::json j;
    j["timestamp"] = std::chrono::system_clock::to_time_t(data.timestamp);
    for(const auto& resp : data.responses)
    {
      j["responses"][std::get<0>(resp)] = std::get<1>(resp);
    }
    return j.dump();
  }

/*   static std::optional<IoResponse> fromStr(const std::string& str) {
    nlohmann::json j = nlohmann::json::parse(str);
    if(j.empty())
    {
      return std::nullopt;
    }

    IoResponse data;
    data.timestamp = std::chrono::system_clock::from_time_t(j["timestamp"].get<std::time_t>());
    for (const auto& [key, value] : j["responses"].items()) {
      data.responses[key] = value.get<int>();
    }
    return data;
  }  */

};

struct IoCommandQueue
{

  public:

  struct Command
  {
    IoRequest::RequestType type;
    int index;
    bool value;

    Command(IoRequest::RequestType type, int index, bool value) : type(type), index(index), value(value) {}
  };

  struct Response
  {
    IoRequest::RequestType type;
    int index;
    int value;

    Response(IoRequest::RequestType type, int index, int value) : type(type), index(index), value(value) {}
    
  };  

  std::queue<Command> commandQueue;
  std::queue<Response> responseQueue;
  std::mutex commandQueueMutex;
};  


} // namespace ecat_sh_hardware

#endif // ECAT_SH_HARDWARE_IO_TCP_SERVER_HPP