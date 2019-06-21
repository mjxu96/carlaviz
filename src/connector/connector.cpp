
#include "connector/utils/package.h"

#include "carla/client/Client.h"
#include "carla/client/TimeoutException.h"
#include "carla/client/World.h"

#include "boost/shared_ptr.hpp"

#include <iostream>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using namespace std::string_literals;

int main() {
  try {
    std::string host("localhost");
    uint16_t port(2000u);

    carla::client::Client client(host, port);
    client.SetTimeout(10s);

    rothberg::utils::Package package(boost::make_shared<carla::client::World>(client.GetWorld()));
    while (true) {
      auto time1 = std::chrono::system_clock::now();
      package.Update();
      package.TmpOutput();
      auto time2 = std::chrono::system_clock::now();
      std::chrono::duration<double> durat = time2 - time1;
      std::cout << "use: " << durat.count() << std::endl;
      std::this_thread::sleep_for(1s);
    }

  } catch (const carla::client::TimeoutException& e) {
    std::cout << e.what() << std::endl;
    return 1;
  } catch (const std::exception& e) {
    std::cout << "Exception: " << e.what() << std::endl;
    return 2;
  }
  return 0;
}