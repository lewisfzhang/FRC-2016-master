#include <string>
#include <netinet/in.h>

class UDPClient {
private:
  bool connected_;
  int socket_;
  std::string hostname_;
  int port_;
  struct sockaddr_in addr_;

public:
  UDPClient(std::string hostname, int port);
  bool connect();
  bool isConnected();
  bool send(std::string message);
};
