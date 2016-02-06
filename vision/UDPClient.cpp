#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <netdb.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "UDPClient.hpp"

UDPClient::UDPClient(std::string hostname, int port) {
  connected_ = false;
  hostname_ = hostname;
  port_ = port;
}

bool UDPClient::connect() {
  struct hostent *he;
  struct in_addr **addr_list;

  // Resolve hostname
  if ((he = gethostbyname(hostname_.c_str())) == NULL) {
    connected_ = false;
    return false;
  }

#ifdef DEBUG_VISION
  // print information about this host:
  printf("Hostname is: %s\n", he->h_name);
  printf("    IP addresses: ");
  addr_list = (struct in_addr **)he->h_addr_list;
  for (int i = 0; addr_list[i] != NULL; i++) {
    printf("%s ", inet_ntoa(*addr_list[i]));
  }
  printf("\n");
#endif

  addr_.sin_family = AF_INET;
  memcpy(&addr_.sin_addr, he->h_addr_list[0], he->h_length);
  addr_.sin_port = htons(port_);

  socket_ = socket(PF_INET, SOCK_DGRAM, 0);
  if (socket_ == -1) {
    connected_ = false;
    return false;
  }

  connected_ = true;
  return true;
}

bool UDPClient::isConnected() { return connected_; }

bool UDPClient::send(std::string message) {
  int ret = sendto(socket_, message.c_str(), message.length(), 0,
                   (struct sockaddr *)&addr_, sizeof(addr_));
  if (ret == -1) {
    connected_ = false;
  }
  return connected_;
}
