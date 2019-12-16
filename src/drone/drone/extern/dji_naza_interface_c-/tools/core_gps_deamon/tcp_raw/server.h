#ifndef _server_h_
#define _server_h_

#include <iostream>
#include <unistd.h>
#include <vector>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <netinet/in.h>
#include <sys/socket.h>

#include "client.h"
#include "mythread.h"

using namespace std;

class Server {

private:
  static vector<Client> clients;

  // Socket stuff
  int serverSock, clientSock;
  struct sockaddr_in serverAddr, clientAddr;
  char buff[256];

public:
  Server(int p);
  void AcceptAndDispatch();
  static void SendToAll(char *message);
  static void *HandleClient(void *args);

private:
  static void ListClients();
  static int FindClientIndex(Client *c);
};

#endif
