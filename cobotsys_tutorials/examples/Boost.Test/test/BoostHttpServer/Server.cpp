#include "BoostHttpServer.h"


int main(){
    boost::shared_ptr<BoostHttpServer> server =  BoostHttpServer::createBoostHttpServer("0.0.0.0","8081");
    server->startAcceptor();
}