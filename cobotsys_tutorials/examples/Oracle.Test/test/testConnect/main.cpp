//
// Created by cobot on 19-10-18.
//

#include <iostream>
#define LINUXOCCI //避免函数重定义错误
#include <occi.h>
#include <boost/shared_ptr.hpp>

using namespace std;
using namespace oracle::occi;

int main()
{
//    boost::shared_ptr<Environment> _jsonAndStructConvertCZT = boost::shared_ptr<JsonAndStructConvertCZT>(new JsonAndStructConvertCZT());
    Environment *env=Environment::createEnvironment(Environment::DEFAULT);
    cout<<"success"<<endl;

    string name = "SYSTEM";
    string pass = "123";
    string srvName = "192.168.10.170:1521/database";

    //string method = "select * from all_users";

    string method = "select * from czt_view";
    string strTemp;
    try
    {
        Connection *conn = env->createConnection(name, pass,srvName);
        cout<<"conn success"<<endl;
        Statement *pStmt = conn->createStatement(method);
        ResultSet *pRs = pStmt->executeQuery();
        while(pRs->next())
        {
            strTemp = pRs->getString(2);
            std::string orderNumber = pRs->getString(3);
            std::string sourceContainerNumber = pRs->getString(4);
            cout<<strTemp<<" "<<orderNumber<<" "<<sourceContainerNumber<<endl;
        }

        pStmt->closeResultSet(pRs);
        conn->terminateStatement(pStmt);
        env->terminateConnection(conn);
    }
    catch(SQLException e)
    {
        cout<<e.what()<<endl;
    }
    Environment::terminateEnvironment(env);
    cout<<"end!"<<endl;

    return 0;
}