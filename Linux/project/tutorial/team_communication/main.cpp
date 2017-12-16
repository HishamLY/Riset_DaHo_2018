#include "TeamCommunication.h"

using namespace Robot;

int main(int argc, char * argv[])
{
    TeamCommunication *teamCom = new TeamCommunication(3838);
    char receivedData[25];

	  teamCom->receiver->set_non_blocking(true);

    while(1)
    {
        //send the message
        if (teamCom->receiver->recv(receivedData, 10) > 0)
        {
		      printf("%s\n", receivedData);
        }
//        else
//		printf("FAILED RECEIVING\n");
    }

    return 0;
}
