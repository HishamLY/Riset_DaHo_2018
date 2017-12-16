#include "TeamCommunication.h"

using namespace Robot;

int main(int argc, char *argv[])
{
  TeamCommunication *teamCom = new TeamCommunication(3838);
  char receivedData[25];

  teamCom->receiver->set_non_blocking(true);

  while (1)
  {
  }

  return 0;
}
