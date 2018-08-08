
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "MagTrackAction.h"

using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "mag_track_node_client");

  // create the action client
  actionlib::SimpleActionClient<mag_track::MagTrackAction> ac("mag_track_action", true);
  ac.waitForServer();
  mag_track::MagTrackGoal goal;

  char command = '0';

  while (command != '4') {
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "*********************************please send a command********************************" << std::endl;
    std::cout << "1: start the action" << std::endl <<
                 "2: pause the action" << std::endl <<
                 "3: switch enable_debug" << std::endl <<
                 "4: exit the program" << std::endl;
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "> ";
    std::cin >> command;
    if (command != '1' && command != '2' && command != '3' && command != '4') {
      std::cout << "please input again!" << std::endl;
      std::cout << "> ";
      std::cin >> command;
    }

    switch (command) {
      //start thread.
      case '1':
        goal.command = 1;
        std::cout << "I am running the request";
        ac.sendGoal(goal);
        break;
      //pause thread.
      case '2':
        goal.command = 2;
        std::cout << "Action server will pause.";
        ac.sendGoal(goal);
      //stop thread.
      case '3':
        goal.command = 3;
        std::cout<<"I am cancelling the request";
        ac.cancelGoal();
        break;
      default:
        break;
    }
  }
  return 0;
}
