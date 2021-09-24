#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/Position.h"
#include "rt2_assignment1/RandomPosition.h"
#include "rt2_assignment1/GoToPointAction.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

bool start = false;

bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res)
{
    if (req.command == "start")
    {
    	start = true;
    	std::cout << "[state_machine] START command received. " << std::endl;
    }
    else 
    {
    	if ( start )
			std::cout << "[state_machine] STOP command received. " << std::endl;
		else
			std::cout << "[state_machine] state machine already stopped. " << std::endl;
		
    	start = false;
    }
    return true;
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   ros::ServiceClient client_p = n.serviceClient<rt2_assignment1::Position>("/go_to_point");
   actionlib::SimpleActionClient<rt2_assignment1::GoToPointAction> ac( "go_to_point", true );
   ac.waitForServer( );
   
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
   rt2_assignment1::Position p;
   
   while(ros::ok()){
   	ros::spinOnce();
   	if (start)
   	{
		// ask for a random target
   		client_rp.call(rp);
   		
   		// prepare the goal
   		//p.request.x = rp.response.x;
   		//p.request.y = rp.response.y;
   		//p.request.theta = rp.response.theta;
   		rt2_assignment1::GoToPointGoal goal;
   		goal.x = rp.response.x;
   		goal.y = rp.response.y;
   		goal.theta = rp.response.theta;
   		cout << "NEXT TARGET (" << goal.x << ", " << goal.y << ", " << goal.theta << ")" << std::endl;
   		// std::cout << "\nGoing to the position: x= " << p.request.x << " y= " <<p.request.y << " theta = " <<p.request.theta << std::endl;
   		//client_p.call(p);
   		
   		// reach the position using the action
   		ac.sendGoal( goal );
   		bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
   		if( finished_before_timeout )
			std::cout << "Position reached" << std::endl;
		else
			std::cout << "Unable to reach the final position within the deadline" << std::endl;
   	}
   }
   return 0;
}
