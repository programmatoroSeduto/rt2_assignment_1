
#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "rt2_assignment1/GoToPointAction.h"
#include "rt2_assignment1/RandomPosition.h"


class DummyActionClient
{
public:
	
	// creazione del client e istanziazione dei servizi per ottenere la posizione random
	DummyActionClient( ros::NodeHandle* nh = nullptr ) :
		ac( "go_to_point", true )
	{
		// richiesta dell'action client
		std::cout << "richiesta action client 'go_to_point' ..." << std::endl;
		//ac = actionlib::SimpleActionClient<rt2_assignment1::GoToPointAction>( "go_to_point", true );
		ac.waitForServer();
		std::cout << "action client 'go_to_point' -> OK" << std::endl;
		
		// richiesta del server per le random positions
		if( nh != nullptr )
		{
			std::cout << "richiesta service 'position_server' ..." << std::endl;
			randomPos = nh->serviceClient<rt2_assignment1::RandomPosition>("/position_server");
			std::cout << "service 'position_server' -> OK" << std::endl;
		}
		else
		{
			std::cout << "richiesta service 'position_server' ..." << std::endl;
			randomPos = innerNodeHandle.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
			std::cout << "service 'position_server' -> OK" << std::endl;
		}
	}
	
	// invio del goal
	void sendRandomGoal( )
	{
		rt2_assignment1::GoToPointGoal goal = generateRandomGoal();
		
		// invia all'ActionServer il goal
		ac.sendGoal( goal );
	}
	
	// attesa
	void waitFor( float duration )
	{
		(ros::Duration( duration )).sleep();
	}
	
	// cancella la richiesta
	void cancelRequest( )
	{
		ac.cancelAllGoals();
	}
	
private:
	// actlion client da cancellare 'go_to_point'
	actionlib::SimpleActionClient<rt2_assignment1::GoToPointAction> ac;
	
	// servizio 'position_server'
	ros::ServiceClient randomPos;
	
	// noe handle
	ros::NodeHandle innerNodeHandle;
	
	// richiedi un goal al service
	rt2_assignment1::GoToPointGoal generateRandomGoal( )
	{
		// richiesta della posizione
		rt2_assignment1::RandomPosition rp;
		rp.request.x_max = 5.0;
		rp.request.x_min = -5.0;
		rp.request.y_max = 5.0;
		rp.request.y_min = -5.0;
		randomPos.call(rp);
		
		// goal
		rt2_assignment1::GoToPointGoal goal;
		goal.x = rp.response.x;
   		goal.y = rp.response.y;
   		goal.theta = rp.response.theta;
   		
   		return goal;
	}
};


int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "DummyActionClient" );
	ros::NodeHandle nh;
	
	//DummyActionClient dac( &nh );
	DummyActionClient dac;
	
	std::cout << "invio del goal ..." << std::endl;
	dac.sendRandomGoal( );
	
	std::cout << "attesa di 2s ..." << std::endl;
	dac.waitFor( 2 );
	
	std::cout << "cancellazione della richiesta ..." << std::endl;
	dac.cancelRequest( );
	
	std::cout << "fatto!" << std::endl;
	return 0;
}
