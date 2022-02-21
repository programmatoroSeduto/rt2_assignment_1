#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <functional>
using std::placeholders::_1;
using std::placeholders::_2;
#include <chrono>
using namespace std::chrono_literals;

#include "rt2_assignment_1/srv/command.hpp"
#include "rt2_assignment_1/srv/position.hpp"
#include "rt2_assignment_1/srv/random_position.hpp"

#define NODE_NAME "state_machine"
#define CLIENT_RANDOM_POSITION "/position_server"
#define CLIENT_GO_TO_POINT "/go_to_point"
#define SERVICE_USER_INTERFACE "/user_interface"
#define LEN_SQUARE 5.0
#define CMD_START "start"

#define LOGSQUARE( str ) "[" << str << "] "
#define OUTLOG( msg_stream ) RCLCPP_INFO_STREAM( rclcpp::get_logger( NODE_NAME ), msg_stream )
#define OUTERR( msg_err_stream ) RCLCPP_ERROR_STREAM( rclcpp::get_logger( NODE_NAME ), "ERROR: " << msg_err_stream )

class state_machine : public rclcpp::Node
{
public:
	/// class constructor of state_machine
	state_machine( ) : rclcpp::Node( NODE_NAME )
	{
		// create a callback group
		this->callback_group = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
		
		// open client CLIENT_RANDOM_POSITION
		//    vedi https://answers.ros.org/question/298594/how-does-ros2-select-the-default-qos-profile/
		/*
		cl_random_position = this->create_client<rt2_assignment_1::srv::RandomPosition>(
			CLIENT_RANDOM_POSITION,
			rmw_qos_profile_default,
			callback_group );
		*/
		cl_random_position = this->create_client<rt2_assignment_1::srv::RandomPosition>(
			CLIENT_RANDOM_POSITION );
		
		// open client CLIENT_GO_TO_POINT
		cl_go_to_point = this->create_client<rt2_assignment_1::srv::Position>(
			CLIENT_GO_TO_POINT );
		
		// open service SERVICE_USER_INTERFACE
		srv_user_interface = this->create_service<rt2_assignment_1::srv::Command>(
			SERVICE_USER_INTERFACE,
			std::bind( &state_machine::callback_user_interface, this, _1, _2 )
		);
		
		// timed working cycle
		//    vedi https://docs.ros2.org/beta3/api/rclcpp/classrclcpp_1_1node_1_1Node.html#a1a727c1777f045074e13b2cbc6e9b0e3
		tm = this->create_wall_timer(
			500ms,
			std::bind( &state_machine::working_cycle, this ),
			this->callback_group
		);
	}

private:
	// the working cycle
	void working_cycle( void )
	{
		if( !start ) return;
		
		is_busy = true;
		
		// ask for a goal randomly generated
		auto random_point_req = std::make_shared<rt2_assignment_1::srv::RandomPosition::Request>( );
		random_point_req->x_min = -LEN_SQUARE;
		random_point_req->x_max = LEN_SQUARE;
		random_point_req->y_min = -LEN_SQUARE;
		random_point_req->y_max = LEN_SQUARE;
		auto random_point_promise = cl_random_position->async_send_request( random_point_req );
		OUTLOG( "waiting request for a random point ..." );
		random_point_promise.wait( ); // sync request
		OUTLOG( "waiting request for a random point ... OK" );
		auto random_point_res = random_point_promise.get( );
		
		// go to the point and wait
		auto go_to_point_req = std::make_shared<rt2_assignment_1::srv::Position::Request>( );
		go_to_point_req->x = random_point_res->x;
		go_to_point_req->y = random_point_res->y;
		go_to_point_req->theta = random_point_res->theta;
		auto go_to_point_promise = cl_go_to_point->async_send_request( go_to_point_req );
		OUTLOG( "waiting service /go_to_point ... " );
		go_to_point_promise.wait( );
		OUTLOG( "waiting service /go_to_point ... OK " );
		auto go_to_point_res = go_to_point_promise.get( );
		
		is_busy = false;
	}
	
	// implementation of the service SERVICE_USER_INTERFACE
	void callback_user_interface(
		const std::shared_ptr<rt2_assignment_1::srv::Command::Request> req,
		std::shared_ptr<rt2_assignment_1::srv::Command::Response> res
	)
	{
		OUTLOG( "service /user_interface RECEIVED COMMAND '" << req->command << "'" );
		if( !( req->command == CMD_START ) && start && is_busy )
		{
			OUTERR( "The service is busy now! UNable to stop." );
			return;
		}
		
		this->start = ( req->command == CMD_START );
		res->ok = this->start;
		OUTLOG( ( this->start ? "START command received" : "STOP command received" ) );
	}
	
	// state of the node
	bool start = false;
	
	// activity status flag of the node
	bool is_busy = false;
	
	// the callback group
	rclcpp::callback_group::CallbackGroup::SharedPtr callback_group;
	
	// handle CLIENT_RANDOM_POSITION
	rclcpp::Client<rt2_assignment_1::srv::RandomPosition>::SharedPtr cl_random_position;
	
	// handle CLIENT_GO_TO_POINT
	rclcpp::Client<rt2_assignment_1::srv::Position>::SharedPtr cl_go_to_point;
	
	// handle SERVICE_USER_INTERFACE
	rclcpp::Service<rt2_assignment_1::srv::Command>::SharedPtr srv_user_interface;
	
	// timer handle
	rclcpp::TimerBase::SharedPtr tm;
};

int main( int argc, char* argv[] )
{
	rclcpp::init( argc, argv );
	
	auto node = std::make_shared<state_machine>( );
	rclcpp::executors::MultiThreadedExecutor exec;
	exec.add_node( node );
	exec.spin( );
	
	rclcpp::shutdown( );
	return 0;
}
