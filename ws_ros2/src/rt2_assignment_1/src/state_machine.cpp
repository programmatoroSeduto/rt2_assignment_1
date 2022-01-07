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

class state_machine : public rclcpp::Node
{
public:
	/// class constructor of state_machine
	state_machine( ) : rclcpp::Node( NODE_NAME )
	{
		// create a callback group
		// ...
		
		// open client CLIENT_RANDOM_POSITION
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
		tm = this->create_wall_timer(
			500ms,
			std::bind( &state_machine::working_cycle, this )
		);
	}

private:
	// the working cycle
	void working_cycle( void )
	{
		if( !start ) return;
		
		// ask for a goal randomly generated
		auto random_point_req = std::make_shared<rt2_assignment_1::srv::RandomPosition::Request>( );
		random_point_req->x_min = -LEN_SQUARE;
		random_point_req->x_max = LEN_SQUARE;
		random_point_req->y_min = -LEN_SQUARE;
		random_point_req->y_max = LEN_SQUARE;
		auto random_point_promise = cl_random_position->async_send_request( random_point_req );
		random_point_promise.wait( ); // sync request
		auto random_point_res = random_point_promise.get( );
		
		// go to the point and wait
		auto go_to_point_req = std::make_shared<rt2_assignment_1::srv::Position::Request>( );
		go_to_point_req->x = random_point_res->x;
		go_to_point_req->y = random_point_res->y;
		go_to_point_req->theta = random_point_res->theta;
		auto go_to_point_promise = cl_go_to_point->async_send_request( go_to_point_req );
		go_to_point_promise.wait( );
		auto go_to_point_res = go_to_point_promise.get( );
	}
	
	// implementation of the service SERVICE_USER_INTERFACE
	void callback_user_interface(
		const std::shared_ptr<rt2_assignment_1::srv::Command::Request> req,
		std::shared_ptr<rt2_assignment_1::srv::Command::Response> res
	)
	{
		this->start = ( req->command == "start" );
		res->ok = this->start;
	}
	
	// state of the node
	bool start = false;
	
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
