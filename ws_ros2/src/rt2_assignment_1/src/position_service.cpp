#include "rclcpp/rclcpp.hpp"
#include "rt2_assignment_1/srv/random_position.hpp"

#include <memory>
#include <functional>
using std::placeholders::_1;
using std::placeholders::_2;

# define M_PI 3.14159265358979323846

#define NODE_NAME "random_position_server"
#define SERVICE_RANDOM_POSITION "/position_server"

class random_position_server : public rclcpp::Node
{
public:
	/// constructor of the class random_position_server
	random_position_server( ) : rclcpp::Node( NODE_NAME )
	{
		srv = this->create_service<rt2_assignment_1::srv::RandomPosition>(
			SERVICE_RANDOM_POSITION,
			std::bind( &random_position_server::myrandom, this, _1, _2 )
		);
	}

private:
	/// service handle
	rclcpp::Service<rt2_assignment_1::srv::RandomPosition>::SharedPtr srv;
	
	/// generate a random planar posture
	void myrandom(
		const std::shared_ptr<rt2_assignment_1::srv::RandomPosition::Request> req,
		std::shared_ptr<rt2_assignment_1::srv::RandomPosition::Response> res
	)
	{
		res->x = randMToN( req->x_min, req->x_max );
		res->y = randMToN( req->y_min, req->y_max );
		res->theta = randMToN( -M_PI, M_PI );
	}
	
	/// generate a random number in [M, N]
	double randMToN(double M, double N)
	{     
		return M + (rand() / ( RAND_MAX / (N-M) ) ) ; 
	}
};

int main(int argc, char **argv)
{
	rclcpp::init( argc, argv );

	auto node = std::make_shared<random_position_server>( );
	rclcpp::spin( node );

	rclcpp::shutdown( );
	return 0;
}
