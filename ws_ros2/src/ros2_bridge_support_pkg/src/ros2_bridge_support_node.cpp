
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <memory>
using std::placeholders::_1;
using std::placeholders::_2;

#include "std_msgs/msg/string.hpp"
#include "rt2_assignment_1/srv/command.hpp"
#include "rt2_assignment_1/srv/position.hpp"

#define NODE_NAME "ros2_bridge_support_node"
#define BRIDGE_TOPIC_NAME_PREFIX "/bridge_topic"
#define BRIDGE_SERVICE_NAME_PREFIX "/bridge_service"
#define QUEUE_SZ_DEFAULT 100
#define WAITING_DELAY 0.01

#define SERVICE_USER_INTERFACE "/user_interface"
#define CLIENT_GO_TO_POINT "/go_to_point"

#define LOGSQUARE( str ) "[" << str << "] "
#define OUTLOG( msg_stream ) RCLCPP_INFO_STREAM( rclcpp::get_logger( NODE_NAME ), msg_stream )
#define OUTERR( msg_err_stream ) RCLCPP_ERROR_STREAM( rclcpp::get_logger( NODE_NAME ), "ERROR: " << msg_err_stream )
#define SS( this_string ) std::string( this_string )
#define SSS( this_thing ) std::to_string( this_thing )

#define BRIDGE_MSG_ROS2_ROS1( topic ) OUTLOG( "bridge topic: " << LOGSQUARE( topic ) << " (ROS2) --> (ROS1)" << LOGSQUARE( BRIDGE_TOPIC_NAME_PREFIX << topic ) )

namespace str_tools
{
	bool check_seq_at( std::string str_arg, std::string to_find, int i, bool check_end = false )
    {
        if( ((std::size_t)i) >= str_arg.length( ) ) return false;
        
        std::size_t next_idx = i + to_find.length( );
        std::string sub = str_arg.substr( i, to_find.length( ) );
        bool found = ( to_find == sub );
        
        if( check_end )
            return found && ( next_idx >= str_arg.length( ) );
        else
            return found;
    }
    
    /// split a string using a separator 
    /// @return this function returns an empty vector if the string is not well-formed
    /// @note nested messages using /{ and /} are not supported right now
    std::vector<std::string> pack_split( std::string str, char separator )
    {
    	std::vector<std::string> res;
    	std::string q = "";
    	bool force_no_sep = false;
    	
    	for( unsigned int i=0; i<str.length(); ++i )
    	{
    	    if( force_no_sep )
    	    {
    	        if( str_tools::check_seq_at( str, "/} ", i) || str_tools::check_seq_at( str, "/}", i, true) )
                {
                    // END OF A NON-EMPTY string
                    force_no_sep = false;
                    res.push_back( q );
                    q = "";
                    
                    ++i;
                }
                else
                    // continue reading
                    q += str[i];
    	        
    	    }
    		else if( str[i] == separator )
    		{
    		    // save the previous token
    			if( q != "" ) 
    			{
    			    res.push_back( q );
    			    q = "";
    			}
    		}
    		else if( str[i] == '/' )
            {
                if( str_tools::check_seq_at( str, "/{", i ) )
                {
                    if( str_tools::check_seq_at( str, "/{//} ", i) || str_tools::check_seq_at( str, "/{//}", i, true) )
                    {
                        // EMPTY STRING DETECTED
                        // insert a blank into the vector
                        res.push_back( "" );
                        
                        // jump to the next char (taking into accout the for loop)
                        i += 5;
                    }
                    else
                    {
                        // NON-EMPTY STRING DETECTED
                        // skip the group "/{" (taking into accout the for loop)
                        ++i;
                        
                        // enter in no-sep mode
                        force_no_sep = true;
                    }
                }
    		}
    		else
    			q += str[i];
    	}
    	
    	if( !force_no_sep )
    	{
        	if( q != "" ) res.push_back( q );
        	return res;
    	}
    	else  // bad format
    	    return std::vector<std::string>( );
    }
	
	// generate the name of the topic on the bridge
	std::string get_name_of_topic( std::string topic_name )
	{
		return SS(BRIDGE_TOPIC_NAME_PREFIX) + topic_name; 
	}
	
	// generate the name of the topic for the service request
	std::string get_name_of_service_request( std::string serv_name )
	{
		return SS(BRIDGE_SERVICE_NAME_PREFIX) + serv_name + "_request";
	}
	
	// generate the name of the topic for the service response
	std::string get_name_of_service_response( std::string serv_name )
	{
		return SS(BRIDGE_SERVICE_NAME_PREFIX) + serv_name + "_response";
	}
}

namespace cast_tools
{
	/// cast back a boolean
	bool cast_back_field( std::string pack )
	{
		return ( pack == "1" ? true : false );
	}
	
	/// @brief get a string type 
	/// @note the empty string generates a sequence /{//} which indicates a empty field
	std::string cast_field( std::string pack, bool use_sep = false, std::string sep = " " )
	{
		// generate the empty sequence if the string is empty
		pack = ( pack != "" ? pack : "/" );
		
		std::string str = "/{" + pack + "/}";
		
		if( use_sep ) str += sep;
		return str;
	}
	
	// bool type to string
	std::string cast_field( bool pack, bool use_sep = false, std::string sep = " " )
	{
		std::string str = ( pack ? "1" : "0" );
		
		if( use_sep ) str += sep;
		return str;
	}
	
	// --- STUB TEMPLATE CAST RULES
	
	// cast topic msg
	// template< typename c_msg_type > 
	// std::string cast_message( const typename c_msg_type::SharedPtr msg )
	std::string cast_message( const std_msgs::msg::String::SharedPtr msg )
	{
		OUTLOG( "calling naive cast_message( )" );
		return "/{//}"; 
	}

	// cast back topic msg
	// template< typename cb_msg_type > 
	void cast_back_message( const std_msgs::msg::String::SharedPtr msg, std_msgs::msg::String& msg_return )
	{ 
		OUTLOG( "calling naive cast_back_message( )" ); 
		// msg = msg; 
		// msg_return = msg_return; 
	}
	
	/*
	// cast the service request
	template< class c_srv_type_req >
	std::string cast_service_request( const std::shared_ptr<typename c_srv_type_req::Request> req )
	{ 
		OUTLOG( "calling naive cast_service_request( )" ); 
		return "/{//}"; 
	}

	// cast back the service request
	template< typename cb_srv_type_req >
	void cast_back_service_request( std::string &msg, std::shared_ptr< typename cb_srv_type_req::Request >& req )
	{ 
		OUTLOG( "calling naive cast_back_service_request( )" ); 
		msg = msg; 
		req = req; 
	}

	// cast the service response
	template< typename c_srv_type_res >
	std::string cast_service_response( const std::shared_ptr<typename c_srv_type_res::Response> res )
	{ 
		OUTLOG( "calling naive cast_service_response( )" ); 
		return "/{//}"; 
	}

	// cast back the service response
	template< typename cb_srv_type_res >
	void cast_back_service_response( std::string &msg, std::shared_ptr< typename cb_srv_type_res::Response >& res )
	{ 
		OUTLOG( "calling naive cast_back_service_response( )" ); 
		msg = msg; 
		res = res; 
	}
	*/
	
	// --- RULES FOR Command.srv
	// cast the service request rt2_assignment_1::srv::Command
	std::string cast_service_request( const std::shared_ptr<rt2_assignment_1::srv::Command::Request> req )
	{
		return cast_tools::cast_field( req->command, false );
	}

	// cast back the service request of rt2_assignment_1::srv::Command
	void cast_back_service_request( std::string &msg, std::shared_ptr<rt2_assignment_1::srv::Command::Request>& req )
	{
		std::vector<std::string> content = str_tools::pack_split( msg, ' ' );
		req->command = content[0];
	}

	// cast the service response of rt2_assignment_1::srv::Command
	std::string cast_service_response( const std::shared_ptr<rt2_assignment_1::srv::Command::Response> res )
	{
		return cast_tools::cast_field( res->ok, false );
	}

	// cast back the service response rt2_assignment_1::srv::Command
	void cast_back_service_response( std::string &msg, std::shared_ptr<rt2_assignment_1::srv::Command::Response>& res )
	{
		res->ok = cast_back_field( msg );
	}
	
	// --- RULES FOR Position.srv
	// cast the service request rt2_assignment_1::srv::Position
	std::string cast_service_request( const std::shared_ptr<rt2_assignment_1::srv::Position::Request> req )
	{
		std::string str = "";
		
		str += SSS( req->x ) + " ";
		str += SSS( req->y ) + " ";
		str += SSS( req->theta );
		
		return str;
	}

	// cast back the service request of rt2_assignment_1::srv::Position
	void cast_back_service_request( std::string &msg, std::shared_ptr<rt2_assignment_1::srv::Position::Request>& req )
	{
		std::vector< std::string > content = str_tools::pack_split( msg, ' ' );
		
		req->x = atof( content[0].c_str( ) );
		req->y = atof( content[1].c_str( ) );
		req->theta = atof( content[2].c_str( ) );
	}

	// cast the service response of rt2_assignment_1::srv::Position
	std::string cast_service_response( const std::shared_ptr<rt2_assignment_1::srv::Position::Response> res )
	{
		return cast_tools::cast_field( res->ok, false );
	}

	// cast back the service response rt2_assignment_1::srv::Position
	void cast_back_service_response( std::string &msg, std::shared_ptr<rt2_assignment_1::srv::Position::Response>& res )
	{
		res->ok = cast_tools::cast_back_field( msg );
	}
}

/// representation of a topic in ROS2
template< typename topicT_pub, typename topicT_sub >
class bridge_topic
{
public:
	bridge_topic( ) { }

	typename rclcpp::Publisher< topicT_pub >::SharedPtr pub;
	typename rclcpp::Subscription< topicT_sub >::SharedPtr sub;
	
	// "OUT BRIDGE" : from ROS2 to ROS1 (cast)
	void bridge_cbk_out( const typename topicT_sub::SharedPtr msg ) const
	{
		// cast the message to string
		std_msgs::msg::String bridge_data;
		// bridge_data.data = cast_tools::cast_message< topicT_sub >( msg );
		bridge_data.data = cast_tools::cast_message( msg );
		
		// publish the message
		this->pub->publish( bridge_data );
	}
	
	// "IN BRIDGE" : from ROS1 to ROS2 (cast back)
	void bridge_cbk_in( const std_msgs::msg::String::SharedPtr msg ) const
	{
		// cast back the message from string to message
		topicT_pub bridge_data;
		// cast_tools::cast_back_message< topicT_pub >( msg, bridge_data );
		cast_tools::cast_back_message( msg, bridge_data );
		
		// publish the message
		this->pub->publish( bridge_data );
	}
};

/// representation of a service in ROS2 
template< typename serviceT >
class bridge_service
{
public:
	bridge_service( ) { }
	
	// first topic from ROS1
	rclcpp::Subscription< std_msgs::msg::String >::SharedPtr topic_in;
	// second topic to ROS1
	rclcpp::Publisher< std_msgs::msg::String >::SharedPtr topic_out;
	// out service - client ROS2 service ROS1
	typename rclcpp::Service< serviceT >::SharedPtr service_out;
	// in service - service ROS2 client ROS1
	typename rclcpp::Client< serviceT >::SharedPtr service_in;
	
	// callback group for the subscriber
	rclcpp::callback_group::CallbackGroup::SharedPtr callback_group;
	// subscription options
	rclcpp::SubscriptionOptions subscription_options;
	
	/// @brief callback for the 'out' service from ROS2 endpoint to ROS1 endpoint
	/// @note 'out' means that the client runs in ROS2 (this framework)
	void service_out_callback(
		const std::shared_ptr< typename serviceT::Request > req,
		std::shared_ptr< typename serviceT::Response > res )
	{
		OUTLOG( "received a ROS2->ROS1 service request from ROS2" );
		
		// set the pending request and reset the response
		this->pending_request = *req;
		this->response_string = "";
		
		// cast the request to string
		std_msgs::msg::String req_str;
		// req_str.data = cast_tools::cast_service_request< serviceT >( req );
		req_str.data = cast_tools::cast_service_request( req );
		
		// send the request to ROS1 through topic_out 
		this->topic_out->publish( req_str );
		waiting_response = true;
		
		OUTLOG( "waiting a response from ROS1..." );
		rclcpp::WallRate loop_rate(500);
		while( waiting_response )
		{
			loop_rate.sleep( );
		}
		OUTLOG( "got a response from ROS1 -> [" << response_string << "]" );
		
		// cast back the message and write it
		// cast_tools::cast_back_service_response< serviceT >( response_string, res );
		cast_tools::cast_back_service_response( response_string, res );
	}
	
	/// subscriber which calls the endpoint
	/// @note 'in' i.e. the client runs in ROS1 (the other framework)
	void service_in_callback(
		const std_msgs::msg::String::SharedPtr msg ) const
	{
		OUTLOG( "received a ROS1->ROS2 request from ROS1 endpoint" ); 
		OUTLOG( "with data: [" << msg->data << "]" );
		
		// cast back of the request
		std::shared_ptr< typename serviceT::Request > req = std::make_shared< typename serviceT::Request >( );
		// cast_tools::cast_back_service_request< serviceT >( msg->data, req );
		cast_tools::cast_back_service_request( msg->data, req );
		
		// call the endpoint and get the result
		OUTLOG( "waiting for a response from the service..." );
		auto res_future = service_in->async_send_request( req );
		// synchronous request
		res_future.wait( );
		std::shared_ptr< typename serviceT::Response > res = res_future.get( );
		
		// cast the response from the endpoint
		std_msgs::msg::String res_str;
		// res_str.data = cast_tools::cast_service_response< serviceT >( res );
		res_str.data = cast_tools::cast_service_response( res );
		
		// publish the response
		OUTLOG( "returning response: [" << res_str.data << "]" );
		this->topic_out->publish( res_str );
	}
	
	/// callback service subscriber
	void topic_in_callback( const std_msgs::msg::String::SharedPtr msg )
	{
		if( waiting_response )
		{
			// store the message into the class variable
			this->response_string = msg->data;
			this->waiting_response = false;
		}
	}

private:
	// pending request
	typename serviceT::Request pending_request;
	// waiting for a response from ROS2
	bool waiting_response = false;
	// the response as string
	std::string response_string = "";
};

class ros2_bridge_support : public rclcpp::Node
{
public:
	ros2_bridge_support( ) : rclcpp::Node( NODE_NAME )  // using static mapping
	{
		// service user interface
		make_link_service_in<rt2_assignment_1::srv::Command>( &srv_user_interface, SERVICE_USER_INTERFACE );
		
		// client go to point
		make_link_service_out<rt2_assignment_1::srv::Position>( &srv_go_to_point, CLIENT_GO_TO_POINT );
		
		OUTLOG( "online!" );
	}

private:
	// in service user interface
	bridge_service<rt2_assignment_1::srv::Command> srv_user_interface;
	
	// out service go to poin
	bridge_service<rt2_assignment_1::srv::Position> srv_go_to_point;
	
	// --- LINK METHODS ---
	
	/// create a "bridge out" topic
	/// @note 'out' topic as 'sub' from the end node and 'pub' to bridge
	template< typename Ttopic >
	void make_link_topic_out( bridge_topic< std_msgs::msg::String, Ttopic >* br_class, std::string sub_topic )
	{
		OUTLOG( "--- (out) TOPIC ROS1 to ROS2: " << sub_topic );
		
		// subscription
		//    https://answers.ros.org/question/308386/ros2-add-arguments-to-callback/
		OUTLOG( "subscription to " << sub_topic );
		br_class->sub = this->create_subscription< Ttopic >(
			sub_topic,
			QUEUE_SZ_DEFAULT,
			std::bind( &bridge_topic< std_msgs::msg::String, Ttopic >::bridge_cbk_out, br_class, _1 )
		);
		
		// publisher
		OUTLOG( "publisher on " << str_tools::get_name_of_topic( sub_topic ) );
		br_class->pub = this->create_publisher< std_msgs::msg::String >(
			str_tools::get_name_of_topic( sub_topic ), 
			QUEUE_SZ_DEFAULT 
		);
	}
	
	/// create a "bridge in" topic
	/// @note 'in' topic as 'sub' from bridge and 'pub' to the end node
	template< typename Ttopic >
	void make_link_topic_in( bridge_topic< Ttopic, std_msgs::msg::String >* br_class, std::string pub_topic )
	{
		OUTLOG( "--- (in) TOPIC ROS2 to ROS1: " << pub_topic );
		
		// subscription
		//    https://answers.ros.org/question/308386/ros2-add-arguments-to-callback/
		OUTLOG( "subscription to " << str_tools::get_name_of_topic( pub_topic ) );
		br_class->sub = this->create_subscription< std_msgs::msg::String >(
			str_tools::get_name_of_topic( pub_topic ),
			QUEUE_SZ_DEFAULT,
			std::bind( &bridge_topic< Ttopic, std_msgs::msg::String >::bridge_cbk_in, br_class, _1 )
		);
		
		// publisher
		OUTLOG( "publisher on " << pub_topic );
		br_class->pub = this->create_publisher< Ttopic >(
			pub_topic,
			QUEUE_SZ_DEFAULT 
		);
	}
	
	/// create a "bridge in" service
	template< typename serviceT >
	void make_link_service_in( bridge_service< serviceT >* br_class, std::string service_name )
	{
		OUTLOG( "--- (in) SERVICE client in ROS2 service in ROS1: " << service_name );
		
		// new callback group
		// vedi 
		//    https://get-help.robotigniteacademy.com/t/how-to-link-callback-function-to-a-subscription/11043/3
		br_class->callback_group = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
		br_class->subscription_options.callback_group = br_class->callback_group;
		
		// in subscriber -- request
		// vedi anche
		//    https://docs.ros.org/en/foxy/Tutorials/Topics/Topic-Statistics-Tutorial.html
		OUTLOG( "subscription to " << str_tools::get_name_of_service_request( service_name ) );
		br_class->topic_in = this->create_subscription< std_msgs::msg::String >(
			str_tools::get_name_of_service_request( service_name ),
			10,  // THE NEW MAGIC NUMBER! (what does it mean '10'? Why especially '10'? Has it a name?)
			std::bind( &bridge_service< serviceT >::service_in_callback, br_class, _1 ),
			br_class->subscription_options
		);
		
		// out publisher -- response
		OUTLOG( "publisher on " << str_tools::get_name_of_service_response( service_name ) );
		br_class->topic_out = this->create_publisher< std_msgs::msg::String >(
			str_tools::get_name_of_service_response( service_name ),
			QUEUE_SZ_DEFAULT
		);
		
		// out service to endpoint
		OUTLOG( "creating 'in' service " << service_name );
		br_class->service_in = this->create_client< serviceT >( service_name );
	}
	
	/// create a "bridge out" service
	/// @todo maybe the label "out" is conceptually wrong...
	template< typename serviceT >
	void make_link_service_out( bridge_service< serviceT >* br_class, std::string service_name )
	{
		OUTLOG( "--- (out) SERVICE client in ROS1 service in ROS2: " << service_name );
		
		// new callback group
		br_class->callback_group = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
		br_class->subscription_options.callback_group = br_class->callback_group;
		
		// in subscriber -- response
		OUTLOG( "subscription to " << str_tools::get_name_of_service_response( service_name ) );
		br_class->topic_in = this->create_subscription< std_msgs::msg::String >(
			str_tools::get_name_of_service_response( service_name ),
			10,  
			std::bind( &bridge_service< serviceT >::topic_in_callback, br_class, _1 ),
			br_class->subscription_options
		);
		
		// out publisher -- request
		OUTLOG( "publisher on " << str_tools::get_name_of_service_request( service_name ) );
		br_class->topic_out = this->create_publisher< std_msgs::msg::String >(
			str_tools::get_name_of_service_request( service_name ),
			QUEUE_SZ_DEFAULT
		);
		
		// out service from endpoint
		OUTLOG( "creating 'out' service " << service_name );
		br_class->service_out = this->create_service< serviceT >(
			service_name,
			std::bind( &bridge_service< serviceT >::service_out_callback, br_class, _1, _2 )
		);
	}
};

int main( int argc, char* argv[] )
{
	rclcpp::init( argc, argv );
	
	// rclcpp::spin( std::make_shared<ros2_bridge_topic_sender>( ) );
	auto node = std::make_shared<ros2_bridge_support>( );
	rclcpp::executors::MultiThreadedExecutor exec;
	exec.add_node(node);
	exec.spin( );
	
	rclcpp::shutdown( );
	return 0;
}
