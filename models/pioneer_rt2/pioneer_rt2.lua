--[[
	LUA SCRIPT (ROS1 complatible) for the VRep simulation of a simple Mobile Robot.
	
	Simple (2, 0) mobile robot. This script provides two topics by which to drive 
	the robot via external ROS nodes: /cmd_vel (geometry_msgs/msg/Twist as subscriber) 
	and /odom (nav_msgs/Odometry as publisher). 
--]]

function sysCall_init( ) 
    if not simROS then
        print( "ERROR: plugin ROS1 not loaded. Stopping simulation..." )
        sim.stopSimulation( )
        return
    else
        print( "ROS1 plugin correctly loaded. " )
    end
    
    robot = sim.getObject( '.' )
    -- geometry of the robot
    leftWheel = sim.getObject(':/leftWheel')
    rightWheel = sim.getObject(':/rightWheel')
    local leftP = sim.getObjectPosition( leftWheel, -1 )
    local rightP = sim.getObjectPosition( rightWheel, -1 )
    -- wheel distance
    r = norm_of( { ( rightP[1] - leftP[1] ), ( rightP[2] - leftP[2] ) } )
    print( "wheel axis length: " .. r )
    -- wheel radius
    a = sim.getShapeBB( sim.getObject(':/rightWheel') )[1] / 2.0
    print( "wheel radius: " .. a )
    
    -- motors
    motorLeft=sim.getObject("./leftMotor")
    motorRight=sim.getObject("./rightMotor")
    sim.setJointTargetVelocity( motorLeft, 0 )
    sim.setJointTargetVelocity( motorRight, 0 )
    
    -- ROS interface
    tw = {
        linear = { x=0.0, y=0.0, z=0.0 },
        angular = { x=0.0, y=0.0, z=0.0 }
    }
    v = 0.0
    w = 0.0
    input_twist = simROS.subscribe( '/cmd_vel', 'geometry_msgs/Twist', 'in_twist_cbk' )
    output_odom = simROS.advertise( '/odom', 'nav_msgs/Odometry' )
    
    -- print( get_odom_msg() )
end
--


function in_twist_cbk( in_tw )
    tw = in_tw
end
--


function norm_of( P )
    return math.sqrt( P[1]*P[1] + P[2]*P[2] )
end
--


function get_odom_msg( )
    p = sim.getObjectPosition( robot, -1 )
    o = sim.getObjectQuaternion( robot, -1 )
    return {
        header={
            stamp = sim.getSystemTime( ),
            frame_id = 'world'
        },
        child_frame_id = 'robot', 
        pose = {
            pose = {
                position    = { x=p[1], y=p[2], z=p[3] },
                orientation = { x=o[1], y=o[2], z=o[3], w=o[4] }
            },
            covariance = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
        },
        twist = {
            twist = {
                linear = { x=tw[1], y=tw[2], z=tw[3] },
                angular = { x=tw[4], y=tw[5], z=tw[6] }
            },
            covariance = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
        }
    }
end
--


function sysCall_actuation( ) 
    -- publish the actual position
    simROS.publish( output_odom, get_odom_msg( ) )
    
    -- move the robot
    v = tw.linear.x
    w = tw.angular.z
    q1 = ( 2*v + r*w ) / (2*a) -- left
    q2 = ( 2*v - r*w ) / (2*a) -- right
    
    sim.setJointTargetVelocity( motorLeft, q1 )
    sim.setJointTargetVelocity( motorRight, q2 )
end 
--


function sysCall_cleanup( ) 
    simROS.shutdownSubscriber( input_twist )
    simROS.shutdownPublisher( output_odom )
end 
--
