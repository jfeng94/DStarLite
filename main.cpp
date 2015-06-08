#include "map.h"
<<<<<<< HEAD
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <cmath>
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include "ros/ros.h"
#include "tf/tf.h"
#include <fstream>
#include <ostream>
#include <istream>

int min_dist = .5;
bool safe_to_move;
double rangeholder[700];
double min_angle, max_angle;

void laser_cb( const sensor_msgs::LaserScan &scan )
{
	min_angle = scan.angle_min;
	max_angle = scan.angle_max;
	for (int i = 0; i < scan.ranges.size(); i++){
		rangeholder[i] = scan.ranges[i];
		if (scan.ranges[i] < min_dist){
			safe_to_move = false;
			return;
}
}
	safe_to_move = true;
}

class OdomWatcher {
public:

	OdomWatcher( ros::NodeHandle &nh, std::string topic_name = "odom" );

	// Callback function for messages from the "odom" topic
	void odom_cb( const nav_msgs::Odometry &od );

	/* Get current pose estimate
	 *
	 * If an estimate is available, return true and place the estimate in the
	 * given array. Otherwise, return false and do nothing to the array.
	 */
	bool get_pose_estimate( double *x );

private:
	double last_opose[3];
	bool valid_opose;
	ros::Subscriber odomsub;
};

OdomWatcher::OdomWatcher( ros::NodeHandle &nh, std::string topic_name )
	: valid_opose(false),
	  odomsub(nh.subscribe( topic_name, 1, &OdomWatcher::odom_cb, this ))
{ }

void OdomWatcher::odom_cb( const nav_msgs::Odometry &od )
{
	valid_opose = false;
	last_opose[0] = od.pose.pose.position.x;
	last_opose[1] = od.pose.pose.position.y;
	last_opose[2] = tf::getYaw( od.pose.pose.orientation );
	valid_opose = true;
}

bool OdomWatcher::get_pose_estimate( double *x )
{
	if (valid_opose) {
		for (int i = 0; i < 3; i++)
			x[i] = last_opose[i];
	}
	return valid_opose;
}

=======

#include <fstream>
#include <ostream>
#include <cstdlib>
#include <istream>

>>>>>>> bb024844c8423d63a5856551e925d436f508727b
int main(int argc, char** argv)
{
    float xmin = atof(argv[1]);
    float ymin = atof(argv[2]);
    float xmax = atof(argv[3]);
    float ymax = atof(argv[4]);
    float res  = atof(argv[5]);

    Map m(xmin, ymin, xmax, ymax, res);

    Point p(0,0);
    
    m.init(p);
<<<<<<< HEAD
	safe_to_move = true;
	ros::init (argc, argv, "Oscar", ros::init_options::AnonymousName );
	ros::NodeHandle nh;

	ros::Publisher action = nh.advertise<geometry_msgs::Twist>( "cmd_vel", 1) ;
	ros::Subscriber lmssub = nh.subscribe( "scan", 10, &laser_cb );
	OdomWatcher pose_watcher( nh );
    // Setting open ranges 
    m.setOpen(Point(0,0), Point(1.5,1.5));
    m.AStar(Point(0, 0), std::vector
=======

    // Setting open ranges 
    m.setOpen(Point(0,0), Point(1.5,1.5));

>>>>>>> bb024844c8423d63a5856551e925d436f508727b
    // Setting blocked ranges
    m.setBlocked(Point(0,0), Point(-1.0, 0.56));
    m.setBlocked(Point(0,0), Point(-1.0, 0.66));
    m.setBlocked(Point(0,0), Point(-1.0, 0.76));
    m.setBlocked(Point(0,0), Point(-1.0, 0.86));
    m.setBlocked(Point(0,0), Point(-1.0, 0.96));
    m.setBlocked(Point(0,0), Point(-0.9, 0.96));
    m.setBlocked(Point(0,0), Point(-0.8, 0.96));
    m.setBlocked(Point(0,0), Point(-0.7, 0.96));
    m.setBlocked(Point(0,0), Point(-0.6, 0.96));
    m.setBlocked(Point(0,0), Point(-0.5, 0.96));


    // Example of how to print out occupancy matrix
    std::ofstream out;
    out.open("test.ppm");
    out << m;


    return 0;
}
