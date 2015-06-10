#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "tf/tf.h"

#include <cmath>
#include <climits>
#include <string>
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <ostream>
#include <istream>

#include "map.h"

int min_dist = .5;
int scan_number;
bool safe_to_move;
double rangeholder[700];
float min_angle, max_angle, max_range;

double IndexToAngle( int index, const int numAngles, const double min, const double max){
    double angle;
    angle = (max - min)*(double)index/numAngles + min;
    return angle;
}

void laser_cb( const sensor_msgs::LaserScan &scan )
{
    max_range = scan.range_max;
        min_angle = scan.angle_min;
    max_angle = scan.angle_max;
        scan_number = scan.ranges.size();
    for (int i = 0; i < scan.ranges.size(); i++){
        rangeholder[i] = scan.ranges[i];
}
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

int main(int argc, char** argv)
{
    printf("Start Main\n");

    float xmin;   
    float ymin;   
    float xmax;   
    float ymax;   
    float res;    
    float inputx; 
    float inputy; 
    float theta;  
    float goalx;  
    float goaly;

    // Handle input
    if (argc != 11)
    {
        std::cerr << "Error! Num args " << argc << "\n"
                  << "Usage: ./main xmin ymin xmax ymax "
                  << "res startx starty theta goalx goaly\n";
        exit(0);
    }
    else
    {
        xmin   = atof(argv[ 1]);
        ymin   = atof(argv[ 2]);
        xmax   = atof(argv[ 3]);
        ymax   = atof(argv[ 4]);
        res    = atof(argv[ 5]);
        inputx = atof(argv[ 6]);
        inputy = atof(argv[ 7]);
        theta  = atof(argv[ 8]);
        goalx  = atof(argv[ 9]);
        goaly  = atof(argv[10]);
    }

    // Set up map
    Map m(xmin, ymin, xmax, ymax, res);
    Point goal(goalx, goaly);
    m.init(goal);

    // Initialize map related objects
    std::vector<Point> waypoints;
    Point Current;
    double Pointing;
    std::ofstream out;

    // Initialize robotic navigation variables
    double pointing, current_pose[3];
    double min_reach_err = res/2;
    double min_angle_err = .1;
    double forward_speed = 1.3;
    double turning_rate = .5;
    double angle_diff;
    int current_index = 0;
    int initial = 1;
    safe_to_move = true;
    int waypointset = 3;

    // Set up navigation 
    ros::init (argc, argv, "Oscar", ros::init_options::AnonymousName );
    ros::NodeHandle nh;
    ros::Publisher action = nh.advertise<geometry_msgs::Twist>( "cmd_vel", 1);
    ros::Subscriber lmssub = nh.subscribe( "scan", 10, &laser_cb );
    OdomWatcher pose_watcher( nh );
    ros::Rate rate( 15. );

    // While navigating
    while(ros::ok())
    {
    
        pose_watcher.get_pose_estimate( current_pose );
        geometry_msgs::Twist mot;

        // Update current pose with initial displacement
        current_pose[0] += inputx;
        current_pose[1] += inputy;
        current_pose[2] += theta;

        Current.x = current_pose[0];
        Current.y = current_pose[1];

        // Check rangefinder is returning readings
        if (scan_number != 0)
        {
            for (int i = 0; i <scan_number; i = i+6)
            {
                Point range_edge;
                Pointing = IndexToAngle(i, scan_number, min_angle, max_angle) +
                           current_pose[2];
                
                // If the readings have not been initialized yet, do nothing 
                if (isnan(Pointing ))
                    printf("NAN!!!!!\n");
        
                // Skip if rangeholder value is nan
                if (isnan(rangeholder[i]))
                {
                    continue;
                }   
                // If the readings HAVE initialized, include them into the map
                else if ( rangeholder[i] > max_range - .01){
                    range_edge.x = max_range * cos( Pointing ) + Current.x;
                    range_edge.y = max_range * sin( Pointing ) + Current.y;
                    m.setOpen(Current, range_edge);
                }

                else {
                    range_edge.x = rangeholder[i] * cos( Pointing ) + Current.x;
                    range_edge.y = rangeholder[i] * sin( Pointing ) + Current.y;
                    m.setBlocked(Current, range_edge);
                }
            }

            out.open("test.ppm");
            out << m;
            out.close();          
                
            if ((waypointset == 3) || initial == 1)
            {
                initial = 0;
                m.AStar(Current);
                int recent = m.getRecent();

                // Check if A* was successful
                if (recent == 1)
                {
                    waypointset = 0;
                    current_index = 1;
                    waypoints = m.getPath();
                }
            }

            // Calculate angle to waypoint.
            angle_diff = atan2(waypoints[current_index-1].y - current_pose[1],
                               waypoints[current_index-1].x - current_pose[0]) -
                                current_pose[2];

            // Fix angle between -PI and PI
            angle_diff = atan2( sin(angle_diff), cos(angle_diff) );

            printf("Next Waypoint: %f %f\n", waypoints[current_index-1].x,
                                             waypoints[current_index-1].y);
            // Navigate towards waypoints
            if (sqrt( (waypoints[current_index-1].x - current_pose[0]) *
                      (waypoints[current_index-1].x - current_pose[0]) +
                      (waypoints[current_index-1].y - current_pose[1]) *
                      (waypoints[current_index-1].y - current_pose[1]) )
                < min_reach_err)
            {
                std::cout << "Reached waypoint " << current_index << std::endl;
                current_index += 1;
                waypointset += 1;
            }
            else if (fabs( angle_diff ) > min_angle_err)
            {

                if (angle_diff > 0)
                {
                    mot.angular.z = turning_rate;
                }
                else
                {
                    mot.angular.z = -turning_rate;
                }

            }
            else 
            {
                mot.linear.x = forward_speed;
            }
        }

        
        action.publish( mot ); 
        ros::spinOnce();
        rate.sleep();

        std::cout << current_index << " " << waypoints.size() << "\n";

        // if (current_index == waypoints.size()-1 )
        if (sqrt( (Current.x - goal.x) * (Current.x - goal.x) +
                  (Current.y - goal.y) * (Current.y - goal.y) )
            < 2 * res)
        {
            printf("Reached Goal!\n");
            return 0;
        }
    }
}
