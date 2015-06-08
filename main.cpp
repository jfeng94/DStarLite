#include "map.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <cmath>
#include <climits>
#include <string>
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include "ros/ros.h"
#include "tf/tf.h"
#include <fstream>
#include <ostream>
#include <istream>

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
    float xmin = atof(argv[1]);
    float ymin = atof(argv[2]);
    float xmax = atof(argv[3]);
    float ymax = atof(argv[4]);
    float res  = atof(argv[5]);
    double pointing, current_pose[3];
    Map m(xmin, ymin, xmax, ymax, res);

    Point p(1.5,1.5);
    
    m.init(p);
    safe_to_move = true;
    ros::init (argc, argv, "Oscar", ros::init_options::AnonymousName );
    ros::NodeHandle nh;

    ros::Publisher action = nh.advertise<geometry_msgs::Twist>( "cmd_vel", 1);
    ros::Subscriber lmssub = nh.subscribe( "scan", 10, &laser_cb );
    OdomWatcher pose_watcher( nh );
    // Setting open ranges 
    /*
    m.setOpen(Point(0,0), Point(1.5,1.5));
    
    // Should be calling from current robot position
    m.AStar(Point(-1.9, 1.9));
    // Setting open ranges 
    m.setOpen(Point(0,0), Point(1.5,1.5));
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
    */
    Point Current;
    double Pointing;
    Current.x = current_pose[0];
    Current.y = current_pose[1];
ros::Rate rate( 30. );
int k = 0;
while(ros::ok()){
    // Example of how to print out occupancy matrix
    std::ofstream out;
geometry_msgs::Twist mot;
    printf("%d\n", scan_number);
    //for ( int i = 0; i < scan_number; i++){
   // for (int i= 0; i < 300; i++){
   for (int i = 0; i <386; i++){
    Pointing = IndexToAngle(i, scan_number, min_angle, max_angle) - current_pose[2];
    // All readings are open
     
      if ( rangeholder[i] > max_range){
            Point range_edge;
            range_edge.x = max_range * cos( Pointing );
            range_edge.y = max_range * sin( Pointing );
            printf("Pointing Beyond %d, %f\n", i, Pointing);
            printf("Range: %f\n", rangeholder[i]);
            m.setOpen(Current, range_edge);
            out.open("test.ppm");
            out << m;
            out.close();
       }
       else {
            Point range_edge;
            range_edge.x = rangeholder[i] * cos( Pointing );
            range_edge.y = rangeholder[i] * sin( Pointing );
            printf("Pointing Within: %d, %f\n", i, Pointing);
            printf("Range: %f\n", rangeholder[i]);
            m.setBlocked(Current, range_edge);
            out.open("test.ppm");
            out << m;
            out.close();
            }
            }
    //   }
    /*
   }
   printf("Paused\n");
    for (int i = 300; i < scan_number; i++){
     Pointing = IndexToAngle(i, scan_number, min_angle, max_angle) - current_pose[2];
      if ( rangeholder[i] > max_range){
            Point range_edge;
            range_edge.x = max_range * cos( Pointing );
            range_edge.y = max_range * sin( Pointing );
            printf("Pointing Beyond %d, %f\n", i, Pointing);
            m.setOpen(Current, range_edge);
            out.open("test.ppm");
            out << m;
            out.close();
       }
       else if ( rangeholder[i] < max_range){
            Point range_edge;
            range_edge.x = rangeholder[i] * cos( Pointing );
            range_edge.y = rangeholder[i] * sin( Pointing );
            printf("Pointing Within: %d, %f\n", i, Pointing);
            m.setBlocked(Current, range_edge);
            out.open("test.ppm");
            out << m;
            out.close();
    //   }
    }*/

    
    action.publish( mot ); 
    ros::spinOnce();
    rate.sleep();
    k++;
    if (k > 5){
        break;
    }

}


    return 0;
}