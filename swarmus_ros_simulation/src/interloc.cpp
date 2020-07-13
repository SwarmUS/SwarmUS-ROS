#include "swarmus_ros_simulation/interloc.h"


/*
Class function implementations
*/

Interloc::Interloc(std::string new_robot_name) {
  robot_name = new_robot_name;
  interloc_pub = n.advertise<swarmus_ros_simulation::Interloc_msg>("interloc", 1000);

  ROS_INFO("HiveBoard initialization of: %s", robot_name.c_str());
}

Interloc::~Interloc()
{
  delete[] robot_list;
}

float Interloc::getDistanceFrom(float x, float y) {
  return sqrt(pow((pos_x - x), 2) + pow((pos_y - y), 2));
}

float Interloc::getAnglefrom(float x, float y) {
  // calculer un angle depuis un point
  return atan2(y,x)*180.0/M_PI;
}


void Interloc::publish(std_msgs::String msg) {
    interloc_pub.publish(msg); 
}

void Interloc:: move(int delta_x, int delta_y) {
  pos_x += delta_x;
  pos_y += delta_y;
}

void Interloc::getRobotList(ros::NodeHandle nh)
{
  XmlRpc::XmlRpcValue v;
  if (!nh.getParam("/robot_list", v))
  {
    ROS_ERROR("No robot_list was found");
  }

  std::cout<<v.size()<<std::endl; // Only for debug
  robot_list = new  std::string[2];

  for(int i =0; i < v.size(); i++)
  {
    robot_list[i] = std::string(v[i]);
    std::cout<<robot_list[i] <<std::endl;  // Only for debug
    std::cout<<i<<std::endl;                // Only for debug
  }
  std::cout<<"sizeof robot list"<<sizeof(robot_list)/sizeof(robot_list[0])<<std::endl;  // Only for debug

}

/*
Helpers
*/
std::stringstream buildInterloc(float distance, float angle) {
  std::stringstream ss;
  ss << "[Robot 0] relative distance : " << distance << ", angle : " << angle;
  return ss;
}



/*
ROS main
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "interloc");
  ros::NodeHandle n;
  
  
  std::string robot_name;
  if (!ros::param::get("~robot_name",robot_name)) // The ~ is used to get param declared inside the <node></node> tags
  {
    ROS_INFO("No param name was given. pioneer_0 will be used instead");
    robot_name = "pioneer_0";
  }
  // Get the robot list
  Interloc interloc(robot_name);
  interloc.getRobotList(n);

  ros::Rate loop_rate(10); // Probablement qu'on pourrait changer la boucle pour des evenements declenches par le board.
  
  tf::TransformListener listener;

  while (ros::ok())
  {

    for(int i=0; i < interloc.robot_list->size(); i++ )
    {
      std::string name = interloc.robot_list[i];
      if (name == interloc.robot_name)
      {
        continue;
      }

      std::string reference = interloc.robot_name + HIVEBOARD_LINK;
      std::string target = name + HIVEBOARD_LINK;

      tf::StampedTransform transform;
      try{
        listener.lookupTransform(reference, target,  
                              ros::Time(0), transform);
      }
        catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }
        
      float dist = interloc.getDistanceFrom(transform.getOrigin().x(), transform.getOrigin().y());
      float angle = interloc.getAnglefrom(transform.getOrigin().x(), transform.getOrigin().y());
      ROS_INFO("Distance: %f, Angle: %f, iiteration: %d", dist, angle, interloc.robot_list->size());         // Only for debug
      
    }
    // TODO: faire une for loop qui passe dans robot_list et fait une tf avec lui meme sauf lui meme avec lui meme
    // TODO: caller les fonctions de distances et dangle
    // TODO: Filtre les données à envoyé selon leur distance
    // TODO: publier les messages 
    //ROS_INFO("X: %f, Y: %f, Z: %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
    
    /*
    float distance = interloc.getDistanceFrom(0,0);
    float angle = interloc.getAnglefrom(0,0);

    std_msgs::String msg;
    msg.data = buildInterloc(distance, angle).str();

    ROS_INFO("%s", msg.data.c_str());
    */

    // Loop and stuff
    // interloc.move(1, 1);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
