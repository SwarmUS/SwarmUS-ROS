#include "swarmus_ros_simulation/interLocalization.h"

/*
Class function implementations
*/
InterLocalization::InterLocalization() {
    m_robotName = Simulation::getParamRobotName();
    m_interlocPub = m_nodeHandle.advertise<swarmus_ros_simulation::InterLocalization_grid>(
        "interlocalization_grid", 1000);
    m_polygonPub = m_nodeHandle.advertise<geometry_msgs::PolygonStamped>("PolygonStamped", 1000);

    ROS_INFO("HiveMind initialization of: %s", m_robotName.c_str());
}

float InterLocalization::getDistanceFrom(float x, float y) { return sqrt(pow(x, 2) + pow(y, 2)); }

float InterLocalization::getAnglefrom(float x, float y) { return atan2(y, x) * 180.0 / M_PI; }

void InterLocalization::publish(swarmus_ros_simulation::InterLocalization_grid grid) {
    m_interlocPub.publish(grid);
    m_polygonPub.publish(generatePolyMsg(grid));
}

std::string InterLocalization::getRobotName() { return m_robotName; }
geometry_msgs::PolygonStamped InterLocalization::generatePolyMsg(
    swarmus_ros_simulation::InterLocalization_grid grid) {
    geometry_msgs::Polygon poly;
    poly.points.reserve(2 * grid.otherRobotsListSize + 1);
    geometry_msgs::Point32 point;
    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.0;
    poly.points.push_back(point);

    for (int i = 0; i < grid.otherRobotsListSize; i++) {
        swarmus_ros_simulation::InterLocalization interLocalization = grid.otherRobots[i];
        Simulation::Angle angle(interLocalization.angle, false);
        point.x = interLocalization.distance * cos(angle.inRadians);
        point.y = interLocalization.distance * sin(angle.inRadians);
        poly.points.push_back(point);

        // Add a point in polygon to return to hiveboard
        point.x = 0.0;
        point.y = 0.0;
        point.z = 0.0;
        poly.points.push_back(point);
    }

    geometry_msgs::PolygonStamped msg;
    msg.polygon = poly;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = m_robotName + "/hiveboard";
    return msg;
}

/*
ROS main
*/
int main(int argc, char** argv) {
    ros::init(argc, argv, "InterLocalization");

    InterLocalization interloc;

    std::string reference = interloc.getRobotName() + gs_hiveBoardLink;
    swarmus_ros_simulation::InterLocalization_grid grid;
    grid.source_robot = reference;

    tf::TransformListener listener;

    ros::Rate loop_rate(10); // Probablement qu'on pourrait changer la boucle pour des evenements
                             // declenches par le board.

    while (ros::ok()) {
        int count = 0;
        for (std::string& m_robotName : Simulation::getRobotList()) {
            if (m_robotName == interloc.getRobotName()) {
                continue;
            }

            std::string target = m_robotName + gs_hiveBoardLink;

            tf::StampedTransform transform;
            try {
                listener.lookupTransform(reference, target, ros::Time(0), transform);
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }

            swarmus_ros_simulation::InterLocalization interlocMessage;
            interlocMessage.target_robot = target;
            interlocMessage.distance =
                interloc.getDistanceFrom(transform.getOrigin().x(), transform.getOrigin().y());
            interlocMessage.angle =
                interloc.getAnglefrom(transform.getOrigin().x(), transform.getOrigin().y());
            Simulation::Angle rotation(transform.getRotation().getAngle(), true);
            interlocMessage.rotation = rotation.inDegrees;
            grid.otherRobots.push_back(interlocMessage);
            count++;
        }

        grid.otherRobotsListSize = count;
        interloc.publish(grid);
        grid.otherRobots.clear();

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
