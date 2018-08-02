#include <memory>
#include <string>
#include <vector>

#include "boost/bind.hpp"
#include "ros/ros.h"
#include "ros/console.h"

#include "control_msgs/JointControllerState.h"
#include "std_msgs/Float64.h"
#include "hexapod_msgs/Leg.h"
#include "hexapod_msgs/HexapodState.h"


using namespace hexapod_msgs;


/**
 * Takes a JointControllerState message and uses it to update the reported hexapod state
 * @param msg The JointControllerState message
 * @param state Stored hexapod state to modify
 * @param joint The joint being reported by the JointControllerState
 */
void jointCallback(const control_msgs::JointControllerState::ConstPtr &msg,
                   std::shared_ptr<HexapodState> state, const std::string &joint) {
    auto destLeg = &(state->right_rear);
    const std::string sub{joint.substr(0, 2)};

    if (sub == "lf") {
        destLeg = &(state->left_front);
    } else if (sub == "lm") {
        destLeg = &(state->left_middle);
    } else if (sub == "lr"){
        destLeg = &(state->left_rear);
    }else if (sub == "rf"){
        destLeg = &(state->right_front);
    }else if (sub == "rm"){
        destLeg = &(state->right_middle);
    }

    const std::string legPart{joint.substr(3)};

    if (legPart == "thigh")
        destLeg->thigh = msg->process_value;
    else if (legPart == "knee")
        destLeg->knee = msg->process_value;
    else
        destLeg->ankle = msg->process_value;
}


/**
 * Updates stored setpoints with message contents
 * @param msg Desired setpoints
 * @param setpoint Stored setpoints to modify
 */
void hexapodSetpointCallback(const HexapodState::ConstPtr &msg, std::shared_ptr<HexapodState> setpoint){
    // Please tell me there is a better way to do this. Nothing else I've tried works.
    setpoint->left_front.thigh = msg->left_front.thigh;
    setpoint->left_front.knee = msg->left_front.knee;
    setpoint->left_front.ankle = msg->left_front.ankle;

    setpoint->left_middle.thigh = msg->left_middle.thigh;
    setpoint->left_middle.knee = msg->left_middle.knee;
    setpoint->left_middle.ankle = msg->left_middle.ankle;

    setpoint->left_rear.thigh = msg->left_rear.thigh;
    setpoint->left_rear.knee = msg->left_rear.knee;
    setpoint->left_rear.ankle = msg->left_rear.ankle;


    setpoint->right_front.thigh = msg->right_front.thigh;
    setpoint->right_front.knee = msg->right_front.knee;
    setpoint->right_front.ankle = msg->right_front.ankle;

    setpoint->right_middle.thigh = msg->right_middle.thigh;
    setpoint->right_middle.knee = msg->right_middle.knee;
    setpoint->right_middle.ankle = msg->right_middle.ankle;

    setpoint->right_rear.thigh = msg->right_rear.thigh;
    setpoint->right_rear.knee = msg->right_rear.knee;
    setpoint->right_rear.ankle = msg->right_rear.ankle;
}


/**
 * Sends stored setpoints to joint controllers
 * @param publishers Mapping from joint name to joint command publishers and joint state subscribers
 * @param setpoints Stored setpoints to send
 */
void publishSetpoints(
        const std::map<std::string, std::pair<ros::Subscriber, ros::Publisher>> &publishers,
        std::shared_ptr<HexapodState> setpoints
        ){

    for (const auto &publisher : publishers) {
        const std::string &key = publisher.first;
        const ros::Publisher &jointPub = publisher.second.second;

        auto sourceLeg = setpoints->right_rear;

        const std::string sub{key.substr(0, 2)};

        if (sub == "lf") {
            sourceLeg = setpoints->left_front;
        } else if (sub == "lm"){
            sourceLeg = setpoints->left_middle;
        } else if (sub == "lr"){
            sourceLeg = setpoints->left_rear;
        }else if (sub == "rf"){
            sourceLeg = setpoints->right_front;
        }else if (sub == "rm"){
            sourceLeg = setpoints->right_middle;
        }

        const std::string &legPart{key.substr(3)};
        double setpoint;

        if (legPart == "thigh")
            setpoint = sourceLeg.thigh;
        else if (legPart == "knee")
            setpoint = sourceLeg.knee;
        else
            setpoint = sourceLeg.ankle;

        std_msgs::Float64 msg;
        msg.data = setpoint;

        jointPub.publish(msg);
    }
}


/**
 * Publishes the stored hexapod state
 * @param state Stored hexapod state to publish
 * @param publisher HexapodState publisher
 */
void publishHexapodState(std::shared_ptr<HexapodState> state, const ros::Publisher &publisher){
    publisher.publish(*state);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "hexapod_interface");
    ros::NodeHandle handle;

    //  Vector containing all joint names
    std::vector<std::string> keys{
        "lf_thigh", "lf_knee", "lf_ankle",
        "lm_thigh", "lm_knee", "lm_ankle",
        "lr_thigh", "lr_knee", "lr_ankle",

        "rf_thigh", "rf_knee", "rf_ankle",
        "rm_thigh", "rm_knee", "rm_ankle",
        "rr_thigh", "rr_knee", "rr_ankle"
    };

    //  Shared pointers for stored hexapod state and desired state (setpoint)
    std::shared_ptr<HexapodState> state = std::make_shared<HexapodState>();
    std::shared_ptr<HexapodState> setpoint = std::make_shared<HexapodState>();

    // Holds a publisher and a subscriber for each joint
    std::map<std::string, std::pair<ros::Subscriber, ros::Publisher>> jointBridge;

    // For listening to desired hexapod states from client
    ros::Subscriber HexapodStateSub = handle.subscribe<HexapodState> (
            "/hexapod_interface/setpoint",
            5,
            boost::bind(hexapodSetpointCallback, _1, setpoint));

    // For publishing joint states from Gazebo, but in HexapodState format
    ros::Publisher hexapodStatePub = handle.advertise<HexapodState> ("/hexapod_interface/state", 5);


    //  Populate the jointBridge mapping
    for(const std::string &joint_name : keys) {
        jointBridge.insert(std::make_pair(joint_name, std::make_pair(
                handle.subscribe<control_msgs::JointControllerState>(
                        "/hexapod/" + joint_name + "/state",
                        20,
                        boost::bind(jointCallback, _1, state, joint_name)),
                handle.advertise<std_msgs::Float64>(
                        "/hexapod/" + joint_name + "/command",
                        20,
                        false)
        )));
    }


    //  Repeatedly update joint controllers with new setpoints
    ros::Duration publishSetpointsDelay{0.05};
    ros::Timer setpointPublisherTimer = handle.createTimer(
            publishSetpointsDelay,
            boost::bind(publishSetpoints, jointBridge, setpoint)
            );

    // Repeatedly update clients with new hexapod state
    ros::Duration publishHexapodStateDelay{0.05};
    ros::Timer hexapodStatePublisherTimer = handle.createTimer(
            publishHexapodStateDelay,
            boost::bind(publishHexapodState, state, hexapodStatePub)
            );

    ros::spin();
}