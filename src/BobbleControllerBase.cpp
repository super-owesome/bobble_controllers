#include <bobble_controllers/BobbleControllerBase.h>

using namespace bobble_controllers;


void BobbleControllerBase::subscriberFunction(){
    ros::NodeHandle n;
    std::vector<ros::Subscriber*> activeSubscribers;

    /// add all of the subscribers
    for (std::map<std::string, callbackFunctionPtr_t>::iterator it = subscribers.begin(); it != subscribers.end(); ++it) {
        ros::TransportHints rosTH;
        activeSubscribers.push_back(new ros::Subscriber(n.subscribe(it->first, 1, it->second, rosTH)));
    }

    ros::Rate loop_rate(subscriberFrequency);

    while (ros::ok() && runSubscriberThread) {
        /// Lock control data transfer mutex
        control_command_mutex.lock();
        /// Transfer all of the data defined
        for (std::map<std::string, double>::iterator it = controlDoubles.begin(); it != controlDoubles.end(); ++it) {
            it->second = controlDoublesNoRT.find(it->first)->second;
        }
        for (std::map<std::string, bool>::iterator it = controlBools.begin(); it != controlBools.end(); ++it) {
            it->second = controlBoolsNoRT.find(it->first)->second;
        }
        /// Unlock control data transfer mutex
        control_command_mutex.unlock();

        /// Sleep thread until next call
        loop_rate.sleep();
    }

        /// shutdown all of the subscribers
        for (std::vector<ros::Subscriber*>::iterator it = activeSubscribers.begin(); it != activeSubscribers.end(); ++it) {
            (*it)->shutdown();
            free(*it);
        }
}

void BobbleControllerBase::populateControlCommands()
{
    control_command_mutex.lock();
    for (std::map<std::string, double>::iterator it = controlDoubles.begin(); it != controlDoubles.end(); ++it) {
        controlDoublesRT.find(it->first)->second = it->second;
    }
    for (std::map<std::string, bool>::iterator it = controlBools.begin(); it != controlBools.end(); ++it) {
        controlBoolsRT.find(it->first)->second = it->second;
    }
    control_command_mutex.unlock();
}

void BobbleControllerBase::populateControlCommandNames()
{
    for (std::vector<std::string>::iterator it = controlBoolNames.begin(); it != controlBoolNames.end(); ++it){
        controlBools[*it] = false;
        controlBoolsRT[*it] = false;
        controlBoolsNoRT[*it] = false;
    }
    for (std::vector<std::string>::iterator it = controlDoubleNames.begin(); it != controlDoubleNames.end(); ++it){
        controlDoubles[*it] = 0.0;
        controlDoublesRT[*it] = 0.0;
        controlDoublesNoRT[*it] = 0.0;
    }
}

double BobbleControllerBase::limit(double cmd, double max)
{
    if (cmd < -max) {
        return -max;
    } else if (cmd > max) {
        return max;
    }
    return cmd;
}

