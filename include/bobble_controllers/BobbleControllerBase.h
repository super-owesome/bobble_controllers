//
// Created by james on 4/4/19.
//

#ifndef SRC_BOBBLECONTROLLERBASE_H
#define SRC_BOBBLECONTROLLERBASE_H

#include <cstddef>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <thread>
#include <mutex>

#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <controller_interface/controller.h>

namespace bobble_controllers {

    /// Generic Controller Base
    template <class T> class BobbleControllerBase : public controller_interface::
    Controller<hardware_interface::EffortJointInterface> {

    protected:
        /// Node and hardware interface
        ros::NodeHandle node_;
        hardware_interface::RobotHW *robot_;

        /// Realtime status publisher
        realtime_tools::RealtimePublisher<T>* pub_status;

        /// Hardware interfaces
        std::vector <hardware_interface::JointHandle> joints_;
        hardware_interface::ImuSensorInterface *imu_;
        hardware_interface::ImuSensorHandle imuData;

        /// Mutex for transferring commands
        std::mutex control_command_mutex;

        /// Thread for managing non-RT subscriber
        std::thread* subscriberThread;
        /// Vector containing subscriber string to function definitions
        std::map<std::string, void*> subscribers;
        /// Subscriber frequency
        double subscriberFrequency;

        /// Vector for housing possible control modes
        std::map<std::string, unsigned int> controlModes;

        /// Vectors for housing possible command transfer elements
        std::map<std::string, double> controlDoubles;
        std::map<std::string, bool> controlBools;
        /// Second copy of vectors for non-RT access
        std::map<std::string, double> controlDoublesNoRT;
        std::map<std::string, bool> controlBoolsNoRT;

        /// Bool for telling subscriber thread to finish
        bool runSubscriberThread;
        /// Subscriber thread function
        void subscriberFunction() {
            ros::NodeHandle n;
            std::vector<ros::Subscriber> activeSubscribers;

            /// add all of the subscribers
            for (std::map<std::string, void *>::iterator it = subscribers.begin(); it != subscribers.end(); ++it) {
                activeSubscribers.push_back(n.subscribe(it->first, 1, &(it->second), this));
            }

            ros::Rate loop_rate(subscriberFrequency);

            while (ros::ok() && runSubscriberThread) {
                /// Lock control data transfer mutex
                control_command_mutex.lock();
                /// Transfer all of the data defined
                for (std::map<std::string, double>::iterator it = controlDoubles.begin();
                     it != controlDoubles.end(); ++it) {
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
            for (std::vector<ros::Subscriber>::iterator it = activeSubscribers.begin();
                 it != activeSubscribers.end(); ++it) {
                it->shutdown();
            }
        }

        /// Output limiting function
        double limit(double cmd, double max) {
            if (cmd < -max) {
                return -max;
            } else if (cmd > max) {
                return max;
            }
            return cmd;
        }

        /// Functions for unpacking flags and ros params
        void unpackFlag(std::string parameterName, bool &referenceToFlag, bool defaultValue)
        {
            if (!node_.getParam(parameterName, referenceToFlag)) {
                referenceToFlag = defaultValue;
                ROS_ERROR("%s not set for (namespace: %s). Setting to false.",
                          parameterName.c_str(),
                          node_.getNamespace().c_str());
            }
        }
        template <class refType> void unpackParameter(std::string parameterName, refType &referenceToParameter, refType defaultValue)
        {
            if (!node_.getParam(parameterName, referenceToParameter)) {
                referenceToParameter = defaultValue;
                ROS_ERROR("%s not set for (namespace: %s) using %f.",
                          parameterName.c_str(),
                          node_.getNamespace().c_str(),
                          defaultValue);
            }
        }
    };

}

#endif //SRC_BOBBLECONTROLLERBASE_H
