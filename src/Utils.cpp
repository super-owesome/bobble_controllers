//
// Created by james on 4/3/19.
//

void unpackParameter(std::string parameterName, double &referenceToParameter,
                                              double defaultValue) {
    if (!node_.getParam(parameterName, referenceToParameter)) {
        referenceToParameter = defaultValue;
        ROS_ERROR("%s not set for (namespace: %s) using %f.",
                  parameterName.c_str(),
                  node_.getNamespace().c_str(),
                  defaultValue);
    }
}

void unpackParameter(std::string parameterName, std::string &referenceToParameter, std::string defaultValue)
{
    if (!node_.getParam(parameterName, referenceToParameter)) {
        referenceToParameter = defaultValue;
        ROS_ERROR("%s not set for (namespace: %s) using %s.",
                  parameterName.c_str(),
                  node_.getNamespace().c_str(),
                  defaultValue.c_str());
    }
}

void unpackFlag(std::string parameterName, bool &referenceToFlag,
                                         bool defaultValue) {
    if (!node_.getParam(parameterName, referenceToFlag)) {
        referenceToFlag = defaultValue;
        ROS_ERROR("%s not set for (namespace: %s). Setting to false.",
                  parameterName.c_str(),
                  node_.getNamespace().c_str());
    }
}

double limit(double cmd, double max) {
    if (cmd < -max) {
        return -max;
    } else if (cmd > max) {
        return max;
    }
    return cmd;
}

