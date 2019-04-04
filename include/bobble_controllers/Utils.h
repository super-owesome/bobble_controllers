//
// Created by james on 4/3/19.
//

#ifndef SRC_UTILS_H
#define SRC_UTILS_H

#include <string>

double limit(double cmd, double max);

void unpackFlag(std::string parameterName, bool &referenceToFlag, bool defaultValue);

void unpackParameter(std::string parameterName, double &referenceToParameter, double defaultValue);
void unpackParameter(std::string parameterName, std::string &referenceToParameter, std::string defaultValue);

#endif //SRC_UTILS_H
