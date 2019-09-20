#include "trafficlight_recognizer/traffic_light_classifier.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trafficlight_classifier");

    trafficlight_recognizer::TrafficLightClassifierNode classifier;
    classifier.run();

    return 0;
}
