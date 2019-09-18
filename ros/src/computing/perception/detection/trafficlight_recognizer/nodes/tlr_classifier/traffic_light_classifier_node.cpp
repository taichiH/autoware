#include "traffic_light_classifier.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trafficlight_classifier");

    tlr_classifier::TrafficLightClassifierNode classifier;
    classifier.run();

    return 0;
}
