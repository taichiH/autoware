#include "trafficlight_classifier.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trafficlight_classifier");

    TrafficLightClassifier classifier;
    classifier.run();

    return 0;
}
