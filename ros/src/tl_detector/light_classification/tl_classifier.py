from styx_msgs.msg import TrafficLight


# This part is unnecessary to do since the simulator give the ground truth traffic light data.
# This Note that it is not required for you to use this class.
# It only exists to help you break down the classification problem into more manageable chunks. 


class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        return TrafficLight.UNKNOWN
