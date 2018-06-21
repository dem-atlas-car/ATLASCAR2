
// Exemplo de publicação de um tópico dentro de uma callback de subscrição

#include <ros/ros.h>

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    // Topic you want to publish
    pub_ = n_.advertise<PUBLISHED_MESSAGE_TYPE>("/published_topic", 1);

    // Topic you want to subscribe
    sub_ = n_.subscribe("/subscribed_topic", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const SUBSCRIBED_MESSAGE_TYPE& input)
  {
    PUBLISHED_MESSAGE_TYPE output;
    //.... do something with the input and generate the output...
    pub_.publish(output);
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

};  // End of class SubscribeAndPublish

int main(int argc, char** argv)
{
  // Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish");

  // Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
