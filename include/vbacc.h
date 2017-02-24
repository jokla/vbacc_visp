#include <visp3/gui/vpDisplayX.h>
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Polygon.h>
#include <std_msgs/Int8.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpQuaternionVector.h>
#include <visp/vpXmlParserCamera.h>
#include <visp/vpXmlParserHomogeneousMatrix.h>

#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/visual_features/vpFeatureDepth.h>
#include <visp/vpFeatureTranslation.h>
#include <visp/vpPolygon.h>
#include <visp/vpServo.h>


#include <visp_bridge/3dpose.h>
#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>



class vbacc
{
public:

  vbacc(ros::NodeHandle &nh);
  ~vbacc();
  void initializationVS();
  bool computeBaseTLDControlLaw();
  void spin();
  void getCameraInfoCb(const sensor_msgs::CameraInfoConstPtr &msg);
  void getObjectPolygonCb(const geometry_msgs::Polygon::ConstPtr &msg);
  void getStatusObjectPolygonCb(const std_msgs::Int8::ConstPtr  &status);
  void publishCmdVel();
  void publishCmdVelStop();


protected:

  //Display
  vpImage<unsigned char> I;
  vpDisplayX d;
  int m_width;
  int m_height;

  vpCameraParameters m_cam;

  // ROS
  ros::NodeHandle n;
  std::string cmdVelTopicName;
  std::string m_cameraInfoName;
  std::string m_statusObjectPolygonTopicName;
  std::string m_objectPolygonTopicName;
  std::string  m_cmdVelTopicName;
  ros::Subscriber m_cameraInfoSub;
  ros::Subscriber m_ObjectPolygonSub;
  ros::Subscriber m_statusObjectPolygonSub;
  ros::Publisher m_cmdVelPub;
  int m_freq;
  int m_mode;




  double m_servo_time_init;

  vpHomogeneousMatrix m_eMh;
  vpHomogeneousMatrix m_offset;

  // Servo Base to track an object
  vpPolygon m_obj_polygon;
  int m_status_obj_polygon;
  vpServo m_base_poly_task;

  vpFeaturePoint m_s;
  vpFeaturePoint m_sd;
  vpFeatureDepth m_s_Z;
  vpFeatureDepth m_s_Zd;
  vpMatrix m_tJe;
  vpMatrix m_eJe;
  vpImagePoint m_head_cog_des;
  vpImagePoint m_head_cog_cur;
  vpHomogeneousMatrix m_cMe;
  double m_Z;
  double m_Zd;
  vpColVector m_base_vel;
  double m_coeff;

  //conditions
  bool m_servo_enabled;
  bool m_camInfoIsInitialized;
  bool m_command_give_box;

};
