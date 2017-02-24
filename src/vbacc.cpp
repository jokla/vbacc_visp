#include <iostream>
#include <vector>
#include <algorithm>


#include <visp/vpFeatureBuilder.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMeterPixelConversion.h>

#include "vbacc.h"


vbacc::vbacc(ros::NodeHandle &nh): m_cam(),  m_camInfoIsInitialized(false), m_width(640), m_height(480)
{
  // read in config options
  n = nh;

  m_servo_time_init = 0;

  n.param( "frequency", m_freq, 20);
  n.param<std::string>("cameraInfoName", m_cameraInfoName, "/camera/camera_info");
  n.param<std::string>("objectPolygonTopicName", m_objectPolygonTopicName, "/vision/predator_alert");
  n.param<std::string>("cmdVelTopicName", m_cmdVelTopicName, "/cmd_vel");

  n.param<std::string>("objectStatusPolygonTopicName", m_statusObjectPolygonTopicName, "/vision/predator_alert/status");

  m_cameraInfoSub = n.subscribe( m_cameraInfoName, 1, (boost::function < void(const sensor_msgs::CameraInfoConstPtr & )>) boost::bind( &vbacc::getCameraInfoCb, this, _1 ));

  m_statusObjectPolygonSub = n.subscribe( m_statusObjectPolygonTopicName, 1, (boost::function < void(const std_msgs::Int8::ConstPtr & )>) boost::bind( &vbacc::getStatusObjectPolygonCb, this, _1 ));
  m_ObjectPolygonSub = n.subscribe( m_objectPolygonTopicName, 1, (boost::function < void(const geometry_msgs::Polygon::ConstPtr & )>) boost::bind( &vbacc::getObjectPolygonCb, this, _1 ));

  m_cmdVelPub  = n.advertise<geometry_msgs::TwistStamped>(m_cmdVelTopicName, 1000);


}

vbacc::~vbacc(){

}


void  vbacc::initializationVS()
{
  I.resize(m_height, m_width, 0);
  d.init(I);
  vpDisplay::setTitle(I, "ViSP viewer");

  // Visual Servoing using the COGx and the Area of the polygon
  m_base_poly_task.setServo(vpServo::EYEINHAND_L_cVe_eJe) ;
  m_base_poly_task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);
  //    vpAdaptiveGain lambda_adapt;
  //    lambda_adapt.initStandard(1.6, 1.8, 15);
  vpAdaptiveGain lambda_base(3.2, 2.0, 24);
  m_base_poly_task.setLambda(lambda_base) ;

  vpImagePoint ip;
  ip.set_uv(m_width/2., m_height/2.);
  m_head_cog_des.set_uv(m_width/2., m_height/2.);
  // Create the current x visual feature
  vpFeatureBuilder::create(m_s, m_cam, ip);
  vpFeatureBuilder::create(m_sd, m_cam, ip);

  // Add the feature
  m_base_poly_task.addFeature(m_s, m_sd, vpFeaturePoint::selectX()) ;

  m_coeff = 1.74;
  m_Z = 6.0;
  m_Zd = m_Z;

  m_s_Z.buildFrom(m_s.get_x(), m_s.get_y(), m_Z , 0); // log(Z/Z*) = 0 that's why the last parameter is 0
  m_s_Zd.buildFrom(m_sd.get_x(), m_sd.get_y(), m_Zd , 0);

  // Add the feature
  m_base_poly_task.addFeature(m_s_Z, m_s_Zd);

  // vx and vy
  m_eJe.resize(6,2);
  m_eJe[0][0]= 1;
  m_eJe[5][1]= 1;

  m_cMe.eye();
  // Position of the camera in the mobile platform frame
  double l_y = 0.25; // distance between the camera frame and the car frame (along y-cam)
  double l_z = -1.0; // distance between the camera frame and the car frame (along z-cam)

  vpTranslationVector cte; // meters
  vpRxyzVector        cre; // radian
  cte.set(0, l_y, l_z);
  cre.buildFrom(vpMath::rad(90.), 0, vpMath::rad(90.));
  m_cMe.buildFrom(cte, vpRotationMatrix(cre));

  std::cout << "cMe:" << m_cMe << std::endl;

  std::cout << "Visual Servoing initialized" << std::endl;

}



void vbacc::spin()
{
  ros::Rate loop_rate(m_freq);
  vpDisplay::display(I);


  while(!m_camInfoIsInitialized)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Initialize Visual servoing variables
  this->initializationVS();

  while(ros::ok()){
    vpDisplay::display(I);

    vpMouseButton::vpMouseButtonType button;
    bool ret = vpDisplay::getClick(I, button, false);


    if (ret && button == vpMouseButton::button2)
    {
      m_servo_enabled = !m_servo_enabled;
      ret = false;
    }

    if(m_status_obj_polygon == 1)
      m_obj_polygon.display(I,vpColor::blue);

    //std::cout << "Area:" << m_obj_polygon.getArea() << std::endl;
    //std::cout << "u:" << m_obj_polygon.getCenter().get_u() << std::endl;
    // std::cout << "v:" << m_obj_polygon.getCenter().get_v() << std::endl;
    // std::cout << "m_obj_polygon.getCenter():" <<   m_obj_polygon.getCenter() << std::endl;
    //double surface = 1./sqrt(m_obj_polygon.getArea()/(m_cam.get_px()*m_cam.get_py()));
    //std::cout << "surface: " << surface << std::endl;
    //std::cout << "dist: " << m_coeff * surface << std::endl;
    //std::cout << "coeff: " << 0.58/surface << std::endl;

    if (m_servo_enabled && !this->computeBaseTLDControlLaw())
      vpDisplay::displayText(I, 30, 30, "Servo Base TDL enabled", vpColor::green);
    else
    {
      this->publishCmdVelStop();

      vpDisplay::displayText(I, 30, 30, "Middle click to enable the base VS", vpColor::green);
    }


    if (ret && button == vpMouseButton::button3)
      break;

    ret = false;
    ros::spinOnce();
    vpDisplay::flush(I);

    loop_rate.sleep();
  }

}




bool vbacc::computeBaseTLDControlLaw()
{
  bool vs_finished = false;

  if ( m_status_obj_polygon == 1 )
  {
    static bool first_time = true;

    if (first_time) {
      std::cout << "-- Start visual servoing of the base" << std::endl;
      m_servo_time_init = vpTime::measureTimeSecond();
      first_time = false;
    }

    m_head_cog_cur = m_obj_polygon.getCenter();

    vpDisplay::displayCross(I, m_head_cog_des, 10, vpColor::red);
    vpDisplay::displayCross(I, m_head_cog_cur, 10, vpColor::green);


    std::cout << "m_head_cog_des" << m_head_cog_des << std::endl;
    std::cout << "m_head_cog_cur" << m_head_cog_cur << std::endl;

    std::cout << "m_eJe:" << m_eJe << std::endl;


    m_base_poly_task.set_eJe( m_eJe );
    m_base_poly_task.set_cVe( vpVelocityTwistMatrix(m_cMe) );

    // Compute distanze box camera
    double surface = 1./sqrt(m_obj_polygon.getArea()/(m_cam.get_px()*m_cam.get_py()));
    m_Z = m_coeff * surface;

    std::cout << "Distance" << m_Z << std::endl;

    vpFeatureBuilder::create(m_s, m_cam, m_head_cog_cur);

    // Update log(Z/Z*) feature. Since the depth Z change, we need to update the intection matrix
    m_s_Z.buildFrom(m_s.get_x(), m_s.get_y(), m_Z, log(m_Z/m_Zd)) ;

    m_base_vel = m_base_poly_task.computeControlLaw(vpTime::measureTimeSecond() - m_servo_time_init);

    this->publishCmdVel();


    std::cout << "  ERROR:  " <<   m_base_poly_task.getError() << std::endl;
    std::cout << "  m_base_vel:  " <<   m_base_vel << std::endl;

  }
  else
    this->publishCmdVelStop();
  return vs_finished;
}


void vbacc::getStatusObjectPolygonCb(const std_msgs::Int8::ConstPtr  &status)
{
  m_status_obj_polygon = status->data;
}

void vbacc::getCameraInfoCb(const sensor_msgs::CameraInfoConstPtr &msg)
{
  std::cout << "Received Camera INFO"<<std::endl;
  // Convert the paramenter in the visp format
  m_cam = visp_bridge::toVispCameraParameters(*msg);
  m_cam.printParameters();

  m_width = msg->width;
  m_height = msg->height;

  // Stop the subscriber (we don't need it anymore)
  this->m_cameraInfoSub.shutdown();

  m_camInfoIsInitialized = 1;
}

void vbacc::getObjectPolygonCb(const geometry_msgs::Polygon::ConstPtr &msg)
{
  double x0 = msg->points[0].y;
  double y0 = msg->points[0].x;
  double w = msg->points[1].y;
  double h = msg->points[1].x;

  std::vector<vpImagePoint> corners;
  // Initialize the corners vector with 4 points
  corners.push_back( vpImagePoint(x0, y0) );
  corners.push_back( vpImagePoint(x0 , y0 + h) );
  corners.push_back( vpImagePoint(x0 + w, y0 + h) );
  corners.push_back( vpImagePoint(x0 + w, y0 ) );
  m_obj_polygon.buildFrom(corners);
}

void vbacc::publishCmdVel()
{
  geometry_msgs::TwistStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.twist.linear.x = m_base_vel[0];
  msg.twist.linear.y = 0.0;
  msg.twist.linear.z = 0.0;
  msg.twist.angular.x = 0;
  msg.twist.angular.y = 0;
  msg.twist.angular.z = m_base_vel[1];

  m_cmdVelPub.publish(msg);

}

void vbacc::publishCmdVelStop()
{
  geometry_msgs::TwistStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.twist.linear.x = 0.0;
  msg.twist.linear.y = 0.0;
  msg.twist.linear.z = 0.0;
  msg.twist.angular.x = 0.0;
  msg.twist.angular.y = 0.0;
  msg.twist.angular.z = 0.0;

  m_cmdVelPub.publish(msg);

}



