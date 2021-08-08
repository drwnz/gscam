
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <gscam_x2/gscam_x2_nodelet.h>

PLUGINLIB_EXPORT_CLASS(gscam_x2::GSCamX2Nodelet, nodelet::Nodelet) 

namespace gscam_x2 {
  GSCamX2Nodelet::GSCamX2Nodelet() :
    nodelet::Nodelet(),
    gscam_x2_driver_(NULL),
    stream_thread_(NULL)
  {
  }

  GSCamX2Nodelet::~GSCamX2Nodelet() 
  {
    stream_thread_->join();
  }

  void GSCamX2Nodelet::onInit()
  {
    gscam_x2_driver_.reset(new gscam_x2::GSCamX2(this->getNodeHandle(), this->getPrivateNodeHandle()));
    stream_thread_.reset(new boost::thread(boost::bind(&GSCamX2::run, gscam_x2_driver_.get())));
  }
}
