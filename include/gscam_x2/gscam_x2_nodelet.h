#ifndef __GSCAM_X2_GSCAM_X2_NODELET_H
#define __GSCAM_X2_GSCAM_X2_NODELET_H

#include <nodelet/nodelet.h>

#include <gscam_x2/gscam_x2.h>

#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>

namespace gscam_x2 {
  class GSCamX2Nodelet : public nodelet::Nodelet
  {
  public:
    GSCamX2Nodelet();
    ~GSCamX2Nodelet();

    virtual void onInit();

  private:
    boost::scoped_ptr<GSCamX2> gscam_x2_driver_;
    boost::scoped_ptr<boost::thread> stream_thread_;
  };
}

#endif // infdef __GSCAM_X2_GSCAM_X2_NODELET_H
