
#include <stdlib.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>


#include <iostream>
extern "C"{
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
}

#include <ros/ros.h>

#include <camera_info_manager/camera_info_manager.h>

#include <boost/thread/thread.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <camera_calibration_parsers/parse_ini.h>

#include <gscam_x2/gscam_x2.h>


namespace gscam_x2 {

  GSCamX2::GSCamX2(ros::NodeHandle nh_camera, ros::NodeHandle nh_private) :
    gsconfig_(""),
    pipeline_(NULL),
    sink_(NULL),
    nh_(nh_camera),
    nh_private_(nh_private),
    //image_(nh_camera),
    camera_info_manager_(nh_camera)
  {
  }

  GSCamX2::~GSCamX2()
  {
  }

  bool GSCamX2::configure()
  {
    // Get gstreamer configuration
    // (either from environment variable or ROS param)
    std::string gsconfig_rosparam = "";
    bool gsconfig_rosparam_defined = false;
    char *gsconfig_env = NULL;

    gsconfig_rosparam_defined = nh_private_.getParam("gscam_x2_config",gsconfig_rosparam);
    gsconfig_env = getenv("GSCAM_X2_CONFIG");

    if (!gsconfig_env && !gsconfig_rosparam_defined) {
      ROS_FATAL( "Problem getting GSCAM_X2_CONFIG environment variable and 'gscam_x2_config' rosparam is not set. This is needed to set up a gstreamer pipeline." );
      return false;
    } else if(gsconfig_env && gsconfig_rosparam_defined) {
      ROS_FATAL( "Both GSCAM_CONFIG environment variable and 'gscam_x2_config' rosparam are set. Please only define one." );
      return false;
    } else if(gsconfig_env) {
      gsconfig_ = gsconfig_env;
      ROS_INFO_STREAM("Using gstreamer config from env: \""<<gsconfig_env<<"\"");
    } else if(gsconfig_rosparam_defined) {
      gsconfig_ = gsconfig_rosparam;
      ROS_INFO_STREAM("Using gstreamer config from rosparam: \""<<gsconfig_rosparam<<"\"");
    }

    // Get additional gscam configuration
    nh_private_.param("sync_sink", sync_sink_, true);
    nh_private_.param("preroll", preroll_, false);
    nh_private_.param("use_gst_timestamps", use_gst_timestamps_, false);

    nh_private_.param("reopen_on_eof", reopen_on_eof_, false);

    // Get the camera parameters file
    nh_private_.getParam("camera_info_url", camera_info_url_);
    nh_private_.getParam("camera_name", camera_name_);

    // Get the triggering parameters
    nh_private_.getParam("frame_rate", fps_);
    nh_private_.getParam("phase", phase_);
    nh_private_.getParam("gpio", gpio_);

    // Get the image encoding
    nh_private_.param("image_encoding", image_encoding_, sensor_msgs::image_encodings::RGB8);
    if (image_encoding_ != sensor_msgs::image_encodings::RGB8 &&
        image_encoding_ != sensor_msgs::image_encodings::MONO8 &&
        image_encoding_ != sensor_msgs::image_encodings::YUV422 &&
        image_encoding_ != "png" &&
        image_encoding_ != "jpeg") {
      ROS_FATAL_STREAM("Unsupported image encoding: " + image_encoding_);
    }

    camera_info_manager_.setCameraName(camera_name_);

    if(camera_info_manager_.validateURL(camera_info_url_)) {
      camera_info_manager_.loadCameraInfo(camera_info_url_);
      ROS_INFO_STREAM("Loaded camera calibration from "<<camera_info_url_);
    } else {
      ROS_WARN_STREAM("Camera info at: "<<camera_info_url_<<" not found. Using an uncalibrated config.");
    }

    // Get TF Frame
    if(!nh_private_.getParam("frame_id",frame_id_)){
      frame_id_ = "/camera_frame";
      ROS_WARN_STREAM("No camera frame_id set, using frame \""<<frame_id_<<"\".");
      nh_private_.setParam("frame_id",frame_id_);
    }

    return true;
  }

  bool GSCamX2::init_stream()
  {
    if(!gst_is_initialized()) {
      // Initialize gstreamer pipeline
      ROS_DEBUG_STREAM( "Initializing gstreamer..." );
      gst_init(0,0);
    }

    ROS_DEBUG_STREAM( "Gstreamer Version: " << gst_version_string() );

    GError *error = 0; // Assignment to zero is a gst requirement

    pipeline_ = gst_parse_launch(gsconfig_.c_str(), &error);
    if (pipeline_ == NULL) {
      ROS_FATAL_STREAM( error->message );
      return false;
    }

    // Create RGB sink
    sink_ = gst_element_factory_make("appsink",NULL);
    GstCaps * caps = gst_app_sink_get_caps(GST_APP_SINK(sink_));

#if (GST_VERSION_MAJOR == 1)
    // http://gstreamer.freedesktop.org/data/doc/gstreamer/head/pwg/html/section-types-definitions.html
    if (image_encoding_ == sensor_msgs::image_encodings::RGB8) {
        caps = gst_caps_new_simple( "video/x-raw", 
            "format", G_TYPE_STRING, "RGB",
            NULL); 
    } else if (image_encoding_ == sensor_msgs::image_encodings::MONO8) {
        caps = gst_caps_new_simple( "video/x-raw", 
            "format", G_TYPE_STRING, "GRAY8",
            NULL); 
    } else if (image_encoding_ == sensor_msgs::image_encodings::YUV422) {
        caps = gst_caps_new_simple( "video/x-raw", 
            "format", G_TYPE_STRING, "UYVY",
            NULL); 
    } else if (image_encoding_ == "jpeg") {
        caps = gst_caps_new_simple("image/jpeg", NULL, NULL);
    } else if (image_encoding_ == "png") {
        caps = gst_caps_new_simple("image/png", NULL, NULL);
    }
#else
    if (image_encoding_ == sensor_msgs::image_encodings::RGB8) {
        caps = gst_caps_new_simple( "video/x-raw-rgb", NULL,NULL); 
    } else if (image_encoding_ == sensor_msgs::image_encodings::MONO8) {
        caps = gst_caps_new_simple("video/x-raw-gray", NULL, NULL);
    } else if (image_encoding_ == sensor_msgs::image_encodings::YUV422) {
        caps = gst_caps_new_simple("video/x-raw-yuv", NULL, NULL);
    } else if (image_encoding_ == "jpeg") {
        caps = gst_caps_new_simple("image/jpeg", NULL, NULL);
    } else if (image_encoding_ == "png") {
        caps = gst_caps_new_simple("image/png", NULL, NULL);
    }
#endif

    gst_app_sink_set_caps(GST_APP_SINK(sink_), caps);
    gst_caps_unref(caps);

    // Set whether the sink should sync
    // Sometimes setting this to true can cause a large number of frames to be
    // dropped
    gst_base_sink_set_sync(
        GST_BASE_SINK(sink_),
        (sync_sink_) ? TRUE : FALSE);

    if(GST_IS_PIPELINE(pipeline_)) {
      GstPad *outpad = gst_bin_find_unlinked_pad(GST_BIN(pipeline_), GST_PAD_SRC);
      g_assert(outpad);

      GstElement *outelement = gst_pad_get_parent_element(outpad);
      g_assert(outelement);
      gst_object_unref(outpad);

      if(!gst_bin_add(GST_BIN(pipeline_), sink_)) {
        ROS_FATAL("gst_bin_add() failed");
        gst_object_unref(outelement);
        gst_object_unref(pipeline_);
        return false;
      }

      if(!gst_element_link(outelement, sink_)) {
        ROS_FATAL("GStreamer: cannot link outelement(\"%s\") -> sink\n", gst_element_get_name(outelement));
        gst_object_unref(outelement);
        gst_object_unref(pipeline_);
        return false;
      }

      gst_object_unref(outelement);
    } else {
      GstElement* launchpipe = pipeline_;
      pipeline_ = gst_pipeline_new(NULL);
      g_assert(pipeline_);

      gst_object_unparent(GST_OBJECT(launchpipe));

      gst_bin_add_many(GST_BIN(pipeline_), launchpipe, sink_, NULL);

      if(!gst_element_link(launchpipe, sink_)) {
        ROS_FATAL("GStreamer: cannot link launchpipe -> sink");
        gst_object_unref(pipeline_);
        return false;
      }
    }

    // Calibration between ros::Time and gst timestamps
    GstClock * clock = gst_system_clock_obtain();
    ros::Time now = ros::Time::now();
    GstClockTime ct = gst_clock_get_time(clock);
    gst_object_unref(clock);
    time_offset_ = now.toSec() - GST_TIME_AS_USECONDS(ct)/1e6;
    ROS_INFO("Time offset: %.3f",time_offset_);

    gst_element_set_state(pipeline_, GST_STATE_PAUSED);

    if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      ROS_FATAL("Failed to PAUSE stream, check your gstreamer configuration.");
      return false;
    } else {
      ROS_DEBUG_STREAM("Stream is PAUSED.");
    }

    // Create ROS camera interface
    if (image_encoding_ == "jpeg") {
        jpeg_pub_ = nh_.advertise<sensor_msgs::CompressedImage>("image_raw/compressed",1);
        cinfo_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info",1);
    } else if (image_encoding_ == "png") {
        jpeg_pub_ = nh_.advertise<sensor_msgs::CompressedImage>("image_raw/compressed",1);
        cinfo_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info",1);
    } else {
        //camera_pub_ = image_.advertiseCamera("camera/image_raw", 1);
        camera_pub_ = nh_.advertise<sensor_msgs::Image>("image_raw",1);
        cinfo_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info",1);
    }

    return true;
  }

  bool GSCamX2::init_triggering()
  {
    // Not yet implemented
    return true;
  }

  void GSCamX2::publish_stream()
  {
    ROS_INFO_STREAM("Publishing stream...");

    // Pre-roll camera if needed
    if (preroll_) {
      ROS_DEBUG("Performing preroll...");

      //The PAUSE, PLAY, PAUSE, PLAY cycle is to ensure proper pre-roll
      //I am told this is needed and am erring on the side of caution.
      gst_element_set_state(pipeline_, GST_STATE_PLAYING);
      if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
        ROS_ERROR("Failed to PLAY during preroll.");
        return;
      } else {
        ROS_DEBUG("Stream is PLAYING in preroll.");
      }

      gst_element_set_state(pipeline_, GST_STATE_PAUSED);
      if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
        ROS_ERROR("Failed to PAUSE.");
        return;
      } else {
        ROS_INFO("Stream is PAUSED in preroll.");
      }
    }

    if(gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      ROS_ERROR("Could not start stream!");
      return;
    }
    ROS_INFO("Started stream.");

    // Poll the data as fast a spossible
    while(ros::ok()) 
    {
      // This should block until a new frame is awake, this way, we'll run at the
      // actual capture framerate of the device.
      // ROS_DEBUG("Getting data...");
#if (GST_VERSION_MAJOR == 1)
      GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink_));
      if(!sample) {
        ROS_ERROR("Could not get gstreamer sample.");
        break;
      }
      GstBuffer* buf = gst_sample_get_buffer(sample);
      GstMemory *memory = gst_buffer_get_memory(buf, 0);
      GstMapInfo info;

      gst_memory_map(memory, &info, GST_MAP_READ);
      gsize &buf_size = info.size;
      guint8* &buf_data = info.data;
#else
      GstBuffer* buf = gst_app_sink_pull_buffer(GST_APP_SINK(sink_));
      guint &buf_size = buf->size;
      guint8* &buf_data = buf->data;
#endif
      GstClockTime bt = gst_element_get_base_time(pipeline_);
      // ROS_INFO("New buffer: timestamp %.6f %lu %lu %.3f",
      //         GST_TIME_AS_USECONDS(buf->timestamp+bt)/1e6+time_offset_, buf->timestamp, bt, time_offset_);


#if 0
      GstFormat fmt = GST_FORMAT_TIME;
      gint64 current = -1;

       Query the current position of the stream
      if (gst_element_query_position(pipeline_, &fmt, &current)) {
          ROS_INFO_STREAM("Position "<<current);
      }
#endif

      // Stop on end of stream
      if (!buf) {
        ROS_INFO("Stream ended.");
        break;
      }

      // ROS_DEBUG("Got data.");

      // Get the image width and height
      GstPad* pad = gst_element_get_static_pad(sink_, "sink");
#if (GST_VERSION_MAJOR == 1)
      const GstCaps *caps = gst_pad_get_current_caps(pad);
#else
      const GstCaps *caps = gst_pad_get_negotiated_caps(pad);
#endif
      GstStructure *structure = gst_caps_get_structure(caps,0);
      gst_structure_get_int(structure,"width",&width_);
      gst_structure_get_int(structure,"height",&height_);

      // Update header information
      sensor_msgs::CameraInfo cur_cinfo = camera_info_manager_.getCameraInfo();
      sensor_msgs::CameraInfoPtr cinfo;
      cinfo.reset(new sensor_msgs::CameraInfo(cur_cinfo));
      if (use_gst_timestamps_) {
#if (GST_VERSION_MAJOR == 1)
          cinfo->header.stamp = ros::Time(GST_TIME_AS_USECONDS(buf->pts+bt)/1e6+time_offset_);
#else
          cinfo->header.stamp = ros::Time(GST_TIME_AS_USECONDS(buf->timestamp+bt)/1e6+time_offset_);
#endif
      } else {
          cinfo->header.stamp = ros::Time::now();
      }
      // ROS_INFO("Image time stamp: %.3f",cinfo->header.stamp.toSec());
      cinfo->header.frame_id = frame_id_;
      if (image_encoding_ == "jpeg") {
          sensor_msgs::CompressedImagePtr img(new sensor_msgs::CompressedImage());
          img->header = cinfo->header;
          img->format = "jpeg";
          img->data.resize(buf_size);
          std::copy(buf_data, (buf_data)+(buf_size),
                  img->data.begin());
          jpeg_pub_.publish(img);
          cinfo_pub_.publish(cinfo);
      } else if (image_encoding_ == "png") {
          sensor_msgs::CompressedImagePtr img(new sensor_msgs::CompressedImage());
          img->header = cinfo->header;
          img->format = "png";
          img->data.resize(buf_size);
          std::copy(buf_data, (buf_data)+(buf_size),
                  img->data.begin());
          jpeg_pub_.publish(img);
          cinfo_pub_.publish(cinfo);
      } else {
          // Complain if the returned buffer is smaller than we expect
          // const unsigned int expected_frame_size =
          //     image_encoding_ == sensor_msgs::image_encodings::RGB8
          //     ? width_ * height_ * 3
          //     : width_ * height_;

          const unsigned int expected_frame_size = width_ * height_ * (
                (image_encoding_ ==  sensor_msgs::image_encodings::RGB8)*3
                + (image_encoding_ ==  sensor_msgs::image_encodings::YUV422)*2
                + (image_encoding_ ==  sensor_msgs::image_encodings::MONO8)
              );

          if (buf_size < expected_frame_size) {
              ROS_WARN_STREAM( "GStreamer image buffer underflow: Expected frame to be "
                      << expected_frame_size << " bytes but got only "
                      << (buf_size) << " bytes. (make sure frames are correctly encoded)");
          }

          // Construct Image message
          sensor_msgs::ImagePtr img(new sensor_msgs::Image());

          img->header = cinfo->header;

          // Image data and metadata
          img->width = width_;
          img->height = height_;
          img->encoding = image_encoding_;
          img->is_bigendian = false;
          img->data.resize(expected_frame_size);

          // Copy only the data we received
          // Since we're publishing shared pointers, we need to copy the image so
          // we can free the buffer allocated by gstreamer
          if (image_encoding_ == sensor_msgs::image_encodings::RGB8) {
              img->step = width_ * 3;
          } else if (image_encoding_ == sensor_msgs::image_encodings::YUV422) {
              img->step = width_ * 2;
          } else {
              img->step = width_;
          }
          std::copy(
                  buf_data,
                  (buf_data)+(buf_size),
                  img->data.begin());

          // Publish the image/info
          cinfo_pub_.publish(cinfo);
          camera_pub_.publish(img);
      }

      // Release the buffer
      if(buf) {
#if (GST_VERSION_MAJOR == 1)
        gst_memory_unmap(memory, &info);
        gst_memory_unref(memory);
#endif
        gst_buffer_unref(buf);
      }

      ros::spinOnce();
    }
  }

  void GSCamX2::triggering(double fps, double phase, int gpio_pin)
  {
    // Configure the GPIO
    mraa::Gpio gpio(gpio_pin);
    // Set GPIO to output
    mraa::Result status = gpio.dir(mraa::DIR_OUT);
    if (status != mraa::SUCCESS) {
        ROS_ERROR("GPIO initialization error: %d", status);
        ROS_ERROR("No trigger will be generated");
    }
    // Start on the first time after TOS
    int32_t start_nsec;
    int32_t end_nsec;
    int32_t target_nsec;
    int32_t interval_nsec = (int32_t)(1e9 / fps);
    int32_t offset = 1e3;
    int32_t pulse_width = 5e6;
    // Fix this later to remove magic numbers
    if (std::abs(phase) <= 1e-7) {
      start_nsec = 0;
    }
    else {
      start_nsec = (int32_t)interval_nsec * (phase / 360.0);
    }
    target_nsec = start_nsec;
    end_nsec = start_nsec - interval_nsec + 1e9;

    while (ros::ok())
    {
      // Do triggering stuff
      // Check current time - assume ROS uses best clock source
      // Fix this later to remove magic numbers
      ros::Time now = ros::Time::now();
      // Wait until the last 10 millisecond
      while (now.nsec > target_nsec && now.nsec < end_nsec) {
        target_nsec = target_nsec + interval_nsec >= 1e9 ? start_nsec : target_nsec + interval_nsec;        
      }
      int32_t wait_nsec = target_nsec < now.nsec ? 1e9 - now.nsec + target_nsec - 1e7 : target_nsec - now.nsec - 1e7;
      ros::Duration(0, wait_nsec).sleep();

      // Block the last millisecond
      now = ros::Time::now();
      while (now.nsec < target_nsec || (target_nsec == start_nsec && now.nsec > end_nsec))
      {
        now = ros::Time::now();
      }
      // Trigger!
      status = gpio.write(1);
      ros::Time now2 = ros::Time::now();
      ros::Duration(0, pulse_width).sleep();
      status = gpio.write(0);  
      target_nsec = target_nsec + interval_nsec >= 1e9 ? start_nsec : target_nsec + interval_nsec;
      //ROS_INFO("Time: %d, trigger interval: %d", now.nsec, (now2 - now).nsec);
    }
  }

  void GSCamX2::cleanup_stream()
  {
    // Clean up
    ROS_INFO("Stopping gstreamer pipeline...");
    if(pipeline_) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      gst_object_unref(pipeline_);
      pipeline_ = NULL;
    }
  }

  void GSCamX2::run() {
    while(ros::ok()) {
      if(!this->configure()) {
        ROS_FATAL("Failed to configure gscam!");
        break;
      }

      if(!this->init_triggering()) {
        ROS_FATAL("Failed to initialize triggering!");
        break;
      }

      if(!this->init_stream()) {
        ROS_FATAL("Failed to initialize gscam stream!");
        break;
      }

      // Spawn triggering thread
      boost::thread triggering_thread(triggering, fps_, phase_, gpio_);

      // Block while publishing
      this->publish_stream();

      // Wait for triggering thread to end
      triggering_thread.join();

      this->cleanup_stream();

      ROS_INFO("GStreamer stream stopped!");

      if(reopen_on_eof_) {
        ROS_INFO("Reopening stream...");
      } else {
        ROS_INFO("Cleaning up stream and exiting...");
        break;
      }
    }
  }

  // Example callbacks for appsink
  // TODO: enable callback-based capture
  void gst_eos_cb(GstAppSink *appsink, gpointer user_data ) {
  }
  GstFlowReturn gst_new_preroll_cb(GstAppSink *appsink, gpointer user_data ) {
    return GST_FLOW_OK;
  }
  GstFlowReturn gst_new_asample_cb(GstAppSink *appsink, gpointer user_data ) {
    return GST_FLOW_OK;
  }

}
