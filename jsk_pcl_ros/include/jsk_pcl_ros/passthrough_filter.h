#ifndef JSK_PCL_ROS_PASSTHROUGH_FILTER_H_
#define JSK_PCL_ROS_PASSTHROUGH_FILTER_H_

#include <nodelet/nodelet.h>
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/filters/conditional_removal.h>
#include "jsk_pcl_ros/PassThroughFilterConfig.h"
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <dynamic_reconfigure/server.h>

#include "jsk_recognition_utils/pcl_conversion_util.h"
#include "jsk_topic_tools/connection_based_nodelet.h"

namespace jsk_pcl_ros
{
  class PassThroughFilter;

  template<class PackedComparison, typename Config>
    class PassThroughFilterBase: public jsk_topic_tools::ConnectionBasedNodelet //same as jsk_pcl_ros::ColorFilter
  {
    friend class PassThroughFilter;
  public:
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2,
      PCLIndicesMsg> SyncPolicy;
    typedef typename pcl::ConditionBase<pcl::PointXYZRGB>::Ptr ConditionPtr;
    typedef typename pcl::ComparisonBase<pcl::PointXYZRGB>::Ptr ComparisonPtr;
    typedef PackedComparison Comparison;

  protected:
    boost::mutex mutex_;
    pcl::ConditionalRemoval<pcl::PointXYZRGB> filter_instance_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<PCLIndicesMsg> sub_indices_;
    ros::Publisher pub_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    virtual void configCallback(Config &config, uint32_t level) = 0;
    virtual void updateCondition() = 0;
    virtual void filter(const sensor_msgs::PointCloud2ConstPtr &input);
    virtual void filter(const sensor_msgs::PointCloud2ConstPtr &input,
                        const PCLIndicesMsg::ConstPtr& indices);
    virtual void subscribe();
    virtual void unsubscribe();
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;

    bool use_indices_;
  private:
    virtual void onInit();

  };

    class PassThroughFilter: public PassThroughFilterBase<pcl::FieldComparison<pcl::PointXYZRGB>, jsk_pcl_ros::PassThroughFilterConfig>
  {
  public:
  protected:
    float x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
    virtual void configCallback(jsk_pcl_ros::PassThroughFilterConfig &config, uint32_t level);
    virtual void updateCondition();
  private:
    virtual void onInit();
  };

}

#endif
