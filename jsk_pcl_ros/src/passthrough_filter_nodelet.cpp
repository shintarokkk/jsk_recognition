#include "jsk_pcl_ros/passthrough_filter.h"

#include <pluginlib/class_list_macros.h>

namespace jsk_pcl_ros
{

  void PassThroughFilter::onInit()
  {
    x_max_ = 10.0;
    x_min_ = -10.0;
    y_max_ = 10.0;
    y_min_ = -10.0;
    z_max_ = 10.0;
    z_min_ = -10.0;

    PassThroughFilterBase::onInit();
  }

  void PassThroughFilter::updateCondition()
  {
    ConditionPtr condp (new pcl::ConditionAnd<pcl::PointXYZRGB> ());

    float x_max, x_min, y_max, y_min, z_max, z_min;
    if ( x_max_ >= x_min_ ) {
      x_max = x_max_;
      x_min = x_min_;
    }
    else {
      x_max = x_min_;
      x_min = x_max_;
    }
    if ( y_max_ >= y_min_ ) {
      y_max = y_max_;
      y_min = y_min_;
    }
    else {
      y_max = y_min_;
      y_min = y_max_;
    }
    if ( z_max_ >= z_min_ ) {
      z_max = z_max_;
      z_min = z_min_;
    }
    else {
      z_max = z_min_;
      z_min = z_max_;
    }

    {
      ConditionPtr cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
      ComparisonPtr le (new Comparison ("x", pcl::ComparisonOps::LE, x_max));
      ComparisonPtr ge (new Comparison ("x", pcl::ComparisonOps::GE, x_min));
      cond->addComparison (le);
      cond->addComparison (ge);
      condp->addCondition(cond);
    }
    {
      ConditionPtr cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
      ComparisonPtr le (new Comparison ("y", pcl::ComparisonOps::LE, y_max));
      ComparisonPtr ge (new Comparison ("y", pcl::ComparisonOps::GE, y_min));
      cond->addComparison (le);
      cond->addComparison (ge);
      condp->addCondition(cond);
    }
    {
      ConditionPtr cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
      ComparisonPtr le (new Comparison ("z", pcl::ComparisonOps::LE, z_max));
      ComparisonPtr ge (new Comparison ("z", pcl::ComparisonOps::GE, z_min));
      cond->addComparison (le);
      cond->addComparison (ge);
      condp->addCondition(cond);
    }

    filter_instance_.setCondition (condp);
  }

  void PassThroughFilter::configCallback(jsk_pcl_ros::PassThroughFilterConfig &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock (mutex_);
    x_max_ = config.x_limit_max;
    x_min_ = config.x_limit_min;
    y_max_ = config.y_limit_max;
    y_min_ = config.y_limit_min;
    z_max_ = config.z_limit_max;
    z_min_ = config.z_limit_min;
    updateCondition();
  }

  template <class PackedComparison, typename Config>
  void PassThroughFilterBase<PackedComparison, Config>::filter(const sensor_msgs::PointCloud2ConstPtr &input,
                                                           const PCLIndicesMsg::ConstPtr& indices)
  {
    boost::mutex::scoped_lock lock (mutex_);
    pcl::PointCloud<pcl::PointXYZRGB> tmp_in, tmp_out;
    sensor_msgs::PointCloud2 out;
    fromROSMsg(*input, tmp_in);

    filter_instance_.setInputCloud(tmp_in.makeShared());
    if (indices) {
      pcl::IndicesPtr vindices;
      vindices.reset(new std::vector<int> (indices->indices));
      filter_instance_.setIndices(vindices);
    }
    filter_instance_.filter(tmp_out);
    if (tmp_out.points.size() > 0) {
      toROSMsg(tmp_out, out);
      pub_.publish(out);
    }
  }

  template <class PackedComparison, typename Config>
  void PassThroughFilterBase<PackedComparison, Config>::filter(const sensor_msgs::PointCloud2ConstPtr &input)
  {
    filter(input, PCLIndicesMsg::ConstPtr());
  }

  template <class PackedComparison, typename Config>
  void PassThroughFilterBase<PackedComparison, Config>::onInit()
  {
    ConnectionBasedNodelet::onInit();

    updateCondition();
    bool keep_organized;
    pnh_->param("keep_organized", keep_organized, false);
    pnh_->param("use_indices", use_indices_, false);
    pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);

    filter_instance_ = pcl::ConditionalRemoval<pcl::PointXYZRGB>(true);
    filter_instance_.setKeepOrganized(keep_organized);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&PassThroughFilterBase::configCallback, this, _1, _2);
    srv_->setCallback (f);
  }

  template <class PackedComparison, typename Config>
  void PassThroughFilterBase<PackedComparison, Config>::subscribe()
  {
    sub_input_.subscribe(*pnh_, "input", 1);
    if (use_indices_) {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(10);
      sub_indices_.subscribe(*pnh_, "indices", 1);
      sync_->connectInput(sub_input_, sub_indices_);
      sync_->registerCallback(boost::bind(&PassThroughFilterBase::filter, this, _1, _2));
    }
    else {
      sub_input_.registerCallback(&PassThroughFilterBase::filter, this);
    }
  }

  template <class PackedComparison, typename Config>
  void PassThroughFilterBase<PackedComparison, Config>::unsubscribe()
  {
    sub_input_.unsubscribe();
    if (use_indices_) {
      sub_indices_.unsubscribe();
    }
  }
}

typedef jsk_pcl_ros::PassThroughFilter PassThroughFilter;
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::PassThroughFilter, nodelet::Nodelet);
