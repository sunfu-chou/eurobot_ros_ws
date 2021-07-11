#include <cup_layer/cup_layer_2.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/master.h>

PLUGINLIB_EXPORT_CLASS(cup_layer::CupLayer2, costmap_2d::Layer)

using costmap_2d::FREE_SPACE;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace cup_layer
{
CupLayer2::CupLayer2()
{
}

void CupLayer2::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), info_n;  //會吃輸入的name名稱(yaml檔裡面寫的)
  map_sub = info_n.subscribe("map_metadata", 1, &CupLayer2::Mapinfo, this);
  cup_sub = info_n.subscribe("plan_cup", 100, &CupLayer2::cup_callback,
                             this);  // this 是這個object(非class) 的指標，指向其記憶體位置

  while (map_sub.getNumPublishers() < 1)
    ;

  current_ = true;
  for (int i = 0; i < cup_num; i++)
  {
    cup_list.push_back(cupNode(i, cup_pose[i][0], cup_pose[i][1]));
  }
  nh.param("use_maximum", use_maximum_, false);

  rolling_window_ = layered_costmap_->isRolling();
  CupLayer2::matchSize();
  global_frame_ = layered_costmap_->getGlobalFrameID();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  //動態參數的回調函數設置
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb;
  cb = boost::bind(&CupLayer2::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void CupLayer2::reconfigureCB(costmap_2d::GenericPluginConfig& config, uint32_t level)
{
  enabled_ = config.enabled;
}

void CupLayer2::matchSize()
{
  if (!layered_costmap_->isRolling())
  {
    // ROS_INFO("wowowowowowoow");
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(), master->getOriginX(),
              master->getOriginY());
  }
}

void CupLayer2::cup_callback(const std_msgs::Int32MultiArray& cup_info)
{
  int cup_take = cup_info.data.size();
  if ((cup_take == 0) || (cup_take == last_erase_cup))
  {
    return;
  }
  else
  {
    left_cup = cup_num - cup_take;
    //把被拿走得杯子erase掉
    for (int i = 0; i < left_cup; i++)
    {
      for (int j = last_erase_cup; j < cup_take; j++)
      {
        if (cup_list[i].get_number() == cup_info.data[j])
        {
          cup_list.erase(cup_list.begin() + i);
        }
        // else{
        //   cup_max_x = std::max(cup_max_x , cup_list[i].get_x() );
        //   cup_min_x = std::min(cup_min_x , cup_list[i].get_x() );
        //   cup_max_y = std::max(cup_max_y , cup_list[i].get_y() );
        //   cup_min_y = std::min(cup_min_y , cup_list[i].get_y() );
        // }
      }
    }
    last_erase_cup = cup_take;
  }
}

void CupLayer2::Mapinfo(const nav_msgs::MapMetaDataConstPtr& map_info)
{
  map_resolution = map_info->resolution;
  map_originx = map_info->origin.position.x;
  map_originy = map_info->origin.position.y;
}

void CupLayer2::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y,
                             double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  useExtraBounds(min_x, min_y, max_x, max_y);
  ros::spinOnce();

  for (int i = 0; i < left_cup; i++)
  {
    touch(cup_list[i].get_x(), cup_list[i].get_y(), min_x, min_y, max_x, max_y);
  }
  // ROS_INFO("pose : %f  %f  %f ", origin_x, origin_y, origin_yaw);
  // *min_x = std::min(cup_min_x, *min_x);
  // *min_y = std::min(*min_y, cup_min_y);
  // *max_x = std::max(*max_x, cup_max_x);
  // *max_y = std::max(*max_y, cup_max_y);   //use touch() replace
}

void CupLayer2::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  // ROS_INFO("left cup : %d ", left_cup);
  if (!enabled_)
    return;

  if (!layered_costmap_->isRolling())
  {
    for (int i = 0; i < left_cup; i++)
    {
      unsigned int mx;
      unsigned int my;
      if (master_grid.worldToMap(cup_list[i].get_x(), cup_list[i].get_y(), mx, my))
      {
        master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      }
    }
  }
  else
  {
    unsigned int mx;
    unsigned int my;
    geometry_msgs::TransformStamped transform;
    try
    {
      transform = tf_->lookupTransform(global_frame_, "map", ros::Time(0));
    }
    catch (tf2::TransformException ex)
    {
      ROS_ERROR("cup layer has Exception problem %s", ex.what());
      return;
    }
    // ROS_INFO_STREAM("tf :"<< transform.transform);
    tf2::Transform tf2_transform;
    tf2::convert(transform.transform, tf2_transform);
    for (int i = 0; i < left_cup; i++)
    {
      tf2::Vector3 p(cup_list[i].get_x(), cup_list[i].get_y(), 0);
      p = tf2_transform * p;

      // worldToMap_calibrate(master_grid,p.x() , p.y() , mx, my)
      if (worldToMap_calibrate(master_grid, p.x(), p.y(), mx, my))
      {
        master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      }
    }
  }
}

bool CupLayer2::worldToMap_calibrate(costmap_2d::Costmap2D& master_grid, double wx, double wy, unsigned int& mx,
                                     unsigned int& my)
{
  if (fabs(layered_costmap_->getCostmap()->getResolution() - map_resolution) > 0.001)
  {
    master_grid.worldToMap(wx, wy, mx, my);
    return true;
  }
  else
  {
    double dx = (master_grid.getOriginX() - map_originx) / map_resolution;
    double dy = (master_grid.getOriginY() - map_originy) / map_resolution;
    mx = (int)((wx - map_originx) / map_resolution - dx);
    my = (int)((wy - map_originy) / map_resolution - dy);
    return true;
  }
  return false;
  // double dx = (master_grid.getOriginX() - map_originx)/map_resolution;
  // double dy = (master_grid.getOriginY() - map_originy)/map_resolution;
  // mx = (int)((wx - map_originx) / map_resolution - dx);
  // my = (int)((wy - map_originy) / map_resolution - dy);
  // return true;
}

CupLayer2::~CupLayer2()
{
}

}  // namespace cup_layer
