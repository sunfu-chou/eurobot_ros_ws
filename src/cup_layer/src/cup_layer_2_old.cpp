#include <cup_layer/cup_layer_2_old.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(cup_layer::CupLayer2, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace cup_layer
{

CupLayer2::CupLayer2() {}

void CupLayer2::onInitialize(){
  ros::NodeHandle nh("~/"+name_),info_n; //會吃輸入的name名稱(yaml檔裡面寫的)
  current_ = true;
  for(int i=0; i<cup_num;i++){
    cup_list.push_back( cupNode( i, cup_pose[i][0], cup_pose[i][1] ) );
  }
  
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  //動態參數的回調函數設置
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb; 
  cb = boost::bind(&CupLayer2::reconfigureCB,this,_1,_2);
  dsrv_->setCallback(cb);
}

void CupLayer2::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level){
  enabled_ = config.enabled;
}

void CupLayer2::cup_callback(const std_msgs::Int32MultiArray& cup_info){
  int cup_take = cup_info.data.size();
  if ( (cup_take == 0) || (cup_take == last_erase_cup) ){
    return;
  }
  else{
    left_cup = cup_num - cup_take;
    //把被拿走得杯子erase掉
    for( int i=0 ; i<left_cup ; i++ ){
      for( int j=last_erase_cup ; j<cup_take ; j++ ){
        if( cup_list[i].get_number() ==  cup_info.data[j] ){
          cup_list.erase( cup_list.begin()+i );
        }
        else{
          cup_max_x = std::max(cup_max_x , cup_list[i].get_x() );
          cup_min_x = std::min(cup_min_x , cup_list[i].get_x() );
          cup_max_y = std::max(cup_max_y , cup_list[i].get_y() );
          cup_min_y = std::min(cup_min_y , cup_list[i].get_y() );
        }
      }
    }
    last_erase_cup = cup_take;
  }
}

void CupLayer2::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,double* min_y, double* max_x, double* max_y){
  if(!enabled_)
    return;

  ros::spinOnce();

  // ROS_INFO("pose : %f  %f  %f ", origin_x, origin_y, origin_yaw);
  *min_x = std::min(cup_min_x, *min_x);
  *min_y = std::min(*min_y, cup_min_y);
  *max_x = std::max(*max_x, cup_max_x);
  *max_y = std::max(*max_y, cup_max_y);
}

void CupLayer2::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
  // ROS_INFO("left cup : %d ", left_cup);
  if(!enabled_)
    return;
  
  for( int i=0 ; i<left_cup ; i++ ){
    unsigned int mx;
    unsigned int my;
    if(master_grid.worldToMap(cup_list[i].get_x() , cup_list[i].get_y() , mx, my)){
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      // ROS_INFO("hhohohooh");
    }
  }
}

CupLayer2::~CupLayer2(){}

} //end namespace