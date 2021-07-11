#ifndef CUP_LAYER_CUP_LAYER_H_
#define CUP_LAYER_CUP_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_math.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Int32MultiArray.h>
#include <iostream>
#include <vector>

namespace cup_layer
{

class cupNode{
  public:
    cupNode(int num,double _x,double _y){
      number=num;
      x = _x;
      y = _y;
    }
    int get_number(){
      return number;
    }
    double get_x(){
      return x;
    }
    double get_y(){
      return y;
    }
  private:
    int number;
    double x,y;
};

class CupLayer2 : public costmap_2d::Layer{
  public:
    CupLayer2();

    virtual ~CupLayer2();
    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
    void cup_callback(const std_msgs::Int32MultiArray& cup_info);

  private:
    ros::NodeHandle info_n;
    ros::Subscriber cup_sub = info_n.subscribe("cup_state",100,&CupLayer2::cup_callback,this); //this 是這個object(非class) 的指標，指向其記憶體位置
    int cup_num =24; 
    double cup_pose[24][2] = { {0.4, 0.3} , {1.2, 0.3} , {0.515,0.445 } , {1.085,0.445} ,
                                                             {0.1,0.67} , {0.4,0.956} , {0.8,1.1} , {1.2,1.27} , 
                                                             {1.655,1.065} , {1.955,1.005} , {1.655,1.335} , {1.955,1.395} , 
                                                             {0.4,2.7} , {1.2,2.7} , {0.515,2.555} , {1.085,2.555} ,
                                                             {0.1,2.33} , {0.4,2.044} , {0.8,1.9} , {1.2,1.73} , 
                                                             {1.655,1.935} , {1.655,1.655} , {1.955,1.995} , {1.955,1.605}
                                                          }; //不可以 cup_pose[變數名稱][變數名稱] 可參考variable length array
    std::vector<cupNode> cup_list ;
    int last_erase_cup=0; //前一次砍掉的杯子數量
    int left_cup =24; //這次更新後實際剩下的杯子數量

    double cup_max_x = 0.0;
    double cup_min_x = 100.0;
    double cup_max_y = 0.0;
    double cup_min_y = 100.0;

    void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};

}  // namespace cup_layer_2_old

#endif  // COSTMAP_2D_CUP_LAYER_H_