#ifndef PLANNER_LPASTAR_ALGORITHM_H
#define PLANNER_LPASTAR_ALGORITHM_H

#include <float.h>
#include <stdint.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <vector>

#include "grid_input.h"

typedef std::numeric_limits<float> INF_f;  // float最大值
void SearchOneMap(int map_num_);           //对一张地图进行ARA*搜索
void PrintSumResult();                     //打印结果统计

//每一个cell的信息，使用结构体定义
struct CellInfo {
  Points xoy;       //坐标点
  float h_value;    //当前状态到终点的启发距离值
  float g_value;    //起始点到当前点的距离值
  float rhs_value;  //引入的v(s)值

  inline float get_f_value() const {
    return std::min(rhs_value, g_value) + h_value;
  }

  bool operator<(const CellInfo& pos) const {
    // if f-vaule 相同，比较 g-vaule
    if ((std::min(rhs_value, g_value) + h_value) ==
        (std::min(pos.rhs_value, pos.g_value) + pos.h_value))
      return std::min(rhs_value, g_value) <
             std::min(pos.rhs_value, pos.g_value);
    else
      return (std::min(rhs_value, g_value) + h_value) <
             (std::min(pos.rhs_value, pos.g_value) + pos.h_value);
  }
};

class LPAstar {
 public:
  LPAstar(int16_t row_arg, int16_t column_arg, Points statr_arg,
          Points goal_arg,
          std::vector<Points> obstacle_list_arg);  //构造函数
  LPAstar(const LPAstar& as) = delete;             //不使用复制构造函数

  // A*算法
  bool AstarAlgorithm();

  //得到临近点的坐标点
  std::vector<Points> GetNeighborsPoint(const Points& current_pos);

  //打印一次搜索结果
  void PrintSearchResult();

  // LPA*特有的更新某点点的信息
  void UpdateVertex(const Points& pos);

  // A*算法循环判断函数
  bool LoopFlg();

  //弹出openlist中最小的元素
  void OpenLIstPopMinElem();

  //移除列表中的特定元素
  template <class T>
  void Remove(int16_t index, std::vector<T>* list);

  //更新当前点的临近点的信息
  void UpdataMapInfo();

  //计算h_value
  inline float DistenceToGoal(const Points& current) {
    return static_cast<float>(abs(current.first - goal_pos_.first) +
                              abs(current.second - goal_pos_.second));
  }

  //判断是否走到了终点
  inline bool ArriveGoal() const { return current_start_ == goal_pos_; }

  //沿着当前path前进一步
  inline void StartMove() {
    current_start_ = current_path_.back();
    current_path_.pop_back();
    ++move_step_nums_;
  }

  //判断要移动的下一个点是否为障碍物
  inline bool NextStepIsInObstacleList() {
    return IsInList(current_path_.back(), current_obstacle_list_);
  }

  //添加障碍物
  void ChangeOfInceaseObstacle(const Points& pos);

  //去除障碍物
  void ChangeOfDecreaseObstacle(const Points& pos);

  //打印计数结果
  void PrintCountResult();

  //**********获取类内私有成员函数**********//
  inline int get_row() const { return row_; }
  inline int get_column() const { return column_; }
  inline Points get_start_pos() const { return start_pos_; }
  inline Points get_goal_pos() const { return goal_pos_; }
  inline Points get_current_start() const { return current_start_; }
  inline std::vector<Points> get_map_obstacle_list() const {
    return map_obstacle_list_;
  }
  inline std::vector<Points> get_current_obstacle_list() const {
    return current_obstacle_list_;
  }
  inline std::vector<Points> get_current_path() const { return current_path_; }
  inline int get_all_expand_nums() const { return all_expand_points_count_; }
  inline int get_search_nums() const { return search_nums_count_; }
  inline int get_move_step_nums() const { return move_step_nums_; }

  std::vector<Points>& set_current_obstacle() { return current_obstacle_list_; }

 private:
  // map info
  int16_t row_, column_;                   // map的行、列数
  Points start_pos_, goal_pos_;            // map的起点与终点
  std::vector<Points> map_obstacle_list_;  //所有障碍物list
  CellInfo start, goal;

  //一次ARA*所需的容器
  Points current_start_;                       //当前起点
  std::vector<Points> current_path_;           //当前路径
  std::vector<CellInfo> open_list_;            //当前openlist
  std::vector<Points> current_obstacle_list_;  //当前已经获得的障碍物列表
  std::map<Points, Points> current_save_path_hash_;  //当前存储path的hash
  //当前存储已经expanded points的信息的hash
  std::map<Points, CellInfo>
      consistent_cell_info_list_;  //存储扩展过且一致的cell

  int32_t current_expand_points_count_,  //一次算法中的expand计数
      all_expand_points_count_,          //整体算法中的expand计数
      search_nums_count_,                //搜索次数计数
      move_step_nums_;                   //移动步数计数
  //判断点是否在list中
  inline bool IsInList(const Points& point,
                       const std::vector<Points>& list) const {
    return std::find(list.begin(), list.end(), point) != list.end();
  }
};

// 移除openlist中的指定点
template <class T>
void LPAstar::Remove(int16_t index, std::vector<T>* list) {
  if (list->size() >= 2) list->at(index) = list->back();
  list->pop_back();
}

void InceaseObstacle(LPAstar* object);
void DecreaseObstacle(LPAstar* object);

#endif