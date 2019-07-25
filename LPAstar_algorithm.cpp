#include "include/LPAstar_algorithm.h"
std::vector<std::string> sum_result;  //总结输出结果

/* 构造函数
 * 输入：地图的行数、列数、起点、终点、障碍物点
 * 输出：无
 * */
LPAstar::LPAstar(int16_t row_arg, int16_t column_arg, Points statr_arg,
                 Points goal_arg, std::vector<Points> obstacle_list_arg)
    : row_(row_arg),
      column_(column_arg),
      start_pos_(statr_arg),
      goal_pos_(goal_arg),
      map_obstacle_list_(obstacle_list_arg) {
  //初始化赋值
  current_start_ = start_pos_;  //赋值当前起点为最初的起点

  current_obstacle_list_ = map_obstacle_list_;

  search_nums_count_ = 0;            //搜索次数初始化
  all_expand_points_count_ = 0;      //扩展点个数计数初始化
  current_expand_points_count_ = 0;  //当前扩展点计数
  move_step_nums_ = 0;               //移动步数

  //初始化信息
  goal = {goal_pos_, 0.0f, INF_f::max(), INF_f::max()};
  start = {start_pos_, 0.0f, INF_f::max(), 0.0f};
  start.h_value = DistenceToGoal(start.xoy);

  open_list_.push_back(start);
}

/* 在一次A*算法中获得当前点的四个临近点的坐标
 * 输入：当前点的坐标
 * 输出：四个临近点的坐标
 * */
std::vector<Points> LPAstar::GetNeighborsPoint(const Points& current_pos) {
  std::vector<Points> neighbors;
  // UP
  if ((current_pos.first - 1) >= 0 &&
      !IsInList(Points(current_pos.first - 1, current_pos.second),
                current_obstacle_list_)) {
    neighbors.push_back(Points(current_pos.first - 1, current_pos.second));
  }
  // Down
  if ((current_pos.first + 1) < row_ &&
      !IsInList(Points(current_pos.first + 1, current_pos.second),
                current_obstacle_list_)) {
    neighbors.push_back(Points(current_pos.first + 1, current_pos.second));
  }
  // Left
  if ((current_pos.second - 1) >= 0 &&
      !IsInList(Points(current_pos.first, current_pos.second - 1),
                current_obstacle_list_)) {
    neighbors.push_back(Points(current_pos.first, current_pos.second - 1));
  }
  // Right
  if ((current_pos.second + 1) < column_ &&
      !IsInList(Points(current_pos.first, current_pos.second + 1),
                current_obstacle_list_)) {
    neighbors.push_back(Points(current_pos.first, current_pos.second + 1));
  }
  return neighbors;
}

/* 在一次A*算法中获得当前点的四个临近点的信息
 * 输入：当前点的坐标
 * 输出：四个临近点的信息
 * */
std::vector<CellInfo> LPAstar::GetNeighborsInfo(const Points& current_pos) {
  std::vector<CellInfo> neighbors;
  std::vector<Points> neighbors_pos = GetNeighborsPoint(current_pos);

  for (int8_t i = 0; i < neighbors_pos.size(); ++i) {
    //如果临近点已经扩展过，则赋值其之前的数据
    if (consistent_cell_info_list_.find(neighbors_pos[i]) !=
        consistent_cell_info_list_.end()) {
      neighbors.push_back(consistent_cell_info_list_[neighbors_pos[i]]);
    } else {  //若没被扩展过，赋为初始值
      neighbors.push_back(
          {neighbors_pos[i], INF_f::max(), INF_f::max(), INF_f::max()});
    }
  }
  return neighbors;
}

/* 更新点的信息,g、rhs、f等
 * 输入：需要更新的点的坐标
 * 输出：无
 *  */
void LPAstar::UpdateVertex(const Points& pos) {
  //如果扩展到了起点，立马退出函数
  if (pos == start_pos_) return;

  //去除openlist中与pos相同的点
  for (int16_t i = 0; i < open_list_.size(); ++i) {
    if (pos == open_list_[i].xoy) Remove<CellInfo>(i, &open_list_);
  }

  //寻找临近点中最小的g_value
  std::vector<CellInfo> neighbors = GetNeighborsInfo(pos);
  float min_g = INF_f::max();
  int16_t min_index = 0;
  int8_t flg_have_neighbors = 0;
  //找到最小的g，在neighbors中
  for (int8_t i = 0; i < neighbors.size(); ++i) {
    min_g = std::min(min_g, neighbors[i].g_value);
    if (min_g == neighbors[i].g_value) min_index = i;
    ++flg_have_neighbors;
  }

  //找到了终点
  if (pos == goal_pos_) {
    if (min_g != INF_f::max()) {
      goal.rhs_value = min_g + 1;
      std::cout << "find goal successful !!" << std::endl;
    } else {
      //寻找失败了
      goal.rhs_value = min_g;
    }
  }

  //如果该点已经被扩展过
  if (consistent_cell_info_list_.find(pos) !=
      consistent_cell_info_list_.end()) {
    //如果该店已经扩展过，其rhs却又变为了INF，说明该点周围发生了变化，应该把该点放入openlist中，让其重新去扩展
    if (min_g == INF_f::max()) {
      current_save_path_hash_.erase(pos);
      consistent_cell_info_list_[pos].rhs_value = min_g;
      // 非常关键的操作
      open_list_.push_back(consistent_cell_info_list_[pos]);
      consistent_cell_info_list_.erase(pos);
    } else {
      consistent_cell_info_list_[pos].rhs_value = min_g + 1;
      if (consistent_cell_info_list_[pos].rhs_value !=
          consistent_cell_info_list_[pos].g_value) {
        open_list_.push_back(consistent_cell_info_list_[pos]);
        consistent_cell_info_list_.erase(pos);
      }
      if (flg_have_neighbors)
        current_save_path_hash_[pos] = neighbors[min_index].xoy;
    }
  } else {
    //若点没有扩展过，将按照初始化的方式将其push进入openlist
    if (min_g != INF_f::max()) {
      open_list_.push_back(
          CellInfo({pos, DistenceToGoal(pos), INF_f::max(), min_g + 1}));
      if (flg_have_neighbors)
        current_save_path_hash_[pos] = neighbors[min_index].xoy;
    }
  }
}
/* 将openlist中的最小元素放到末尾
 * 输入：无
 * 输出：无
 * */
void LPAstar::OpenLIstPopMinElem() {
  if (open_list_.empty()) {
    open_list_.push_back(
        CellInfo({Points(-1, -1), INF_f::max(), INF_f::max(), INF_f::max()}));
    return;
  }
  if (open_list_.size() < 2) return;

  std::vector<CellInfo>::iterator itr =
      std::min_element(open_list_.begin(), open_list_.end());

  CellInfo temp = *itr;
  *itr = open_list_.back();
  open_list_.back() = temp;
}

bool LPAstar::LoopFlg() {
  OpenLIstPopMinElem();

  if (std::min(goal.g_value, goal.rhs_value) == open_list_.back().get_f_value())
    return std::min(goal.g_value, goal.rhs_value) >
           std::min(open_list_.back().g_value, open_list_.back().rhs_value);
  else
    return std::min(goal.g_value, goal.rhs_value) >
           open_list_.back().get_f_value();
}

/* 执行一次A*算法
 * 输入：当前起点与终点的信息
 * 输出：false：搜索失败; true：搜索成功
 * */
bool LPAstar::AstarAlgorithm() {
  std::vector<Points> path_result_list;  //存放本次搜索的路径

  int8_t search_successful_flg = 0;  //判断flg
  current_expand_points_count_ = 0;

  //*****搜索循环*****//
  while (LoopFlg() || goal.g_value != goal.rhs_value) {
    //*****弹出当前节点*****//
    CellInfo current_cell = open_list_.back();
    open_list_.pop_back();

    int8_t neighbor_expand_cnt = 0;

    if (current_cell.g_value > current_cell.rhs_value) {
      //使cell consistent
      current_cell.g_value = current_cell.rhs_value;
      if (current_cell.xoy == goal_pos_) goal.g_value = goal.rhs_value;

      //存储已经扩展过的cell
      consistent_cell_info_list_[current_cell.xoy] = current_cell;
      std::vector<Points> neighbors_pos = GetNeighborsPoint(current_cell.xoy);
      for (int8_t i = 0; i < neighbors_pos.size(); ++i) {
        ++neighbor_expand_cnt;
        UpdateVertex(neighbors_pos[i]);
      }
    } else {
      current_cell.g_value = INF_f::max();
      if (current_cell.xoy == goal_pos_) {
        goal.g_value = INF_f::max();
      }
      //如果重置了一个cell，那么将它从存储列表中去除
      consistent_cell_info_list_.erase(current_cell.xoy);

      UpdateVertex(current_cell.xoy);
      std::vector<Points> neighbors_pos = GetNeighborsPoint(current_cell.xoy);
      for (int8_t i = 0; i < neighbors_pos.size(); ++i) {
        ++neighbor_expand_cnt;
        UpdateVertex(neighbors_pos[i]);
      }
    }
    //扩展点自曾
    if (neighbor_expand_cnt) ++current_expand_points_count_;
  }
  //失败标志
  if (goal.rhs_value == INF_f::max()) search_successful_flg = 1;

  current_path_.clear();
  //*****搜索结果判断*****//
  if (search_successful_flg) {
    std::cout << "search fail !!" << std::endl;
    //打印结果
    PrintSearchResult();
    return false;
  }
  // there is one shortest path to goal
  else {
    std::cout << "search successfully !!" << std::endl;

    Points node = goal.xoy;

    //**路径回溯**//
    while (current_save_path_hash_.find(node) !=
           current_save_path_hash_.end()) {
      path_result_list.push_back(node);  // path中含有goal
      node = current_save_path_hash_[node];
    }

    current_path_ = path_result_list;

    //打印结果
    PrintSearchResult();

    //总扩展点计数
    all_expand_points_count_ += current_expand_points_count_;

    //扩展点计数清零
    current_expand_points_count_ = 0;

    return true;
  }
}

/* 打印一次搜索的结果
 * 输入：无
 * 输出：无
 * */
void LPAstar::PrintSearchResult() {
  for (int i = 0; i < row_; ++i) {
    for (int j = 0; j < column_; ++j) {
      if (start.xoy.first == i && start.xoy.second == j)
        std::cout << "s ";

      else if (goal_pos_.first == i && goal_pos_.second == j)
        std::cout << "g ";

      else if (IsInList(Points(i, j), current_obstacle_list_))
        std::cout << "x ";

      else if (IsInList(Points(i, j), current_path_))
        std::cout << "o ";
      else
        std::cout << "_ ";
    }
    std::cout << std::endl;
  }
  std::cout << "shortest path step nums : " << current_path_.size()
            << "    expand point nums : " << current_expand_points_count_
            << std::endl;

  std::cout << std::endl << std::endl;
}

/* 在整体ARA*算法中获得当前起点的四个临近点的信息
 * 输入：无
 * 输出：无
 * */
void LPAstar::UpdataMapInfo() {
  // UP
  if ((current_start_.first - 1) >= 0) {
    if (IsInList(Points(current_start_.first - 1, current_start_.second),
                 map_obstacle_list_))
      current_obstacle_list_.push_back(
          Points(current_start_.first - 1, current_start_.second));
  }
  // Down
  if ((current_start_.first + 1) < row_) {
    if (IsInList(Points(current_start_.first + 1, current_start_.second),
                 map_obstacle_list_))
      current_obstacle_list_.push_back(
          Points(current_start_.first + 1, current_start_.second));
  }
  // Left
  if ((current_start_.second - 1) >= 0) {
    if (IsInList(Points(current_start_.first, current_start_.second - 1),
                 map_obstacle_list_))
      current_obstacle_list_.push_back(
          Points(current_start_.first, current_start_.second - 1));
  }
  // Right
  if ((current_start_.second + 1) < column_) {
    if (IsInList(Points(current_start_.first, current_start_.second + 1),
                 map_obstacle_list_))
      current_obstacle_list_.push_back(
          Points(current_start_.first, current_start_.second + 1));
  }
}

/* 打印计数结果
 * 输入：无
 * 输出：无
 *  */
void LPAstar::PrintCountResult() {
  std::cout << std::endl
            << "The nums of search : " << search_nums_count_
            << "  total expanded nums : " << all_expand_points_count_
            << std::endl
            << std::endl;
}

/* 进行一次总体的ARA*算法
 * 输入：文件序号
 * 输出：无
 *  */
void SearchOneMap(int map_num_) {
  //**获得map信息**//
  GrideInput map_info(map_num_);
  map_info.GetOneGrid();
  map_info.PrintMap();  //打印原始map

  //数据传入，构造ARA*算法对象
  LPAstar LPAstar_algorithm(map_info.get_grid_rows(),
                            map_info.get_grid_columns(),
                            map_info.get_start_pos(), map_info.get_goal_pos(),
                            map_info.get_obstacle_pos());

  bool flg = LPAstar_algorithm.AstarAlgorithm();
  while (1) {
    std::cout << "*****  q-->quit  i-->increalse  d-->decrease  *****"
              << std::endl;
    std::cout << "Please input your choose : ";
    char choose_flg = '\0';
    std::cin >> choose_flg;
    switch (choose_flg) {
      case 'i':
        InceaseObstacle(&LPAstar_algorithm);
        break;
      case 'd':
        DecreaseObstacle(&LPAstar_algorithm);
        break;
      case 'q':
        break;

      default:
        std::cout << "please input correct choose !!" << std::endl;
        choose_flg = 0;
        break;
    }
    if (choose_flg == 'q') break;
    if (choose_flg == 'i' || choose_flg == 'd') {
      LPAstar_algorithm.AstarAlgorithm();  //以当前起点为起点进行一次路径规划
    }
  }
}

/* 打印统计结果
 * 输入：无
 * 输出：无
 *  */
void PrintSumResult() {
  std::cout << std::endl
            << "-——-——-——-——-——-——-——-——-***-——-——-——-——-——-——-——-——-——-"
            << std::endl
            << "-——                   Sum  Result                    ——-"
            << std::endl
            << "-——-——-——-——-——-——-——-——-***-——-——-——-——-——-——-——-——-——-"
            << std::endl;
  std::cout << "| map num | search nums | expand nums | move_step nums |"
            << std::endl;
  for (int16_t i = 0; i < sum_result.size(); ++i) {
    std::cout << sum_result[i] << std::endl;
  }
}

/* 人机交互函数
 * 输入：当前LPA*算法对象
 * 输出：在LPA*对象中对应的障碍物列表中的添加了pos
 *  */
void InceaseObstacle(LPAstar* object) {
  int16_t row = 0, col = 0;
  std::cout << "Please input your wang to add pos: ";
  std::cin >> row >> col;
  while (row < 0 || row >= object->get_row() || col < 0 ||
         col >= object->get_column() ||
         Points(row, col) == object->get_start_pos() ||
         Points(row, col) == object->get_goal_pos()) {
    std::cout << "The pos can`t pass !!" << std::endl << "Please input agin: ";
    std::cin >> row >> col;
  }
  object->ChangeOfInceaseObstacle(Points(row, col));
}

/* 人机交互函数
 * 输入：当前LPA*算法对象
 * 输出：在LPA*对象中对应的障碍物列表中的去除了pos
 *  */
void DecreaseObstacle(LPAstar* object) {
  std::cout << "current obstacle list: ";
  for (int16_t i = 0; i < object->get_current_obstacle_list().size(); ++i) {
    std::cout << "(" << object->get_current_obstacle_list()[i].first << ","
              << object->get_current_obstacle_list()[i].second << ") ";
  }
  int16_t row = 0, col = 0;
  std::cout << std::endl << "Please input pos from above list: ";
  std::cin >> row >> col;
  while (std::find(object->get_current_obstacle_list().begin(),
                   object->get_current_obstacle_list().end(),
                   Points(row, col)) ==
         object->get_current_obstacle_list().end()) {
    std::cout << "The pos can`t pass !!" << std::endl << "Please input agin: ";
    std::cin >> row >> col;
  }
  object->ChangeOfDecreaseObstacle(Points(row, col));
}

/* 增加障碍物，并更新障碍物
 * 输入：需要去除障碍物的坐标
 * 输出：更新障碍物所有临近点的信息
 *  */
void LPAstar::ChangeOfInceaseObstacle(const Points& pos) {
  consistent_cell_info_list_.erase(pos);

  for (int16_t i = 0; i < open_list_.size(); ++i) {
    if (pos == open_list_[i].xoy) Remove<CellInfo>(i, &open_list_);
  }
  current_obstacle_list_.push_back(pos);
  std::vector<Points> neighbors =
      GetNeighborsPoint(current_obstacle_list_.back());
  for (int8_t i = 0; i < neighbors.size(); ++i) {
    UpdateVertex(neighbors[i]);
  }
}

/* 去除障碍物，并更新障碍物
 * 输入：需要去除障碍物的坐标
 * 输出：更新去除点以及其所有临近点的信息
 *  */
void LPAstar::ChangeOfDecreaseObstacle(const Points& pos) {
  for (int16_t i = 0; i < current_obstacle_list_.size(); ++i) {
    if (pos == current_obstacle_list_[i])
      Remove<Points>(i, &current_obstacle_list_);
  }
  UpdateVertex(pos);
  std::vector<Points> neighbors = GetNeighborsPoint(pos);
  for (int8_t i = 0; i < neighbors.size(); ++i) {
    UpdateVertex(neighbors[i]);
  }
}