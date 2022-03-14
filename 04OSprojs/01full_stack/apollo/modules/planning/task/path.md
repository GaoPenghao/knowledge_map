# 概览

规划模块的运行总体流程图如下：

<img src="D:\project\Git\dig-into-apollo\modules\planning\img\planning_flow01.png" alt="planning_flow01"  />

总体流程图以lane follow场景为例子进行说明。这里只说明主体的流程，不涉及到所有的细节。task的主要功能位于`Process`函数中。

第一，规划模块的入口函数是PlanningComponent的Proc。

第二，以规划模式OnLanePlanning为例，执行RunOnce。在RunOnce中先执行交通规则，再规划轨迹。规划轨迹的函数是Plan。

第三，进入到PublicRoadPlanner中的Plan函数，进行轨迹规划。ScenarioManager的Update函数根据当前的scenario_type选择合适的场景。这里的流程图是以lane follow为例。

第四，选择lane follow的场景后，执行Process函数。然后，执行LaneFollowStage中的Process函数，在PlanOnReferenceLine中执行LaneFollowStage中的所有的task。通过调用Excute函数执行task，Excute调用了task的Process（以decider为例子）函数。最后一个图中，TaskType指的不是具体的类名称，代表所有的task类型。虚线的箭头，表示在LaneFollowStage中按照vector中的顺序执行所有的任务。

最后，Task的流程都在Process函数中。之后对task的解析都从Process函数开始。

注：lane follow default stage中的task列表如下所示，在执行stage的时候按照顺序执行如下的taskxx：

```
stage_type: LANE_FOLLOW_DEFAULT_STAGE
  enabled: true
  task_type: LANE_CHANGE_DECIDER
  task_type: PATH_REUSE_DECIDER
  task_type: PATH_LANE_BORROW_DECIDER
  task_type: PATH_BOUNDS_DECIDER
  task_type: PIECEWISE_JERK_PATH_OPTIMIZER
  task_type: PATH_ASSESSMENT_DECIDER
  task_type: PATH_DECIDER
  task_type: RULE_BASED_STOP_DECIDER
  task_type: ST_BOUNDS_DECIDER
  task_type: SPEED_BOUNDS_PRIORI_DECIDER
  task_type: SPEED_HEURISTIC_OPTIMIZER
  task_type: SPEED_DECIDER
  task_type: SPEED_BOUNDS_FINAL_DECIDER
  # task_type: PIECEWISE_JERK_SPEED_OPTIMIZER
  task_type: PIECEWISE_JERK_NONLINEAR_SPEED_OPTIMIZER
  task_type: RSS_DECIDER
```

# 变道决策

其类关系和数据结构参考`路径边界决策`

## 代码流程及框架

```c++
// added a dummy parameter to enable this task in ExecuteTaskOnReferenceLine
Status LaneChangeDecider::Process(
    Frame* frame, ReferenceLineInfo* const current_reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);

  const auto& lane_change_decider_config = config_.lane_change_decider_config();
  // 根据当前信息获取 reference_line_info，并判断其是否为空
  std::list<ReferenceLineInfo>* reference_line_info =
      frame->mutable_reference_line_info();
  if (reference_line_info->empty()) {
    const std::string msg = "Reference lines empty.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // 查看配置信息，reckless_change_lane是否为true
  if (lane_change_decider_config.reckless_change_lane()) {
    PrioritizeChangeLane(true, reference_line_info);
    return Status::OK();
  }
  // 获取最近的ChangeLaneStatus信息
  auto* prev_status = injector_->planning_context()
                          ->mutable_planning_status()
                          ->mutable_change_lane();
  double now = Clock::NowInSeconds();
  // 将ChangeLaneStatus中的is_clear_to_change_lane设置为false（默认值，即不满足变道条件）
  // 如果满足变道条件，则将其设置为true
  prev_status->set_is_clear_to_change_lane(false);
  if (current_reference_line_info->IsChangeLanePath()) {
    prev_status->set_is_clear_to_change_lane(
        IsClearToChangeLane(current_reference_line_info));
  }
  // 如果ChangeLaneStatus消息中的Status为空，则变更为CHANGE_LANE_FINISHED，并返回
  if (!prev_status->has_status()) {
    UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FINISHED,
                 GetCurrentPathId(*reference_line_info));
    prev_status->set_last_succeed_timestamp(now);
    return Status::OK();
  }
  // 正常的变道决策逻辑
  bool has_change_lane = reference_line_info->size() > 1;
  ADEBUG << "has_change_lane: " << has_change_lane;
  if (!has_change_lane) {  // 1 不存在可变车道
    const auto& path_id = reference_line_info->front().Lanes().Id();
    if (prev_status->status() == ChangeLaneStatus::CHANGE_LANE_FINISHED) {  // 1.1 如果最近的Status为CHANGE_LANE_FINISHED，则不做处理
    } else if (prev_status->status() == ChangeLaneStatus::IN_CHANGE_LANE) {  // 1.2 如果最近的Status为IN_CHANGE_LANE，则将其更新为CHANGE_LANE_FINISHED
      UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FINISHED, path_id);
    } else if (prev_status->status() == ChangeLaneStatus::CHANGE_LANE_FAILED) {  //1.3 如果最近的Status为CHANGE_LANE_FAILED，则同样不做处理
    } else {  // 1.4 除此以外，则视为未知状态，记录并返回规划失败
      const std::string msg =
          absl::StrCat("Unknown state: ", prev_status->ShortDebugString());
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    return Status::OK();
  } else {  // 2 存在可变车道
    auto current_path_id = GetCurrentPathId(*reference_line_info);
    if (current_path_id.empty()) {  // 如果当前车道的id为空，则记录并返回规划失败
      const std::string msg = "The vehicle is not on any reference line";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    if (prev_status->status() == ChangeLaneStatus::IN_CHANGE_LANE) {  // 2.1 如果最近的Status为IN_CHANGE_LANE
      if (prev_status->path_id() == current_path_id) {  // 2.1.1 如果最近的path_id == 当前的path_id，说明还未成功变道，则设置优先变道
        PrioritizeChangeLane(true, reference_line_info);
      } else {  // 2.1.2 否则说明变道成功，设置不优先变道，更新状态
        // RemoveChangeLane(reference_line_info);
        PrioritizeChangeLane(false, reference_line_info);
        ADEBUG << "removed change lane.";
        UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FINISHED,
                     current_path_id);
      }
      return Status::OK();
    } else if (prev_status->status() == ChangeLaneStatus::CHANGE_LANE_FAILED) {  // 2.2 如果最近的Status为CHANGE_LANE_FAILED，表明有变道需求
      // TODO(SHU): add an optimization_failure counter to enter
      // change_lane_failed status
      if (now - prev_status->timestamp() <
          lane_change_decider_config.change_lane_fail_freeze_time()) {  // 2.2.1 如果此时距离最近的Status的时间间隔（以下简称时间间隔）小于冻结间隔（防止变道失败时，频繁尝试），设置不优先变道
        // RemoveChangeLane(reference_line_info);
        PrioritizeChangeLane(false, reference_line_info);
        ADEBUG << "freezed after failed";
      } else {  // 2.2.2 否则说明已经渡过冻结期，则更新状态为变道中
        UpdateStatus(now, ChangeLaneStatus::IN_CHANGE_LANE, current_path_id);
        ADEBUG << "change lane again after failed";
      }
      return Status::OK();
    } else if (prev_status->status() ==
               ChangeLaneStatus::CHANGE_LANE_FINISHED) {  // 2.3 如果最近的Status为CHANGE_LANE_FINISHED
      if (now - prev_status->timestamp() <
          lane_change_decider_config.change_lane_success_freeze_time()) {  // 2.3.1 如果此时时间间隔小于冻结期，则设置不优先变道
        // RemoveChangeLane(reference_line_info);
        PrioritizeChangeLane(false, reference_line_info);
        ADEBUG << "freezed after completed lane change";
      } else {  // 2.3.2 否则说明已经渡过冻结期，则设置优先变道，并更新状态为变道中
        PrioritizeChangeLane(true, reference_line_info);
        UpdateStatus(now, ChangeLaneStatus::IN_CHANGE_LANE, current_path_id);
        ADEBUG << "change lane again after success";
      }
    } else {  // 2.4 除上述以外，视为最近的Status未知，记录并返回规划错误
      const std::string msg =
          absl::StrCat("Unknown state: ", prev_status->ShortDebugString());
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }
  return Status::OK();
}
```

# 重复使用路径决策

TODO：

- lane_follow_config.pb.txt中包含了path_reuse_decider_config，其中声明了`reuse_path: false`；另外planning_gflags.cc中声明了`enable_reuse_path_in_lane_follow, false`；还有在planning_config.pb.txt中包含了path_reuse_decider_config，其中同样声明了`reuse_path: false`；但是在planning.config中声明了`--enable_reuse_path_in_lane_follow=true`。
- 针对以上问题，目前的猜测是，planning_gflags中声明了众多的全局变量，并赋予了缺省值，但是可以在planning.config中进行赋值，然后重启planning模块，从而改变配置，所以planning.config优先级更高。
- `if (lane_change_status->status() != ChangeLaneStatus::IN_CHANGE_LANE && !FLAGS_enable_reuse_path_in_lane_follow) {...}`这个判断条件的后半部分设计的有问题
- `static constexpr int kWaitCycle = -2`这个参数的赋值为什么是-2？
- `front_static_obstacle_cycle_counter`的物理意义？

path reuse的条件：

1. 处于lane_follow场景
2. 只有当lane_change_status（变道状态）为IN_CHANGE_LANE（在变道中），才允许path reuse
3. 对于hybrid model，如果存在valid_path_reference（有效的参考路径），则跳过path reuse
4. 不在变道的参考线上或者变道成功，则允许path reuse  TODO：如何理解`reference_line_info->IsChangeLanePath()`
5. 当前帧的规划轨迹不是重规划  TODO：如何理解current_frame_planned_trajectory，如何理解replan，难道是考虑迭代？
6. 没有碰撞风险
7. 成功修剪历史路径
8. 上一次的路径速度优化状态为成功

# 借道决策

借道相较于变道最本质的区别是目标参考线的不同，变道的目标参考线是将要变入的目标车道，而借道的目标参考线仍是本车道，但是由于本车道的通行空间不充足，所以借用相邻车道的横向空间。

其主流程如下：

1. 如果重复使用path，则return
2. 设置默认值，不借道
3. 根据当前信息检查，如果允许借道并且需要借道，则设置借道

```c++
Status PathLaneBorrowDecider::Process(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // skip path_lane_borrow_decider if reused path
  if (FLAGS_enable_skip_path_tasks && reference_line_info->path_reusable()) {
    // for debug
    AINFO << "skip due to reusing path";
    return Status::OK();
  }

  // By default, don't borrow any lane.
  reference_line_info->set_is_path_lane_borrow(false);
  // Check if lane-borrowing is needed, if so, borrow lane.
  if (Decider::config_.path_lane_borrow_decider_config()
          .allow_lane_borrowing() &&
      IsNecessaryToBorrowLane(*frame, *reference_line_info)) {
    reference_line_info->set_is_path_lane_borrow(true);
  }
  return Status::OK();
}
```

可见其中主要的实现过程为`IsNecessaryToBorrowLane(*frame, *reference_line_info)`，下面主要分析这个函数，其主要流程如下：

```c++
bool PathLaneBorrowDecider::IsNecessaryToBorrowLane(
    const Frame& frame, const ReferenceLineInfo& reference_line_info) {
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  // 1 判断当前是否已经在借道
  if (mutable_path_decider_status->is_in_path_lane_borrow_scenario()) {
    // 1.1 如果已经在借道，进而判断已经能够使用自车道的次数:
    if (mutable_path_decider_status->able_to_use_self_lane_counter() >= 6) {
      // 如果次数≥6，则切换至没有在借道，
      mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
      mutable_path_decider_status->clear_decided_side_pass_direction();
      AINFO << "Switch from LANE-BORROW path to SELF-LANE path.";
    }
  } else {
    // If originally not borrowing neighbor lane:
    ADEBUG << "Blocking obstacle ID["
           << mutable_path_decider_status->front_static_obstacle_id() << "]";
    // ADC requirements check for lane-borrowing:
    if (!HasSingleReferenceLine(frame)) {
      return false;
    }
    if (!IsWithinSidePassingSpeedADC(frame)) {
      return false;
    }

    // Obstacle condition check for lane-borrowing:
    if (!IsBlockingObstacleFarFromIntersection(reference_line_info)) {
      return false;
    }
    if (!IsLongTermBlockingObstacle()) {
      return false;
    }
    if (!IsBlockingObstacleWithinDestination(reference_line_info)) {
      return false;
    }
    if (!IsSidePassableObstacle(reference_line_info)) {
      return false;
    }

    // switch to lane-borrowing
    // set side-pass direction
    const auto& path_decider_status =
        injector_->planning_context()->planning_status().path_decider();
    if (path_decider_status.decided_side_pass_direction().empty()) {
      // first time init decided_side_pass_direction
      bool left_borrowable;
      bool right_borrowable;
      CheckLaneBorrow(reference_line_info, &left_borrowable, &right_borrowable);
      if (!left_borrowable && !right_borrowable) {
        mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
        return false;
      } else {
        mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(true);
        if (left_borrowable) {
          mutable_path_decider_status->add_decided_side_pass_direction(
              PathDeciderStatus::LEFT_BORROW);
        }
        if (right_borrowable) {
          mutable_path_decider_status->add_decided_side_pass_direction(
              PathDeciderStatus::RIGHT_BORROW);
        }
      }
    }

    AINFO << "Switch from SELF-LANE path to LANE-BORROW path.";
  }
  return mutable_path_decider_status->is_in_path_lane_borrow_scenario();
}
```



# 路径边界决策

## 类关系

<img src="D:\project\Git\dig-into-apollo\modules\planning\img\decider_path_bounds.png" alt="decider_path_bounds" style="zoom:67%;" />

（1）继承关系

① `PathBoundsDecider`类继承`Decider`类，实现了`Process`方法，路径边界决策主要的执行过程就在`process`方法中。

```c++
// modules/planning/tasks/deciders/path_bounds_decider/path_bounds_decider.h
class PathBoundsDecider : public Decider {
... };
```

② `Decider`类继承`Task`类，实现Excute方法，主要是给两个变量赋值：`frame`和`reference_line_info`，并且执行Process方法。对应了上述的Process方法

```c++
// modules/planning/tasks/deciders/decider.h
class Decider : public Task {
... };

// modules/planning/tasks/deciders/decider.cc
apollo::common::Status Decider::Execute(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  Task::Execute(frame, reference_line_info);
  return Process(frame, reference_line_info);
}

apollo::common::Status Decider::Execute(Frame* frame) {
  Task::Execute(frame);
  return Process(frame);
}
```

③ `Task`类，定义类保护类型的变量，是路径边界决策的输入

```c++
// modules/planning/tasks/task.h
class Task {
 public:
  // 虚方法，主要是给frame和reference_line_info赋值
  virtual common::Status Execute(Frame* frame,
                                 ReferenceLineInfo* reference_line_info);
  virtual common::Status Execute(Frame* frame);

 protected:
  // frame和reference_line_info变量
  Frame* frame_ = nullptr;
  ReferenceLineInfo* reference_line_info_ = nullptr;

  // 配置与名字
  TaskConfig config_;
  std::string name_;
... };
```

（2）调用

主要描述task在stage中是如何创建和调用的

① `TaskFactory`类，注册所有的task，包括decider、optimizer和other（E2E的task）。工厂模式

```c++
// modules/planning/tasks/task_factory.h
class TaskFactory {
 public:
  // 两个函数都是static属性
  static void Init(...);    // 在初始化函数中，注册所有的task
  static std::unique_ptr<Task> CreateTask(...);    // 创建具体task的实例，返回指向该实例的指针
... };
```

② stage中task的创建与执行

- 创建：在stage的构造函数中根据stage配置创建task。并将指针放入到tasks\_和task\_list\_中
- 使用：在具体的stage中，重写Process方法。调用Process方法，进而调用ExecuteTask*方法（ExecuteTaskOnReferenceLine），最后调用相应的task的Process方法

```c++
// modules/planning/scenarios/stage.h
class Stage {
 // 在构造函数中根据stage的配置创建task
 Stage(const ScenarioConfig::StageConfig& config,
        const std::shared_ptr<DependencyInjector>& injector);
 public:

 // 纯虚函数，留给具体的stage实现，不同的stage有不同的实现逻辑
 virtual StageStatus Process(
      const common::TrajectoryPoint& planning_init_point, Frame* frame) = 0;
 protected:
  // 三个执行task的函数，在每个函数中都调用类task的Excute方法，进一步调用具体task的Process方法
  bool ExecuteTaskOnReferenceLine(
      const common::TrajectoryPoint& planning_start_point, Frame* frame);

  bool ExecuteTaskOnReferenceLineForOnlineLearning(
      const common::TrajectoryPoint& planning_start_point, Frame* frame);

  bool ExecuteTaskOnOpenSpace(Frame* frame);

 protected:
  // task的map，key是TaskType，value是指向Task的指针
  std::map<TaskConfig::TaskType, std::unique_ptr<Task>> tasks_;
  // 保存Task列表
  std::vector<Task*> task_list_;
  // stage 配置
  ScenarioConfig::StageConfig config_;
...};
```

## 数据结构

`PathBoundsDecider`类主要的输入、输出，数据结构，变量设置。

（1）输入和输出

① 输入有两个：`frame`与`reference_line_info`

- frame，frame中包含了一次规划所需要的所有的数据

```c++
// modules/planning/common/frame.h
class Frame {
 private:
  static DrivingAction pad_msg_driving_action_;
  uint32_t sequence_num_ = 0;

  /* Local_view是一个结构体，包含了如下信息
  // modules/planning/common/local_view.h
  struct LocalView {
    std::shared_ptr<prediction::PredictionObstacles> prediction_obstacles;
    std::shared_ptr<canbus::Chassis> chassis;
    std::shared_ptr<localization::LocalizationEstimate> localization_estimate;
    std::shared_ptr<perception::TrafficLightDetection> traffic_light;
    std::shared_ptr<routing::RoutingResponse> routing;
    std::shared_ptr<relative_map::MapMsg> relative_map;
    std::shared_ptr<PadMessage> pad_msg;
    std::shared_ptr<storytelling::Stories> stories;
  };
  */
  LocalView local_view_;
  // 高清地图
  const hdmap::HDMap *hdmap_ = nullptr;
  common::TrajectoryPoint planning_start_point_;
  // 车辆状态
  // modules/common/vehicle_state/proto/vehicle_state.proto
  common::VehicleState vehicle_state_;
  // 参考线信息
  std::list<ReferenceLineInfo> reference_line_info_;

  bool is_near_destination_ = false;

  /**
   * the reference line info that the vehicle finally choose to drive on
   **/
  const ReferenceLineInfo *drive_reference_line_info_ = nullptr;

  ThreadSafeIndexedObstacles obstacles_;

  std::unordered_map<std::string, const perception::TrafficLight *>
      traffic_lights_;

  // current frame published trajectory
  ADCTrajectory current_frame_planned_trajectory_;

  // current frame path for future possible speed fallback
  DiscretizedPath current_frame_planned_path_;

  const ReferenceLineProvider *reference_line_provider_ = nullptr;

  OpenSpaceInfo open_space_info_;

  std::vector<routing::LaneWaypoint> future_route_waypoints_;

  common::monitor::MonitorLogBuffer monitor_logger_buffer_;
};
```

- reference_line_info，reference_line_info包含了有关reference_line的所有的数据

```c++
// modules/planning/common/reference_line_info.h

class ReferenceLineInfo {
 ...
 private:
  static std::unordered_map<std::string, bool> junction_right_of_way_map_;
  const common::VehicleState vehicle_state_;  // 车辆状态
  const common::TrajectoryPoint adc_planning_point_;  // TrajectoryPoint定义在modules/common/proto/pnc_point.proto中

  /* 参考线，以道路中心线，做过顺滑的一条轨迹，往后80米，往前130米。
  class ReferenceLine {
  ...
  private:
  struct SpeedLimit {
    double start_s = 0.0;
    double end_s = 0.0;
    double speed_limit = 0.0;  // unit m/s
    ...};
  
  // This speed limit overrides the lane speed limit
  std::vector<SpeedLimit> speed_limit_;
  std::vector<ReferencePoint> reference_points_;  // ReferencePoint包含有信息(k, dk, x, y, heading, s, l)
  hdmap::Path map_path_;
  uint32_t priority_ = 0;
  };
  */
  ReferenceLine reference_line_;

  /**
   * @brief this is the number that measures the goodness of this reference
   * line. The lower the better.
   */
  // 评价函数，值越低越好
  double cost_ = 0.0;

  bool is_drivable_ = true;
  // PathDecision包含了一条路径上的所有obstacle的决策，有两种：lateral(Nudge, Ignore)和longitudinal(Stop, Yield, Follow, Overtake, Ignore)
  PathDecision path_decision_;
  // 指针
  Obstacle* blocking_obstacle_;

  /* path的边界，结果保存在这个变量里。通过**SetCandidatePathBoundaries**方法保存到此变量
    // modules/planning/common/path_boundary.h
    class PathBoundary {
    ...
      private:
     double start_s_ = 0.0;
     double delta_s_ = 0.0;
     std::vector<std::pair<double, double>> boundary_;
     std::string label_ = "regular";
     std::string blocking_obstacle_id_ = "";
  };
  */
  std::vector<PathBoundary> candidate_path_boundaries_;
  // PathData类，包含XY坐标系和SL坐标系的相互转化
  std::vector<PathData> candidate_path_data_;

  PathData path_data_;
  PathData fallback_path_data_;
  SpeedData speed_data_;

  DiscretizedTrajectory discretized_trajectory_;

  RSSInfo rss_info_;

  /**
   * @brief SL boundary of stitching point (starting point of plan trajectory)
   * relative to the reference line
   */
  SLBoundary adc_sl_boundary_;
... };
```

② 输出：

```c++
Status PathBoundsDecider::Process(
    Frame* const frame, ReferenceLineInfo* const reference_line_info)
```

Process函数定义，最终结果保存到了`reference_line_info`中

（2）参数设置

```c++
// modules/planning/tasks/deciders/path_bounds_decider/path_bounds_decider.cc
// s方向的距离
constexpr double kPathBoundsDeciderHorizon = 100.0;
// s方向的间隔
constexpr double kPathBoundsDeciderResolution = 0.5;

// Lane宽度
constexpr double kDefaultLaneWidth = 5.0;
// Road的道路
constexpr double kDefaultRoadWidth = 20.0;

// TODO(all): Update extra tail point base on vehicle speed.
constexpr int kNumExtraTailBoundPoint = 20;
constexpr double kPulloverLonSearchCoeff = 1.5;
constexpr double kPulloverLatSearchCoeff = 1.25;
```

（3）数据结构

```c++
// modules/planning/tasks/deciders/path_bounds_decider/path_bounds_decider.cc
namespace {
// PathBoundPoint contains: (s, l_min, l_max). 路径边界点
using PathBoundPoint = std::tuple<double, double, double>;
// PathBound contains a vector of PathBoundPoints. 路径边界
using PathBound = std::vector<PathBoundPoint>;
// ObstacleEdge contains: (is_start_s, s, l_min, l_max, obstacle_id). 障碍物的边
using ObstacleEdge = std::tuple<int, double, double, double, std::string>;
}  // namespace
```

## 代码流程及框架

![decider_path_bounds01](D:\project\Git\dig-into-apollo\modules\planning\img\decider_path_bounds01.png)

上图是路径边界决策的流程图。在Process方法中，分四种场景对路径边界进行计算，按照处理的顺序分别是：fallback，pull-over（靠边停车），lane-change，regular。其中regular场景根据是否借道又分为LEFT_BORROW，NO_BORROW，RIGHT_BORROW。

fallback场景的path bounds一定会生成，另外三种看情况，都是需要if判断。

> fallback 的一种解释为 an alternative plan that may be used in an emergency.

## 算法解析

1. fallback

<img src="D:\project\Git\dig-into-apollo\modules\planning\img\decider_path_bounds02.png" alt="decider_path_bounds02" style="zoom:67%;" />

fallback场景生成过程如上图所示。fallback只考虑adc信息和静态道路信息，主要调用两个函数

- InitPathBoundary

```c++
bool PathBoundsDecider::InitPathBoundary(
  ...
  // Starting from ADC's current position, increment until the horizon, and
  // set lateral bounds to be infinite at every spot.
  // 从adc当前位置开始，以0.5m为间隔取点，直到终点，将 [左, 右] 边界设置为double的 [lowerst, max]
  for (double curr_s = adc_frenet_s_;
       curr_s < std::fmin(adc_frenet_s_ +
                              std::fmax(kPathBoundsDeciderHorizon,
                                        reference_line_info.GetCruiseSpeed() *
                                            FLAGS_trajectory_time_length),
                          reference_line.Length());
       curr_s += kPathBoundsDeciderResolution) {
    path_bound->emplace_back(curr_s, std::numeric_limits<double>::lowest(),
                             std::numeric_limits<double>::max());
  }
...}
```

- GetBoundaryFromLanesAndADC

```c++
// TODO(jiacheng): this function is to be retired soon.
bool PathBoundsDecider::GetBoundaryFromLanesAndADC(
  ...
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_s = std::get<0>((*path_bound)[i]);
    // 1. Get the current lane width at current point.获取当前点车道的宽度
    if (!reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                     &curr_lane_right_width)) {
      AWARN << "Failed to get lane width at s = " << curr_s;
      curr_lane_left_width = past_lane_left_width;
      curr_lane_right_width = past_lane_right_width;
    } else {...}

    // 2. Get the neighbor lane widths at the current point.获取当前点相邻车道的宽度
    double curr_neighbor_lane_width = 0.0;
    if (CheckLaneBoundaryType(reference_line_info, curr_s, lane_borrow_info)) {
      hdmap::Id neighbor_lane_id;
      if (lane_borrow_info == LaneBorrowInfo::LEFT_BORROW) {
        // 借左车道
        ...
      } else if (lane_borrow_info == LaneBorrowInfo::RIGHT_BORROW) {
        // 借右车道
        ...
      }
    }

    // 3. 根据道路宽度，adc的位置和速度计算合适的边界。
    static constexpr double kMaxLateralAccelerations = 1.5;
    double offset_to_map = 0.0;
    reference_line.GetOffsetToMap(curr_s, &offset_to_map);

    double ADC_speed_buffer = (adc_frenet_ld_ > 0 ? 1.0 : -1.0) *
                              adc_frenet_ld_ * adc_frenet_ld_ /
                              kMaxLateralAccelerations / 2.0;
    // 向左车道借到，左边界会变成左侧车道左边界
    double curr_left_bound_lane =
        curr_lane_left_width + (lane_borrow_info == LaneBorrowInfo::LEFT_BORROW
                                    ? curr_neighbor_lane_width
                                    : 0.0);
    // 和上面类似
    double curr_right_bound_lane =
        -curr_lane_right_width -
        (lane_borrow_info == LaneBorrowInfo::RIGHT_BORROW
             ? curr_neighbor_lane_width
             : 0.0);

    double curr_left_bound = 0.0;  // 左边界
    double curr_right_bound = 0.0;  // 右边界
    // 计算左边界和右边界（应该是左负右正，所以左边界取最大值，右边界取最小值）
    if (config_.path_bounds_decider_config()
            .is_extend_lane_bounds_to_include_adc() ||
        is_fallback_lanechange) {
      // extend path bounds to include ADC in fallback or change lane path
      // bounds.
      double curr_left_bound_adc =
          std::fmax(adc_l_to_lane_center_,
                    adc_l_to_lane_center_ + ADC_speed_buffer) +
          GetBufferBetweenADCCenterAndEdge() + ADC_buffer;
      curr_left_bound =
          std::fmax(curr_left_bound_lane, curr_left_bound_adc) - offset_to_map;

      double curr_right_bound_adc =
          std::fmin(adc_l_to_lane_center_,
                    adc_l_to_lane_center_ + ADC_speed_buffer) -
          GetBufferBetweenADCCenterAndEdge() - ADC_buffer;
      curr_right_bound =
          std::fmin(curr_right_bound_lane, curr_right_bound_adc) -
          offset_to_map;
    } else {
      curr_left_bound = curr_left_bound_lane - offset_to_map;
      curr_right_bound = curr_right_bound_lane - offset_to_map;
    }

    // 4. 更新边界.
    if (!UpdatePathBoundaryWithBuffer(i, curr_left_bound, curr_right_bound,
                                      path_bound, is_left_lane_boundary,
                                      is_right_lane_boundary)) {
      path_blocked_idx = static_cast<int>(i);
    }
... }
```

2. pull over

<img src="D:\project\Git\dig-into-apollo\modules\planning\img\decider_path_bounds03.png" alt="decider_path_bounds03" style="zoom: 67%;" />

（1）GetBoundaryFromRoads

与`GetBoundaryFromLanesAndADC`不同，`GetBoundaryFromRoads`函数根据道路信息计算出边界：

- 获取参考线信息

- 对路径上的点，逐点计算

  - 边界

  - 更新

（2）GetBoundaryFromStaticObstacles

根据障碍车调整边界：

- 计算障碍车在frenet坐标系下的坐标
- 扫描线排序，S方向扫描
  - 只关注在路径边界内的障碍物
  - 只关注在adc前方的障碍物
  - 将障碍物分解为两个边界，开始和结束
- 映射障碍物ID
  - Adc能从左边通过为True，否则为False
- 逐个点的检查path路径上的障碍物
  - 根据新来的障碍物
  - 根据已有的障碍物

（3）SearchPullOverPosition

搜索pull over位置的过程：

- 根据pull_over_status.pull_over_type()判断是前向搜索（pull over开头第一个点），还是后向搜索（pull over末尾后一个点）
- 两层循环，外层控制搜索的索引idx，内层控制进一步的索引（前向idx+1，后向idx-1）。
- 根据内外两层循环的索引，判断搜索到的空间是否满足宽度和长度要求，判断是否可以pull over

代码如下：

```c++
bool PathBoundsDecider::SearchPullOverPosition(
    const Frame& frame, const ReferenceLineInfo& reference_line_info,
    const std::vector<std::tuple<double, double, double>>& path_bound,
    std::tuple<double, double, double, int>* const pull_over_configuration) {
  const auto& pull_over_status =
      injector_->planning_context()->planning_status().pull_over();

  // 搜索方向，默认前向搜索
  bool search_backward = false;  // search FORWARD by default

  double pull_over_s = 0.0;
  if (pull_over_status.pull_over_type() ==
      PullOverStatus::EMERGENCY_PULL_OVER) {...}

  int idx = 0;
  if (search_backward) {
    // 后向搜索，定位pull over末尾的一个点.
    idx = static_cast<int>(path_bound.size()) - 1;
    while (idx >= 0 && std::get<0>(path_bound[idx]) > pull_over_s) {
      --idx;
    }
  } else {
    // 前向搜索，定位emergency pull over开头后的第一个点.
    while (idx < static_cast<int>(path_bound.size()) &&
           std::get<0>(path_bound[idx]) < pull_over_s) {
      ++idx;
    }
  }
  
  // 为pull over搜索到一个可行的位置，主要是确定该区域的宽度和长度
  const double pull_over_space_length =
      kPulloverLonSearchCoeff *
          VehicleConfigHelper::GetConfig().vehicle_param().length() -
      FLAGS_obstacle_lon_start_buffer - FLAGS_obstacle_lon_end_buffer;
  const double pull_over_space_width =
      (kPulloverLatSearchCoeff - 1.0) *
      VehicleConfigHelper::GetConfig().vehicle_param().width();
  const double adc_half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;

  // 2. Find a window that is close to road-edge.
  /*
    这里用了内外两层循环进行搜索，外层循环控制搜索的开始的端点idx。
    内层控制另一个端点。根据找到的两个端点，判断区域是否可以pull over
  */
  bool has_a_feasible_window = false;
  while ((search_backward && idx >= 0 &&
          std::get<0>(path_bound[idx]) - std::get<0>(path_bound.front()) >
              pull_over_space_length) ||
         (!search_backward && idx < static_cast<int>(path_bound.size()) &&
          std::get<0>(path_bound.back()) - std::get<0>(path_bound[idx]) >
              pull_over_space_length)) {

    while ((search_backward && j >= 0 &&
            std::get<0>(path_bound[idx]) - std::get<0>(path_bound[j]) <
                pull_over_space_length) ||
           (!search_backward && j < static_cast<int>(path_bound.size()) &&
            std::get<0>(path_bound[j]) - std::get<0>(path_bound[idx]) <
                pull_over_space_length)) {...}
    
    // 找到可行区域，获取停车区域的位置和姿态
    if (is_feasible_window) {
    ...
    break;}
    ...}  // 外层while
...
}
```

3. lane change

<img src="D:\project\Git\dig-into-apollo\modules\planning\img\decider_path_bounds04.png" alt="decider_path_bounds04" style="zoom: 67%;" />

代码流程如下：

```c++
Status PathBoundsDecider::GenerateLaneChangePathBound(
    const ReferenceLineInfo& reference_line_info,
    std::vector<std::tuple<double, double, double>>* const path_bound) {
  // 1.初始化，和前面的步骤类似
  if (!InitPathBoundary(reference_line_info, path_bound)) {...}


  // 2. 根据道路和adc的信息获取一个大致的路径边界
  std::string dummy_borrow_lane_type;
  if (!GetBoundaryFromLanesAndADC(reference_line_info,
                                  LaneBorrowInfo::NO_BORROW, 0.1, path_bound,
                                  &dummy_borrow_lane_type, true)) {...}
 

  // 3. Remove the S-length of target lane out of the path-bound.
  GetBoundaryFromLaneChangeForbiddenZone(reference_line_info, path_bound);


  // 根据静态障碍物调整边界.
  if (!GetBoundaryFromStaticObstacles(reference_line_info.path_decision(),
                                      path_bound, &blocking_obstacle_id)) {...}
  ...
}
```

GetBoundaryFromLaneChangeForbiddenZone函数是lane change重要的函数。运行过程如下：

- 如果当前位置可以变道，则直接变道
- 如果有一个lane-change的起点，则直接使用它
- 逐个检查变道前的点的边界，改变边界的值（如果已经过了变道点，则返回）

```c++
void PathBoundsDecider::GetBoundaryFromLaneChangeForbiddenZone(
    const ReferenceLineInfo& reference_line_info, PathBound* const path_bound) {


  // 1.当前位置直接变道。
  auto* lane_change_status = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_change_lane();
  if (lane_change_status->is_clear_to_change_lane()) {
    ADEBUG << "Current position is clear to change lane. No need prep s.";
    lane_change_status->set_exist_lane_change_start_position(false);
    return;
  }


  // 2.如果已经有一个lane-change的起点，就直接使用它，否则再找一个
  double lane_change_start_s = 0.0;
  if (lane_change_status->exist_lane_change_start_position()) {
    common::SLPoint point_sl;
    reference_line.XYToSL(lane_change_status->lane_change_start_position(),
                          &point_sl);
    lane_change_start_s = point_sl.s();
  } else {
    // TODO(jiacheng): train ML model to learn this.
    // 设置为adc前方一段距离为变道起始点
    lane_change_start_s = FLAGS_lane_change_prepare_length + adc_frenet_s_;

    // Update the decided lane_change_start_s into planning-context.
    // 更新变道起始点的信息
    common::SLPoint lane_change_start_sl;
    lane_change_start_sl.set_s(lane_change_start_s);
    lane_change_start_sl.set_l(0.0);
    common::math::Vec2d lane_change_start_xy;
    reference_line.SLToXY(lane_change_start_sl, &lane_change_start_xy);
    lane_change_status->set_exist_lane_change_start_position(true);
    lane_change_status->mutable_lane_change_start_position()->set_x(
        lane_change_start_xy.x());
    lane_change_status->mutable_lane_change_start_position()->set_y(
        lane_change_start_xy.y());
  }

  // Remove the target lane out of the path-boundary, up to the decided S.
  // 逐个检查变道前的点的边界，改变边界的值
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_s = std::get<0>((*path_bound)[i]);
    if (curr_s > lane_change_start_s) {
      break;
    }
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    double offset_to_map = 0.0;
    reference_line.GetOffsetToMap(curr_s, &offset_to_map);
    if (reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                    &curr_lane_right_width)) {
      double offset_to_lane_center = 0.0;
      reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      curr_lane_left_width += offset_to_lane_center;
      curr_lane_right_width -= offset_to_lane_center;
    }
    curr_lane_left_width -= offset_to_map;
    curr_lane_right_width += offset_to_map;

    std::get<1>((*path_bound)[i]) =
        adc_frenet_l_ > curr_lane_left_width
            ? curr_lane_left_width + GetBufferBetweenADCCenterAndEdge()
            : std::get<1>((*path_bound)[i]);
    std::get<1>((*path_bound)[i]) =
        std::fmin(std::get<1>((*path_bound)[i]), adc_frenet_l_ - 0.1);
    std::get<2>((*path_bound)[i]) =
        adc_frenet_l_ < -curr_lane_right_width
            ? -curr_lane_right_width - GetBufferBetweenADCCenterAndEdge()
            : std::get<2>((*path_bound)[i]);
    std::get<2>((*path_bound)[i]) =
        std::fmax(std::get<2>((*path_bound)[i]), adc_frenet_l_ + 0.1);
  }
}
```

4. Regular

<img src="D:\project\Git\dig-into-apollo\modules\planning\img\decider_path_bounds05.png" alt="decider_path_bounds05" style="zoom:67%;" />

代码流程如下：

```c++
Status PathBoundsDecider::GenerateRegularPathBound(
    const ReferenceLineInfo& reference_line_info,
    const LaneBorrowInfo& lane_borrow_info, PathBound* const path_bound,
    std::string* const blocking_obstacle_id,
    std::string* const borrow_lane_type) {
  // 1.初始化边界.
  if (!InitPathBoundary(reference_line_info, path_bound)) {...}


  // 2.根据adc位置和lane信息确定大致的边界
  if (!GetBoundaryFromLanesAndADC(reference_line_info, lane_borrow_info, 0.1,
                                  path_bound, borrow_lane_type)) {...}
  // PathBoundsDebugString(*path_bound);

  // 3.根据障碍物调整道路边界
  if (!GetBoundaryFromStaticObstacles(reference_line_info.path_decision(),
                                      path_bound, blocking_obstacle_id)) {...}
  ...
}
```

流程和上面的几个基本类似，借道有三种类型

```c++
  enum class LaneBorrowInfo {
    LEFT_BORROW,
    NO_BORROW,
    RIGHT_BORROW,
  };
```

# 分段加加速度路径优化

## 代码流程及框架

`分段加加速度路径优化`代码的流程图如下，

<img src="D:\project\Git\dig-into-apollo\modules\planning\img\optimizer_piecewise_jerk_path.png" alt="optimizer_piecewise_jerk_path" style="zoom: 80%;" />

* 如果重复使用path则return

```c++
common::Status PiecewiseJerkPathOptimizer::Process(
    const SpeedData& speed_data, const ReferenceLine& reference_line,
    const common::TrajectoryPoint& init_point, const bool path_reusable,
    PathData* const final_path_data) {
  // 跳过piecewise_jerk_path_optimizer 如果路径重复使用
  if (FLAGS_enable_skip_path_tasks && path_reusable) {
    return Status::OK();
  }
  ... ...
```

* adc起始点转化为frenet坐标

```c++
  ... ...
  const auto init_frenet_state =
      reference_line.ToFrenetFrame(planning_start_point);

  // 为lane-change选择lane_change_path_config
  // 否则, 选择default_path_config
  const auto& config = reference_line_info_->IsChangeLanePath()
                           ? config_.piecewise_jerk_path_optimizer_config()
                                 .lane_change_path_config()
                           : config_.piecewise_jerk_path_optimizer_config()
                                 .default_path_config();
  ... ...
```

* 遍历每个路径边界

```c++
  ... ...
  const auto& path_boundaries =
      reference_line_info_->GetCandidatePathBoundaries();
  ADEBUG << "There are " << path_boundaries.size() << " path boundaries.";
  const auto& reference_path_data = reference_line_info_->path_data();

  std::vector<PathData> candidate_path_data;
  // 遍历每个路径
  for (const auto& path_boundary : path_boundaries) {
    size_t path_boundary_size = path_boundary.boundary().size();
  ... ...
```

* 判断是否pull-over或regular

  ① 判断是否pull-over

  ```
      ... ...
      if (!FLAGS_enable_force_pull_over_open_space_parking_test) {
        // pull over场景
        const auto& pull_over_status =
            injector_->planning_context()->planning_status().pull_over();
        if (pull_over_status.has_position() &&
            pull_over_status.position().has_x() &&
            pull_over_status.position().has_y() &&
            path_boundary.label().find("pullover") != std::string::npos) {
          common::SLPoint pull_over_sl;
          reference_line.XYToSL(pull_over_status.position(), &pull_over_sl);
          end_state[0] = pull_over_sl.l();
        }
      }
      ... ...
  ```

  ② 判断是否regular

  ```c++
      ... ...
      if (path_boundary.label().find("regular") != std::string::npos &&
          reference_path_data.is_valid_path_reference()) {
        ADEBUG << "path label is: " << path_boundary.label();
        // 当参考路径就位
        for (size_t i = 0; i < path_reference_size; ++i) {
          common::SLPoint path_reference_sl;
          reference_line.XYToSL(
              common::util::PointFactory::ToPointENU(
                  reference_path_data.path_reference().at(i).x(),
                  reference_path_data.path_reference().at(i).y()),
              &path_reference_sl);
          path_reference_l[i] = path_reference_sl.l();
        }
        end_state[0] = path_reference_l.back();
        path_data.set_is_optimized_towards_trajectory_reference(true);
        is_valid_path_reference = true;
      }
      ... ...
  ```

* 优化路径

  优化过程：

  1. 定义piecewise_jerk_problem变量，优化算法；
  2. 设置变量（a.权重  b.横向距离、速度、加速度边界  c.最大转角速度  d.jerk bound）
  3. 优化算法
  4. 获取结果

```c++
    ... ...
    // 设置参数
    const auto& veh_param =
        common::VehicleConfigHelper::GetConfig().vehicle_param();
    const double lat_acc_bound =
        std::tan(veh_param.max_steer_angle() / veh_param.steer_ratio()) /
        veh_param.wheel_base();
    std::vector<std::pair<double, double>> ddl_bounds;
    for (size_t i = 0; i < path_boundary_size; ++i) {
      double s = static_cast<double>(i) * path_boundary.delta_s() +
                 path_boundary.start_s();
      double kappa = reference_line.GetNearestReferencePoint(s).kappa();
      ddl_bounds.emplace_back(-lat_acc_bound - kappa, lat_acc_bound - kappa);
    }
    // 优化算法
    bool res_opt = OptimizePath(
        init_frenet_state.second, end_state, std::move(path_reference_l),
        path_reference_size, path_boundary.delta_s(), is_valid_path_reference,
        path_boundary.boundary(), ddl_bounds, w, max_iter, &opt_l, &opt_dl,
        &opt_ddl);
    ... ...
```

* 如果成功，将值保存在candidate_path_data

```c++
    ... ...
    if (res_opt) {
      for (size_t i = 0; i < path_boundary_size; i += 4) {
        ADEBUG << "for s[" << static_cast<double>(i) * path_boundary.delta_s()
               << "], l = " << opt_l[i] << ", dl = " << opt_dl[i];
      }
      auto frenet_frame_path =
          ToPiecewiseJerkPath(opt_l, opt_dl, opt_ddl, path_boundary.delta_s(),
                              path_boundary.start_s());

      path_data.SetReferenceLine(&reference_line);
      path_data.SetFrenetPath(std::move(frenet_frame_path));
      if (FLAGS_use_front_axe_center_in_path_planning) {
        auto discretized_path = DiscretizedPath(
            ConvertPathPointRefFromFrontAxeToRearAxe(path_data));
        path_data.SetDiscretizedPath(discretized_path);
      }
      path_data.set_path_label(path_boundary.label());
      path_data.set_blocking_obstacle_id(path_boundary.blocking_obstacle_id());
      candidate_path_data.push_back(std::move(path_data));
    }
    ... ...
```

* 失败则返回错误码

```c++
  ... ...
  if (candidate_path_data.empty()) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "Path Optimizer failed to generate path");
  }
  reference_line_info_->SetCandidatePathData(std::move(candidate_path_data));
  return Status::OK();
  ... ...
```

## 算法解析

`分段加加速度路径优化`算法详细介绍在论文《Optimal Vehicle Path Planning Using Quadratic Optimization for Baidu Apollo Open Platform》中。 

# 路径评估决策

## 代码流程及框架

代码主体流程如下所示：

<img src="D:\project\Git\dig-into-apollo\modules\planning\img\decider_path_assessment.png" alt="decider_path_assessment" style="zoom:80%;" />

* 路径重复使用

  ```c++
    ... ...
    // 如果路径重复使用则跳过
    if (FLAGS_enable_skip_path_tasks && reference_line_info->path_reusable()) {
      return Status::OK();
    ... ...
  ```

* 去掉无效路径

  ```c++
    ... ...
    // 1. 删掉无效路径.
    std::vector<PathData> valid_path_data;
    for (const auto& curr_path_data : candidate_path_data) {
      // RecordDebugInfo(curr_path_data, curr_path_data.path_label(),
      //                 reference_line_info);
      if (curr_path_data.path_label().find("fallback") != std::string::npos) {
        if (IsValidFallbackPath(*reference_line_info, curr_path_data)) {
          valid_path_data.push_back(curr_path_data);
        }
      } else {
        if (IsValidRegularPath(*reference_line_info, curr_path_data)) {
          valid_path_data.push_back(curr_path_data);
        }
      }
    }
    const auto& end_time1 = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end_time1 - end_time0;
    ADEBUG << "Time for path validity checking: " << diff.count() * 1000
           << " msec.";
    ... ...
  ```

  其中fallback的无效路径是偏离参考线以及道路的路径。regular的无效路径是偏离参考线，道路，碰撞，停在相邻的逆向车道的路径。

* 分析并加入重要信息

  这一步骤的代码执行流程如下：

  1）去掉空的路径

  2）从尾部开始剪掉lane-borrow路径，从尾部开始向前搜索，剪掉如下类型path_point：

  1. OUT_ON_FORWARD_LANE
  2. OUT_ON_REVERSE_LANE
  3. 未知类型

  3）找到自车道的障碍物id，用于车道选择

  4）如果没有有效路径，返回错误码

  ```c++
    ... ...
    // 2. 分析并加入重要信息给speed决策
    size_t cnt = 0;
    const Obstacle* blocking_obstacle_on_selflane = nullptr;
    for (size_t i = 0; i != valid_path_data.size(); ++i) {
      auto& curr_path_data = valid_path_data[i];
      if (curr_path_data.path_label().find("fallback") != std::string::npos) {
        // remove empty path_data.
        if (!curr_path_data.Empty()) {
          if (cnt != i) {
            valid_path_data[cnt] = curr_path_data;
          }
          ++cnt;
        }
        continue;
      }
      SetPathInfo(*reference_line_info, &curr_path_data);
      // 修剪所有的借道路径，使其能够以in-lane结尾
      if (curr_path_data.path_label().find("pullover") == std::string::npos) {
        TrimTailingOutLanePoints(&curr_path_data);
      }
  
      // 找到 blocking_obstacle_on_selflane, 为下一步选择车道做准备
      if (curr_path_data.path_label().find("self") != std::string::npos) {
        const auto blocking_obstacle_id = curr_path_data.blocking_obstacle_id();
        blocking_obstacle_on_selflane =
            reference_line_info->path_decision()->Find(blocking_obstacle_id);
      }
  
      // 删掉空路径
      if (!curr_path_data.Empty()) {
        if (cnt != i) {
          valid_path_data[cnt] = curr_path_data;
        }
        ++cnt;
      }
  
      // RecordDebugInfo(curr_path_data, curr_path_data.path_label(),
      //                 reference_line_info);
      ADEBUG << "For " << curr_path_data.path_label() << ", "
             << "path length = " << curr_path_data.frenet_frame_path().size();
    }
    valid_path_data.resize(cnt);
    // 如果没有有效路径，退出
    if (valid_path_data.empty()) {
      const std::string msg = "Neither regular nor fallback path is valid.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    ADEBUG << "There are " << valid_path_data.size() << " valid path data.";
    const auto& end_time2 = std::chrono::system_clock::now();
    diff = end_time2 - end_time1;
    ADEBUG << "Time for path info labeling: " << diff.count() * 1000 << " msec.";
    ... ...
  ```

* 排序并选出最优路径

  ```c++
    ... ...
    // 3. Pick the optimal path.
    std::sort(valid_path_data.begin(), valid_path_data.end(),
              std::bind(ComparePathData, std::placeholders::_1,
                        std::placeholders::_2, blocking_obstacle_on_selflane));
  
    ADEBUG << "Using '" << valid_path_data.front().path_label()
           << "' path out of " << valid_path_data.size() << " path(s)";
    if (valid_path_data.front().path_label().find("fallback") !=
        std::string::npos) {
      FLAGS_static_obstacle_nudge_l_buffer = 0.8;
    }
    *(reference_line_info->mutable_path_data()) = valid_path_data.front();
    reference_line_info->SetBlockingObstacle(
        valid_path_data.front().blocking_obstacle_id());
    const auto& end_time3 = std::chrono::system_clock::now();
    diff = end_time3 - end_time2;
    ADEBUG << "Time for optimal path selection: " << diff.count() * 1000
           << " msec.";
  
    reference_line_info->SetCandidatePathData(std::move(valid_path_data));
    ... ...
  ```

  排序算法的具体流程如下：

  ```c++
  bool ComparePathData(const PathData& lhs, const PathData& rhs,
                       const Obstacle* blocking_obstacle){
      ... ...
  }
  ```

  路径排序：（道路评估的优劣通过排序获得）

  1、空的路径永远排在后面

  2、regular > fallback

  3、如果self-lane有一个存在，选择那个。如果都存在，选择较长的，如果长度接近，选择self-lane；如果self-lane都不存在，选择较长的路径

  4、如果路径长度接近，且都要借道

  1. 都要借逆向车道，选择距离短的
  2. 针对具有两个借道方向的情况：
     * 有障碍物，根据障碍物的横向位置选择借道方向（向左或向右）
     * 无障碍物，根据自车的横向位置选择借道方向
  3. 相邻车道都是前向的，选择较早返回自车道的路径
  4. 前向借道，返回自车道的时间相同，选择从左侧借道的路径

  5、最后则两条路径相同，lhs is not < rhs

  排序之后选择最优路径，即第一条路径

* 更新必要信息

  1、更新adc前方静态障碍物的信息

  2、更新自车道使用信息

  3、更新side-pass的方向

  * 根据PathDeciderStatus是RIGHT_BORROW或LEFT_BORROW判断是从左侧的借道，还是从右侧借道

  ```c++
    // 4. Update necessary info for lane-borrow decider's future uses.
    // Update front static obstacle's info.
    auto* mutable_path_decider_status = injector_->planning_context()
                                            ->mutable_planning_status()
                                            ->mutable_path_decider();
    if (reference_line_info->GetBlockingObstacle() != nullptr) {
      int front_static_obstacle_cycle_counter =
          mutable_path_decider_status->front_static_obstacle_cycle_counter();
      mutable_path_decider_status->set_front_static_obstacle_cycle_counter(
          std::max(front_static_obstacle_cycle_counter, 0));
      mutable_path_decider_status->set_front_static_obstacle_cycle_counter(
          std::min(front_static_obstacle_cycle_counter + 1, 10));
      mutable_path_decider_status->set_front_static_obstacle_id(
          reference_line_info->GetBlockingObstacle()->Id());
    } else {
      int front_static_obstacle_cycle_counter =
          mutable_path_decider_status->front_static_obstacle_cycle_counter();
      mutable_path_decider_status->set_front_static_obstacle_cycle_counter(
          std::min(front_static_obstacle_cycle_counter, 0));
      mutable_path_decider_status->set_front_static_obstacle_cycle_counter(
          std::max(front_static_obstacle_cycle_counter - 1, -10));
    }
  
    // Update self-lane usage info.
    if (reference_line_info->path_data().path_label().find("self") !=
        std::string::npos) {
      // && std::get<1>(reference_line_info->path_data()
      //                 .path_point_decision_guide()
      //                 .front()) == PathData::PathPointType::IN_LANE)
      int able_to_use_self_lane_counter =
          mutable_path_decider_status->able_to_use_self_lane_counter();
  
      if (able_to_use_self_lane_counter < 0) {
        able_to_use_self_lane_counter = 0;
      }
      mutable_path_decider_status->set_able_to_use_self_lane_counter(
          std::min(able_to_use_self_lane_counter + 1, 10));
    } else {
      mutable_path_decider_status->set_able_to_use_self_lane_counter(0);
    }
  
    // Update side-pass direction.
    if (mutable_path_decider_status->is_in_path_lane_borrow_scenario()) {
      bool left_borrow = false;
      bool right_borrow = false;
      const auto& path_decider_status =
          injector_->planning_context()->planning_status().path_decider();
      for (const auto& lane_borrow_direction :
           path_decider_status.decided_side_pass_direction()) {
        if (lane_borrow_direction == PathDeciderStatus::LEFT_BORROW &&
            reference_line_info->path_data().path_label().find("left") !=
                std::string::npos) {
          left_borrow = true;
        }
        if (lane_borrow_direction == PathDeciderStatus::RIGHT_BORROW &&
            reference_line_info->path_data().path_label().find("right") !=
                std::string::npos) {
          right_borrow = true;
        }
      }
  
      mutable_path_decider_status->clear_decided_side_pass_direction();
      if (right_borrow) {
        mutable_path_decider_status->add_decided_side_pass_direction(
            PathDeciderStatus::RIGHT_BORROW);
      }
      if (left_borrow) {
        mutable_path_decider_status->add_decided_side_pass_direction(
            PathDeciderStatus::LEFT_BORROW);
      }
    }
    const auto& end_time4 = std::chrono::system_clock::now();
    diff = end_time4 - end_time3;
    ADEBUG << "Time for FSM state updating: " << diff.count() * 1000 << " msec.";
  
    // Plot the path in simulator for debug purpose.
    RecordDebugInfo(reference_line_info->path_data(), "Planning PathData",
                    reference_line_info);
    return Status::OK();
  ```

# 路径决策

## 代码流程及框架

`路径决策`的整体流程图如下：

<img src="D:\project\Git\dig-into-apollo\modules\planning\img\path_decider.png" alt="path_decider" style="zoom:80%;" />

在`Process`函数中主要调用了`MakeObjectDecision`函数，而其又主要调用了`MakeStaticObstacleDecision`函数，所以路径决策的主要功能都在`MakeStaticObstacleDecision`中。

## 算法解析

* 获取frenet坐标系下的坐标

  ```c++
    ... ...
    // 1.获取frenet坐标下的path路径
    const auto &frenet_path = path_data.frenet_frame_path();
    if (frenet_path.empty()) {
      AERROR << "Path is empty.";
      return false;
    }
    ... ...
  ```

* 根据障碍物做决策

  ```c++
    ... ...
    // 2.遍历每个障碍物，做决策
    for (const auto *obstacle : path_decision->obstacles().Items()) {
      const std::string &obstacle_id = obstacle->Id();
      const std::string obstacle_type_name =
          PerceptionObstacle_Type_Name(obstacle->Perception().type());
      ADEBUG << "obstacle_id[<< " << obstacle_id << "] type["
             << obstacle_type_name << "]";
      ... ...
  ```

上图的红框中是循环体的主要内容，主要功能是遍历每个障碍物做决策。

* 如果障碍物不是静态或virtual，则跳过

  ```c++
      // 2.1 如果障碍物不是静态的或者是virtual的，就跳过
      if (!obstacle->IsStatic() || obstacle->IsVirtual()) {    // （stop fence，各种fence）
        continue;
      }
  ```

* 如果障碍物有了ignore/stop决策，则跳过

  ```c++
      // 2.2 如果障碍物已经有 ignore/stop 决策，就跳过
      if (obstacle->HasLongitudinalDecision() &&
          obstacle->LongitudinalDecision().has_ignore() &&
          obstacle->HasLateralDecision() &&
          obstacle->LateralDecision().has_ignore()) {
        continue;
      }
      if (obstacle->HasLongitudinalDecision() &&
          obstacle->LongitudinalDecision().has_stop()) {
        // STOP decision
        continue;
      }
  ```

* 如果障碍物挡住了路径，加stop决策

  ```c++
      // 2.3 如果障碍物挡住了路径，加stop决策
      if (obstacle->Id() == blocking_obstacle_id &&
          !injector_->planning_context()
               ->planning_status()
               .path_decider()
               .is_in_path_lane_borrow_scenario()) {
        // Add stop decision
        ADEBUG << "Blocking obstacle = " << blocking_obstacle_id;
        ObjectDecisionType object_decision;
        *object_decision.mutable_stop() = GenerateObjectStopDecision(*obstacle);
        path_decision->AddLongitudinalDecision("PathDecider/blocking_obstacle",
                                               obstacle->Id(), object_decision);
        continue;
      }
  ```

* 如果是clear-zone，则跳过

  ```c++
      // 2.4 如果是clear-zone，跳过
      if (obstacle->reference_line_st_boundary().boundary_type() ==
          STBoundary::BoundaryType::KEEP_CLEAR) {
        continue;
      }
  ```

* 如果障碍物不在路径上，跳过

  ```c++
      // 2.5 如果障碍物不在路径上，跳过
      ObjectDecisionType object_decision;
      object_decision.mutable_ignore();
      const auto &sl_boundary = obstacle->PerceptionSLBoundary();
      if (sl_boundary.end_s() < frenet_path.front().s() ||
          sl_boundary.start_s() > frenet_path.back().s()) {
        path_decision->AddLongitudinalDecision("PathDecider/not-in-s",
                                               obstacle->Id(), object_decision);
        path_decision->AddLateralDecision("PathDecider/not-in-s", obstacle->Id(),
                                          object_decision);
        continue;
      }
  ```

* nudge判断

  ```c++
      // 2.6 nudge判断，如果距离静态障碍物距离太远，则忽略。
      //               如果静态障碍物距离车道中心太近，则停止。
      //               如果横向方向很近，则避开。
      if (curr_l - lateral_radius > sl_boundary.end_l() ||
          curr_l + lateral_radius < sl_boundary.start_l()) {
        // 1. IGNORE if laterally too far away.
        path_decision->AddLateralDecision("PathDecider/not-in-l", obstacle->Id(),
                                          object_decision);
      } else if (sl_boundary.end_l() >= curr_l - min_nudge_l &&
                 sl_boundary.start_l() <= curr_l + min_nudge_l) {
        // 2. STOP if laterally too overlapping.
        *object_decision.mutable_stop() = GenerateObjectStopDecision(*obstacle);
  
        if (path_decision->MergeWithMainStop(
                object_decision.stop(), obstacle->Id(),
                reference_line_info_->reference_line(),
                reference_line_info_->AdcSlBoundary())) {
          path_decision->AddLongitudinalDecision("PathDecider/nearest-stop",
                                                 obstacle->Id(), object_decision);
        } else {
          ObjectDecisionType object_decision;
          object_decision.mutable_ignore();
          path_decision->AddLongitudinalDecision("PathDecider/not-nearest-stop",
                                                 obstacle->Id(), object_decision);
        }
      } else {
        // 3. NUDGE if laterally very close.
        if (sl_boundary.end_l() < curr_l - min_nudge_l) {  // &&
          // sl_boundary.end_l() > curr_l - min_nudge_l - 0.3) {
          // LEFT_NUDGE
          ObjectNudge *object_nudge_ptr = object_decision.mutable_nudge();
          object_nudge_ptr->set_type(ObjectNudge::LEFT_NUDGE);
          object_nudge_ptr->set_distance_l(
              config_.path_decider_config().static_obstacle_buffer());
          path_decision->AddLateralDecision("PathDecider/left-nudge",
                                            obstacle->Id(), object_decision);
        } else if (sl_boundary.start_l() > curr_l + min_nudge_l) {  // &&
          // sl_boundary.start_l() < curr_l + min_nudge_l + 0.3) {
          // RIGHT_NUDGE
          ObjectNudge *object_nudge_ptr = object_decision.mutable_nudge();
          object_nudge_ptr->set_type(ObjectNudge::RIGHT_NUDGE);
          object_nudge_ptr->set_distance_l(
              -config_.path_decider_config().static_obstacle_buffer());
          path_decision->AddLateralDecision("PathDecider/right-nudge",
                                            obstacle->Id(), object_decision);
        }
      }
  ```

  
