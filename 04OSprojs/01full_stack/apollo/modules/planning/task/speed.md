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

# 基于规则的停止决策

## 代码流程及框架

代码的运行流程如下图：

<img src="D:\project\Git\dig-into-apollo\modules\planning\img\decider_rule_based_stop.png" alt="decider_rule_based_stop" style="zoom:80%;" />

代码结构比较清楚：

```c++
apollo::common::Status RuleBasedStopDecider::Process(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  // 1. 逆向车道通过，停止
  StopOnSidePass(frame, reference_line_info);

  // 2. 紧急换道，停止
  if (FLAGS_enable_lane_change_urgency_checking) {
    CheckLaneChangeUrgency(frame);
  }

  // 3. 路径尽头，停止
  AddPathEndStop(frame, reference_line_info);

  return Status::OK();
}
```

## 相关算法解析

* stop on side pass

  <img src="D:\project\Git\dig-into-apollo\modules\planning\img\stop_on_side_pass.png" alt="stop_on_side_pass" style="zoom: 67%;" />

  代码如下：

  ```c++
  void RuleBasedStopDecider::StopOnSidePass(
      Frame *const frame, ReferenceLineInfo *const reference_line_info) {
    static bool check_clear;    // 默认false
    static common::PathPoint change_lane_stop_path_point;
  
    // 获取path_data
    const PathData &path_data = reference_line_info->path_data();
    double stop_s_on_pathdata = 0.0;
  
    // 1. 找到"self"，直接return
    if (path_data.path_label().find("self") != std::string::npos) {
      check_clear = false;
      change_lane_stop_path_point.Clear();
      return;
    }
  
    // 2. 如果check_clear为true，且CheckClearDone成功。设置check_clear为false
    if (check_clear &&
        CheckClearDone(*reference_line_info, change_lane_stop_path_point)) {
      check_clear = false;
    }
  
    // 3.如果check_clear为false，且检查stop fence
    if (!check_clear &&
        CheckSidePassStop(path_data, *reference_line_info, &stop_s_on_pathdata)) {
      // 3.1 如果障碍物没有阻塞且可以换道，直接return
      if (!LaneChangeDecider::IsPerceptionBlocked(
              *reference_line_info,
              rule_based_stop_decider_config_.search_beam_length(),
              rule_based_stop_decider_config_.search_beam_radius_intensity(),
              rule_based_stop_decider_config_.search_range(),
              rule_based_stop_decider_config_.is_block_angle_threshold()) &&
          LaneChangeDecider::IsClearToChangeLane(reference_line_info)) {
        return;
      }
      // 3.2 检查adc是否停在了stop fence前，否返回true
      if (!CheckADCStop(path_data, *reference_line_info, stop_s_on_pathdata)) {
        // 设置stop fence，成功就执行 check_clear = true;
        if (!BuildSidePassStopFence(path_data, stop_s_on_pathdata,
                                    &change_lane_stop_path_point, frame,
                                    reference_line_info)) {
          AERROR << "Set side pass stop fail";
        }
      } else {
        if (LaneChangeDecider::IsClearToChangeLane(reference_line_info)) {
          check_clear = true;
        }
      }
    }
  }
  ```

* check lane change urgency

  <img src="D:\project\Git\dig-into-apollo\modules\planning\img\check_lane_change_urgency.png" alt="check_lane_change_urgency" style="zoom:80%;" />

  检查紧急换道，代码如下：

  ```c++
  void RuleBasedStopDecider::CheckLaneChangeUrgency(Frame *const frame) {
    // 直接进入循环，检查每个reference_line_info
    for (auto &reference_line_info : *frame->mutable_reference_line_info()) {
  
      // 1. 检查目标道路是否阻塞，如果在change lane path上，就跳过
      if (reference_line_info.IsChangeLanePath()) {
        is_clear_to_change_lane_ =
            LaneChangeDecider::IsClearToChangeLane(&reference_line_info);
        is_change_lane_planning_succeed_ =
            reference_line_info.Cost() < kStraightForwardLineCost;
        continue;
      }
  
      // 2.如果不是换道的场景，或者（目标lane没有阻塞）并且换道规划成功，跳过
      if (frame->reference_line_info().size() <= 1 ||
          (is_clear_to_change_lane_ && is_change_lane_planning_succeed_)) {
        continue;
      }
      // When the target lane is blocked in change-lane case, check the urgency
      // Get the end point of current routing
      const auto &route_end_waypoint =
          reference_line_info.Lanes().RouteEndWaypoint();
  
      // 3.在route的末端无法获得lane，跳过
      if (!route_end_waypoint.lane) {
        continue;
      }
      auto point = route_end_waypoint.lane->GetSmoothPoint(route_end_waypoint.s);
      auto *reference_line = reference_line_info.mutable_reference_line();
      common::SLPoint sl_point;
  
      // 将当前参考线的点映射到frenet坐标系下
      if (reference_line->XYToSL(point, &sl_point) &&
          reference_line->IsOnLane(sl_point)) {
        // Check the distance from ADC to the end point of current routing
        double distance_to_passage_end =
            sl_point.s() - reference_line_info.AdcSlBoundary().end_s();
  
        // 4. 如果adc距离routing终点较远，不需要停止，跳过
        if (distance_to_passage_end >
            rule_based_stop_decider_config_.approach_distance_for_lane_change()) {
          continue;
        }
  
        // 5.如果遇到紧急情况，设置临时的stop fence，等待换道
        const std::string stop_wall_id = "lane_change_stop";
        std::vector<std::string> wait_for_obstacles;
        util::BuildStopDecision(
            stop_wall_id, sl_point.s(),
            rule_based_stop_decider_config_.urgent_distance_for_lane_change(),
            StopReasonCode::STOP_REASON_LANE_CHANGE_URGENCY, wait_for_obstacles,
            "RuleBasedStopDecider", frame, &reference_line_info);
      }
    }
  }
  ```

* add path end stop

  <img src="D:\project\Git\dig-into-apollo\modules\planning\img\add_path_end_stop.png" alt="add_path_end_stop" style="zoom:80%;" />

  在道路的尽头添加stop fence。代码如下：	

  ```c++
  void RuleBasedStopDecider::AddPathEndStop(
      Frame *const frame, ReferenceLineInfo *const reference_line_info) {
    if (!reference_line_info->path_data().path_label().empty() &&
        reference_line_info->path_data().frenet_frame_path().back().s() -
                reference_line_info->path_data().frenet_frame_path().front().s() <
            FLAGS_short_path_length_threshold) {
      const std::string stop_wall_id =
          PATH_END_VO_ID_PREFIX + reference_line_info->path_data().path_label();
      std::vector<std::string> wait_for_obstacles;
  
      // 创建stop fence
      util::BuildStopDecision(
          stop_wall_id,
          reference_line_info->path_data().frenet_frame_path().back().s() - 5.0,
          0.0, StopReasonCode::STOP_REASON_REFERENCE_END, wait_for_obstacles,
          "RuleBasedStopDecider", frame, reference_line_info);
    }
  }
  ```


# ST边界决策

# 速度边界预先决策

调用speed_bounds_decider任务，其boundary_buffer取0.25。

# 启发式的速度优化（DP）

见`path_time_heuristic_optimizer`，其核心算法为动态规划

# 速度决策

# 速度边界最终决策

同样调用speed_bounds_decider任务，其boundary_buffer取0.1

# 分段加加速度非线性速度优化









