#!/usr/bin/env python3
import sys
sys.path.append("/home/suda/.local/lib/python3.10/site-packages")
import rclpy  # Python ROS2核心API（替换rclcpp）
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy  # Python QoS配置
from rmf_fleet_msgs.msg import FleetState
from std_msgs.msg import String
from geometry_msgs.msg import Point
# 新增：导入自定义服务类型
from rmf_custom_tasks_self.srv import SingleNavTask
import numpy as np
from stable_baselines3 import PPO
import json
import time
import random

#1.16 感知无人车状态 → 接收目标任务 → RL 决策选车（硬编码） → 发送指定无人车的任务请求 → 配合监控节点完成任务  
# 可以跑通，硬编码了选车deliveryRobot_0，这个小车会按照发布顺序执行给它的八个任务

# ===================== 第一步：定义极简版RL环境（先跑通逻辑） =====================
class SimpleDispatchingEnv:
    def __init__(self):
        # 1. 初始化无人车信息（修正名称：带下划线，匹配RMF真实名称）
        self.robot_names = ["deliveryRobot_0", "deliveryRobot_1", "deliveryRobot_2"]
        self.robot_positions = {name: Point(x=0.0, y=0.0) for name in self.robot_names}
        self.robot_idle = {name: True for name in self.robot_names}  # 初始都空闲
        
        # 2. 目标信息
        self.pending_targets = []  # 待处置目标队列
        self.disposed_targets = 0  # 已处置目标数
        self.current_time = 0.0    # 当前时间（秒）
        
        # 3. RL动作空间：0=deliveryRobot_0, 1=deliveryRobot_1, 2=deliveryRobot_2
        self.action_space = [0, 1, 2]
        
        # 4. 状态维度：3台车*(x,y,idle) + 1个目标*(x,y,wait_time) + 环境*(current_time, disposed_num)
        self.state_dim = 3*3 + 3 + 2  # 修正语法错误：去掉=14
        
    def reset(self):
        """重置环境（训练时用，部署时仅初始化一次）"""
        self.robot_idle = {name: True for name in self.robot_names}
        self.pending_targets = []
        self.disposed_targets = 0
        self.current_time = 0.0
        return self._get_state()

    def _get_state(self):
        """构建状态向量（极简版，先跑通）"""
        # 1. 无人车状态：x,y,是否空闲（0/1）
        robot_state = []
        for name in self.robot_names:
            robot_state.extend([
                self.robot_positions[name].x,
                self.robot_positions[name].y,
                1.0 if self.robot_idle[name] else 0.0
            ])
        
        # 2. 目标状态（只取第一个待处置目标）
        target_state = [0.0, 0.0, 0.0]  # x,y,wait_time
        if self.pending_targets:
            target = self.pending_targets[0]
            target_state = [target["x"], target["y"], target["wait_time"]]
        
        # 3. 环境状态
        env_state = [self.current_time, self.disposed_targets]
        
        # 合并为状态向量
        return np.array(robot_state + target_state + env_state, dtype=np.float32)

    def step(self, action):
        """执行动作（部署时核心逻辑）"""
        # 1. 动作映射：0→deliveryRobot_0，1→deliveryRobot_1，2→deliveryRobot_2
        selected_robot = self.robot_names[action]
        
        # 2. 若无待处置目标，奖励为0
        if not self.pending_targets:
            return self._get_state(), 0.0, True, {}
        
        # 3. 取出第一个目标
        target = self.pending_targets.pop(0)
        
        # 4. 计算奖励（极简版：距离越近，奖励越高）
        robot_pos = self.robot_positions[selected_robot]
        distance = np.sqrt((robot_pos.x - target["x"])**2 + (robot_pos.y - target["y"])**2)
        reward = -distance  # 负号：距离越近，奖励越高
        
        # 5. 更新状态：标记无人车为忙碌，已处置目标数+1
        self.robot_idle[selected_robot] = False
        self.disposed_targets += 1
        
        # 6. 返回：新状态、奖励、是否结束、额外信息
        return self._get_state(), reward, False, {"selected_robot": selected_robot, "target": target}

# ===================== 第二步：RL调度节点（适配自定义任务服务调用） =====================
class RLDispatcherNode(Node):
    def __init__(self):
        super().__init__("rl_dispatcher_node")
        self.get_logger().info("🚀 初始化RL调度节点（适配自定义任务节点）...")
        
        # 1. 强制开启仿真时间（Python版本的参数设置）
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        
        # 2. 初始化RL环境
        self.rl_env = SimpleDispatchingEnv()
        
        # 3. 加载预训练模型（先注释，后续训练后放开）
        # self.model = PPO.load("rl_dispatching_model")
        
        # 4. 订阅无人车状态（复用你的fleet_state逻辑）
        self.fleet_state_sub = self.create_subscription(
            FleetState,
            "/fleet_states",
            self.fleet_state_callback,
            10
        )
        
        # 5. 订阅无人机发现的目标（对应你的/task_monitor/start话题）
        self.target_sub = self.create_subscription(
            String,
            "/task_monitor/start",
            self.target_callback,
            10
        )
        
        # 6. 核心新增：创建自定义任务服务客户端（参考C++的AutoSendWaypointsNode）
        self.nav_client = self.create_client(SingleNavTask, "/submit_single_nav_task")
        # 等待服务可用（启动时校验）
        while not self.nav_client.wait_for_service(timeout_sec=5.0):
            if not rclpy.ok():
                self.get_logger().error("❌ 等待服务时节点退出！")
                return
            self.get_logger().info("⏳ 等待/submit_single_nav_task服务可用...")
        self.get_logger().info("✅ 已连接自定义任务服务！")
        
        # 7. 航点坐标映射（完全复用你任务监控节点的硬编码）
        self.waypoint_coords = self._init_waypoint_coords()
        
        # 8. 定时器：更新当前时间和目标等待时间
        self.timer = self.create_timer(1.0, self.timer_callback)

        # ========== 新增：目标去重+防抖逻辑 ==========
        self.processed_target_ids = set()  # 记录已处理的目标ID，避免重复
        self.last_target_time = 0.0        # 最后一次处理目标的时间（防抖）
        self.DEBOUNCE_DELAY = 1.0          # 防抖延迟：1秒内不处理重复目标
        
        self.get_logger().info("✅ RL调度节点初始化完成！")

    def _init_waypoint_coords(self):
        """完全复用你任务监控节点的航点坐标"""
        waypoint_coords = {}
        waypoint_coords["junction_n01"] = Point(x=1.57, y=-45.93)
        waypoint_coords["n08"] = Point(x=59.61, y=-7.42)
        waypoint_coords["n14"] = Point(x=80.84, y=-28.52)
        waypoint_coords["n13"] = Point(x=84.44, y=-4.94)
        waypoint_coords["n23"] = Point(x=182.80, y=-42.30)
        waypoint_coords["west_koi_pond"] = Point(x=34.32, y=-10.13)
        waypoint_coords["s08"] = Point(x=96.61, y=-50.50)
        waypoint_coords["s10"] = Point(x=122.10, y=-46.68)
        waypoint_coords["s11"] = Point(x=152.73, y=-43.00)
        waypoint_coords["junction_south_west"] = Point(x=84.56, y=-38.81)
        return waypoint_coords

    def fleet_state_callback(self, msg):
        """更新无人车位置和状态（完全复用你的逻辑）"""
        if msg.name != "deliveryRobot":
            return
        for robot in msg.robots:
            robot_name = robot.name
            if robot_name in self.rl_env.robot_names:
                self.rl_env.robot_positions[robot_name].x = robot.location.x
                self.rl_env.robot_positions[robot_name].y = robot.location.y
                # 简单判断是否空闲：task_id为空则空闲
                self.rl_env.robot_idle[robot_name] = (robot.task_id == "")
                self.get_logger().debug(
                    f"🔍 更新无人车状态：{robot_name} (x:{robot.location.x:.2f}, y:{robot.location.y:.2f}) 空闲: {self.rl_env.robot_idle[robot_name]}"
                )

    def target_callback(self, msg):
        """接收无人机发现的目标（适配你的消息格式：task_id,target_waypoint）"""
        # 解析消息：和你的task_monitor节点格式完全一致
        data = msg.data.split(",")
        if len(data) < 2:
            self.get_logger().error("❌ 目标消息格式错误：%s", msg.data)
            return
        
        # ========== 新增：解析目标ID和航点 ==========
        target_id = data[0].strip()
        target_waypoint = data[1].strip()
        
        # 1. 防抖检查：1秒内不处理重复请求
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_target_time < self.DEBOUNCE_DELAY:
            self.get_logger().debug(f"⚠️ 防抖过滤：{target_id}（距离上次处理不足{self.DEBOUNCE_DELAY}秒）")
            return
        
        # 2. 去重检查：已处理的目标ID不再处理
        if target_id in self.processed_target_ids:
            self.get_logger().debug(f"⚠️ 去重过滤：目标{target_id}已处理过")
            return
        
        # 3. 记录目标ID和处理时间
        self.processed_target_ids.add(target_id)
        self.last_target_time = current_time
        
        # 获取目标坐标（复用你的航点坐标）
        if target_waypoint not in self.waypoint_coords:
            self.get_logger().error("❌ 未知航点：%s", target_waypoint)
            return
        target_coords = self.waypoint_coords[target_waypoint]
        
        # 加入待处置队列
        self.rl_env.pending_targets.append({
            "task_id": target_id,
            "waypoint": target_waypoint,
            "x": target_coords.x,
            "y": target_coords.y,
            "wait_time": 0.0
        })
        self.get_logger().info(f"📥 新增目标：{target_id} -> {target_waypoint} (x:{target_coords.x:.2f}, y:{target_coords.y:.2f})")
        
        # 调用RL决策，指定无人车执行任务
        self._rl_dispatch()

    def timer_callback(self):
        """更新当前时间和目标等待时间"""
        self.rl_env.current_time += 1.0
        for target in self.rl_env.pending_targets:
            target["wait_time"] += 1.0

    def _rl_dispatch(self):
        """核心：RL决策并调用自定义任务服务（替换原JSON发布逻辑）"""
        # 1. 判空防护
        if not self.rl_env.pending_targets:
            self.get_logger().warn("⚠️ 待处置目标队列为空，跳过调度")
            return
        
        # 2. 获取当前状态
        state = self.rl_env._get_state()
        
        # 3. RL决策（先写死为deliveryRobot_0，后续替换为模型推理）
        # action, _ = self.model.predict(state, deterministic=True)  # 训练后放开
        action = 0  # 先固定选deliveryRobot_0，验证流程
        
        # 4. 执行动作，获取选中的无人车和目标
        _, _, _, info = self.rl_env.step(action)
        # 防止info为空
        if not info or "selected_robot" not in info or "target" not in info:
            self.get_logger().error("❌ RL决策返回空信息，跳过调度")
            return
        selected_robot = info["selected_robot"]
        target = info["target"]
        self.get_logger().info(f"🤖 RL决策：指定{selected_robot}执行目标{target['task_id']}")
        
        # 5. 核心修改：调用自定义任务服务（参考C++的send_waypoint_callback）
        self._call_nav_service(selected_robot, target)

    def _call_nav_service(self, robot_name, target):
        """调用自定义任务服务，发送指定小车的任务请求（修正API错误）"""
        # 构造服务请求（完全匹配C++的SingleNavTask服务）
        req = SingleNavTask.Request()
        req.target_waypoint = target["waypoint"]  # 目标航点
        req.fleet_name = "deliveryRobot"          # 固定车队名
        req.robot_name = robot_name               # RL决策选中的小车
        req.priority = 0                          # 优先级（和你的C++节点一致）
        
        # 核心修正：Python中异步调用用 call_async，而非 async_send_request
        future = self.nav_client.call_async(req)
        # 注册回调函数处理响应
        future.add_done_callback(self._nav_service_response_callback)
        
        self.get_logger().info(f"📤 调用自定义服务：{robot_name} -> {target['waypoint']}")

    def _nav_service_response_callback(self, future):
        """处理自定义服务的响应（验证调用结果）"""
        try:
            # 修正：future.result() 会返回服务响应
            res = future.result()
            if res.success:
                self.get_logger().info(f"✅ 任务发送成功！Task ID: {res.task_id}, 消息: {res.message}")
            else:
                self.get_logger().error(f"❌ 任务发送失败！消息: {res.message}")
        except Exception as e:
            self.get_logger().error(f"💥 服务调用异常：{str(e)}")


# ===================== 第三步：主函数 =====================
def main(args=None):
    rclpy.init(args=args)
    node = RLDispatcherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
