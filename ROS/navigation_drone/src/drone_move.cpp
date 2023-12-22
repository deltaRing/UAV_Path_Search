#include "drone_move.h"
#include "callback_functions.h"
#include "path_optimal/path_optimal.h"

// 发布消息
ros::Subscriber state_sub, battery_sub;
ros::Publisher local_pos_pub;
ros::ServiceClient arming_client, set_mode_client;

// 无人机状态
DroneStatus * droneStatus = new DroneStatus();
// 单芯低电压
double low_battery = 3.5;
double velocity_drone = 1.0;
double acc_drone = 5.0;
// 膨胀体积
extern int expand_range;
extern RRTstarPreparatory * _RRTstar_preparatory;

DroneStatus::DroneStatus() { 
    offb_set_mode.request.custom_mode = "OFFBOARD";
    expected_posture.header.frame_id = "map";
    expected_posture.type_mask = 0B101111111000;
    expected_posture.position.x = 0;
    expected_posture.position.y = 0;
    expected_posture.position.z = 1.0; // 默认的位置坐标
    expected_posture.velocity.x = 0;
    expected_posture.velocity.y = 0;
    expected_posture.velocity.z = 0;
    expected_posture.acceleration_or_force.x = 0.0;
    expected_posture.acceleration_or_force.y = 0.0;
    expected_posture.acceleration_or_force.z = 0.0;
    expected_posture.yaw = 0.0;
 }

void DroneStatus::setYawPID(double p, double i, double d) { this->yaw_p = p; this->yaw_i = i; this->yaw_d = d; }
void DroneStatus::setExpectedVelocity(double new_expected_velocity) { this->expected_velocity = new_expected_velocity; }
void DroneStatus::setDetectRange(double new_detect_range) { this->detect_range = new_detect_range; }
void DroneStatus::setCurrectPosture(geometry_msgs::PoseStamped Posture) { 
    this->currect_posture = Posture; 
    tf::Quaternion tf_posture(Posture.pose.orientation.x, 
                                                        Posture.pose.orientation.y, 
                                                        Posture.pose.orientation.z, 
                                                        Posture.pose.orientation.w);
    tf::Matrix3x3(tf_posture).getRPY(current_roll, current_pitch, current_yaw);
}
void DroneStatus::setRangeThreshold(double new_threshold) { this->range_threshold = new_threshold; } // 设置距离阈值 
void DroneStatus::setNewRoute(std::vector<Eigen::Vector3d> & newRoute) { this->theRoute = newRoute; } // 设置路线
void DroneStatus::setState(mavros_msgs::State state) { this->current_state = state; } // 设置状态（是否连接）
void DroneStatus::setBattery(sensor_msgs::BatteryState battery) { this->battery_is_set = true; this->current_battery = battery; } // 设置电池电压
void DroneStatus::setStatus(uint8_t new_status) { status = new_status; }
void DroneStatus::setMap(uint8_t * data) { this->map = data; }
void DroneStatus::setIsObs(uint8_t obs) { this->isObs = obs; }
void DroneStatus::setMapSizeX(int size_x) { this->GLX_SIZE = size_x; }
void DroneStatus::setMapSizeY(int size_y) { this->GLY_SIZE = size_y; }
void DroneStatus::setMapSizeZ(int size_z) { this->GLZ_SIZE = size_z; }
void DroneStatus::setResolution(double _res_) { resolution = _res_; inv_resolution = 1 / _res_; }
void DroneStatus::setMapGLx(double _gl_xl) { gl_xl = _gl_xl; }
void DroneStatus::setMapGLy(double _gl_yl) { gl_yl = _gl_yl; }
void DroneStatus::setMapGLz(double _gl_zl) { gl_zl = _gl_zl; }
void DroneStatus::setArm() {
    if (status == _INIT_ || status == _LAND_ || status == _ARMED_) { 
        arm_cmd.request.value = true;
        ROS_INFO("[node] try to arm the drone");
        setStatus(_ARMED_); 
    }
    else {
        arm_cmd.request.value = false;
        ROS_INFO("[node] try to disarm the drone");
    }
    arming_client.call(arm_cmd);
}
bool DroneStatus::setMode() {
    if(battery_is_set && current_battery.cell_voltage[0] < low_battery) {
        ROS_WARN("[node] low voltage detected, landing...");
        mavros_msgs::SetMode land_set_mode; 
        land_set_mode.request.custom_mode = "LAND";
        if (set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent){
            ROS_INFO("[node] set LAND successfully");
            return true;
        }
        return false;
    }
    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
        ROS_INFO("[node] set OFFBOARD successfully");
        return true;
    }
    ROS_INFO("[node] set OFFBOARD fail");
    return false;
} // 设置为offboard mode
    // 检测无人机的状态
void DroneStatus::checkMode() {
    if (current_state.armed && current_state.mode == "OFFBOARD") {
        setStatus(_ARMED_);
    }else if (current_state.mode == "LAND") {
        ROS_INFO("[node] LAND is triggered");
        setStatus(_LAND_);
    }else {
        
    }
}
void DroneStatus::resetExpectedPosture() { 
    expected_posture.header.frame_id = "map";
    expected_posture.position.x = 0;
    expected_posture.position.y = 0;
    expected_posture.position.z = 1.0; // 默认的位置坐标
    expected_posture.velocity.x = 0;
    expected_posture.velocity.y = 0;
    expected_posture.velocity.z = 0;
    expected_posture.acceleration_or_force.x = 0.0;
    expected_posture.acceleration_or_force.y = 0.0;
    expected_posture.acceleration_or_force.z = 0.0;
    //expected_posture.yaw = 0.0;
}
void DroneStatus::update() {
    if (!current_state.connected) {
        ROS_WARN("Drone is not Connected");
        return;
    }
    checkMode();
    if (status == _FAILSAFE_) {
        ROS_ERROR("[CRTICAL] Drone IN FAILSAFE");
        return; // 不再执行后续的操作
    }
    if (need_rc){

    }else {
        if (current_state.mode != "OFFBOARD") {
            ROS_INFO("[node] set mode to OFFBOARD");
            setMode();
        }
        if (!current_state.armed) {
            ROS_INFO("[node] arm the Drone");
            setArm();
        }
        uint8_t route_status = detect_the_obscale();
        if (route_status == _EXPECTED_ROUTE_IS_EMPTY_ || route_status == _EXPECTED_ROUTE_IS_CLEAR_){

        }else if (route_status == _EXPECTED_ROUTE_IS_BLOCKED_ || route_status == _DRONE_AROUND_IS_NOT_CLEAR_) {
            // 清零 不要执行路线
            theRoute.clear();
        }else {

        }
        execute_the_route();
        local_pos_pub.publish(expected_posture);
    }
}

// 检测障碍物
// 如果检测到了障碍物 请求重新规划路线 或者停止运动
uint8_t DroneStatus::detect_the_obscale(){
    if (this->map == NULL) {
        ROS_ERROR("[node] MAP is NULL ptr, can not detect the obscale");
        return _MAP_IS_NULL_;
    }

    if (GLX_SIZE < 0 || GLY_SIZE < 0 || GLZ_SIZE < 0){
        ROS_ERROR("[node] MAP size is less than zero");
        return _SIZE_IS_LESS_THAN_ZERO_;
    }

    if (gl_xl < -1e9 || gl_yl < -1e9 | gl_zl < -1e9) {
        ROS_ERROR("[node] MAP lower Bound is not initialized");
        return _BOUND_IS_NOT_INITIALIZED_;
    }

    if (resolution <= 0.0 || inv_resolution <= 0.0){
        ROS_ERROR("[node] RESOLUTION is less than zero");
        return _RESOLUTION_IS_LESS_THAN_ZERO_;
    }

    if (theRoute.size() <= 0) {
        // 没有新的路径
        return _EXPECTED_ROUTE_IS_EMPTY_;
    }

    int GLYZ_SIZE = GLZ_SIZE * GLY_SIZE;
    Eigen::Vector3d theCurrentPt(currect_posture.pose.position.x,
                                                                currect_posture.pose.position.y,
                                                                currect_posture.pose.position.z);

    int dx_x = static_cast<int>( (theCurrentPt(0) - gl_xl) * inv_resolution);
    int dx_y = static_cast<int>( (theCurrentPt(1) - gl_yl) * inv_resolution);
    int dx_z = static_cast<int>( (theCurrentPt(2) - gl_zl) * inv_resolution);
    for (int ii = dx_x - expand_range/2; ii < dx_x + expand_range/2; ii++) {
        if (ii < 0 || ii >= GLX_SIZE) break;
        for (int jj = dx_y - expand_range/2; jj < dx_y + expand_range/2; jj++) {
            if (jj < 0 || jj >= GLY_SIZE) break;
            for (int kk = dx_z - expand_range/2; kk < dx_z + expand_range/2; kk++) {
                if (kk < 0 || kk >= GLZ_SIZE) break;
                bool observed_obstacle = map[ii * GLYZ_SIZE + jj * GLZ_SIZE + kk] == isObs;
                if (observed_obstacle) {
                    ROS_ERROR("[node] Obstacle is detected around the drone");
                    return _DRONE_AROUND_IS_NOT_CLEAR_;
                }
            }
        }
    }

    for (int i = 0; i < theRoute.size(); i++) {
        Eigen::Vector3d theRoutePt = theRoute[i];
        Eigen::Vector3d dist = (theRoutePt - theCurrentPt);
        if (sqrt(dist(0) * dist(0) + dist(1) * dist(1) + dist(2) * dist(2)) > detect_range)
            break;

        int idx_x = static_cast<int>( (theRoutePt(0) - gl_xl) * inv_resolution);
        int idx_y = static_cast<int>( (theRoutePt(1) - gl_yl) * inv_resolution);
        int idx_z = static_cast<int>( (theRoutePt(2) - gl_zl) * inv_resolution);
        
        for (int ii = idx_x - expand_range; ii < idx_x + expand_range; ii++) {
            if (ii < 0 || ii >= GLX_SIZE) 
                break;
            for (int jj = idx_y - expand_range; jj < idx_y + expand_range; jj++) {
                if (jj < 0 || jj >= GLY_SIZE) 
                    break;
                for (int kk = idx_z - expand_range; kk < idx_z + expand_range; kk++) {
                    if (kk < 0 || kk >= GLZ_SIZE)
                        break;
                    bool observed_obstacle = map[ii * GLYZ_SIZE + jj * GLZ_SIZE + kk] < isObs;
                    if (!observed_obstacle) {
                        ROS_ERROR("[node] Obstacle is detected");
                        return _EXPECTED_ROUTE_IS_BLOCKED_;
                    }
                }
            }
        }
    }
    return _EXPECTED_ROUTE_IS_CLEAR_;
}

// 执行theRoute 中的每个路径点（同时，检查每个路径点上的障碍物）
void DroneStatus::execute_the_route(){
    if (theRoute.size() == 0) {
        expected_posture.type_mask = 0B111111111000;
        expected_posture.yaw_rate = 0;
        expected_posture.velocity.x = 0;
        expected_posture.velocity.y = 0;
        expected_posture.velocity.z = 0;
        expected_posture.acceleration_or_force.x = 0;
        expected_posture.acceleration_or_force.y = 0;
        expected_posture.acceleration_or_force.z = 0;
        yaw_diff = 0;
        yaw_int = 0;
        return; // 没有路线可以执行
    }

    double xx = currect_posture.pose.position.x,
                    yy = currect_posture.pose.position.y,
                    zz = currect_posture.pose.position.z;
    Eigen::Vector3d pos = theRoute[0];
    pos -= Eigen::Vector3d(xx, yy, zz);
    if (pos.norm() <= range_threshold) {
        // 到达路标了
        ROS_INFO("[node] Next WayPoint");
        theRoute.erase(theRoute.begin() + 0);
    }else {
        ROS_INFO("[node] Executing WayPoint");
    }

    if (theRoute.size() == 0) {
        ROS_INFO("[node] Will arrive the destination");
    }

    if (theRoute.size() >= 3){
        // Enable A
        // 取出第一个位置点 和第二个位置点 计算角度差异
        Eigen::Vector3d att1 = Eigen::Vector3d(xx, yy, zz),
         att2 = theRoute[1], 
         att3 = theRoute[2];
        Eigen::Vector3d delta_att = (att2 - att1);
        double delta_x = delta_att(0), delta_y = delta_att(1), delta_z = delta_att(2);
        double delta_azi = atan2(delta_x, delta_y);
        // 得到角度
        ROS_INFO("[node] Changing Position \n Expect Location x %f y %f z %f, Current Location x %f y %f z %f", 
            att2(0), att2(1), att2(2), xx, yy, zz);
        // 设置加速度 速度 以及 坐标点
        double px = att2(0), py = att2(1), pz = att2(2);
        double vx = att2(0) - att1(0), vy = att2(1) - att1(1), vz = att2(2) - att1(2);
        double norm_v = sqrt(vx * vx + vy * vy + vz * vz);
        double ax = att3(0) - 2 * att2(0) + att1(0), ay = att3(1) - 2 * att2(1) + att1(1), az = att3(2) - 2 * att2(2) + att1(2);
        double norm_a = sqrt(ax * ax + ay * ay + az * az);
        double expected_yaw = atan2(vy, vx);
        double control_yaw_rate = 0.0;
        PID(current_yaw, expected_yaw, control_yaw_rate,
         yaw_diff, yaw_int,
         yaw_p, yaw_i, yaw_d);
        ROS_INFO("[node] Current Yaw %f Expected Yaw %f", current_yaw, expected_yaw);

        if (norm_v < velocity_drone && norm_a < acc_drone && norm_v > 1e-6) {
            this->expected_posture.type_mask = 0B011000000000;
            this->expected_posture.position.x = px;
            this->expected_posture.position.y = py;
            this->expected_posture.position.z = pz;
            this->expected_posture.velocity.x = vx;
            this->expected_posture.velocity.y = vy;
            this->expected_posture.velocity.z = vz;
            this->expected_posture.acceleration_or_force.x = ax;
            this->expected_posture.acceleration_or_force.y = ay;
            this->expected_posture.acceleration_or_force.z = az;
            this->expected_posture.yaw_rate = control_yaw_rate;
            ROS_INFO("[node] Currenct Velocity  norm %f vx %f vy %f vz %f", 
                norm_v, expected_posture.velocity.x, expected_posture.velocity.y, expected_posture.velocity.z);
        }else{
            this->expected_posture.type_mask = 0B011111111000;
            this->expected_posture.position.x = theRoute[0](0);
            this->expected_posture.position.y = theRoute[0](1);
            this->expected_posture.position.z = theRoute[0](2);
            this->expected_posture.yaw_rate = control_yaw_rate;
        }
    }else {
        this->expected_posture.type_mask = 0B111111111000;
        this->expected_posture.position.x = theRoute[0](0);
        this->expected_posture.position.y = theRoute[0](1);
        this->expected_posture.position.z = theRoute[0](2);
    }
}

void getStateCallBack(const mavros_msgs::State::ConstPtr & state) {
    droneStatus->setState(*state);
}

void getBatteryCallBack(const sensor_msgs::BatteryState::ConstPtr & battery){
    droneStatus->setBattery(*battery);
}
