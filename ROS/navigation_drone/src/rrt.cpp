#include <rrt.h>

using namespace std;
using namespace Eigen;

// 初始化地图
// resolution_ = 0.2
// 路径位置
// global_xyz_l
// global_xyz_u
void RRTstarPreparatory::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id) {   
	gl_xl = global_xyz_l(0);
	gl_yl = global_xyz_l(1);
	gl_zl = global_xyz_l(2);

	gl_xu = global_xyz_u(0);
	gl_yu = global_xyz_u(1);
	gl_zu = global_xyz_u(2);
    
	GLX_SIZE = max_x_id;
	GLY_SIZE = max_y_id;
	GLZ_SIZE = max_z_id;
	GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
	GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

	resolution = _resolution;
	inv_resolution = 1.0 / _resolution;    

	data = new uint8_t[GLXYZ_SIZE];
	memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
}

// 更新观测
void RRTstarPreparatory::updateObs(const int x, const int y, const int z) {
	if (data[x * GLYZ_SIZE + y * GLZ_SIZE + z] == isObs) {
		return;
	}
	else if (data[x * GLYZ_SIZE + y * GLZ_SIZE + z] > confirmObservedFrame) {
		data[x * GLYZ_SIZE + y * GLZ_SIZE + z] = isObs;
	}
	else if (data[x * GLYZ_SIZE + y * GLZ_SIZE + z] > 0) {
		data[x * GLYZ_SIZE + y * GLZ_SIZE + z] -= 1;
	}
	else {
		data[x * GLYZ_SIZE + y * GLZ_SIZE + z] = 0;
		return; // 已经是0了
	}
}

// 直接置1
void RRTstarPreparatory::setObs(const double coord_x, const double coord_y, const double coord_z)
{   
	if (isnan(coord_x) || isnan(coord_y) || isnan(coord_z)) {
		ROS_ASSERT("[ERROR] Coordinate of NAN is detected!");
		return;
	}
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      
    
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = isObs;
}

// 增加观测
void RRTstarPreparatory::increaseObs(const double coord_x, const double coord_y, const double coord_z) {
	if (isnan(coord_x) || isnan(coord_y) || isnan(coord_z)) {
		ROS_ASSERT("[ERROR] Coordinate of NAN is detected!");
		return;
	}

	 if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      

	if (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == isObs) return; // 已经是障碍物了

	data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] += 2;
}

// 查看地图是否为0
bool RRTstarPreparatory::isObsFree(const double coord_x, const double coord_y, const double coord_z, 
			 const double extend_x, const double extend_y, const double extend_z)
{
    Vector3d pt;
    Vector3i idx;
    
    pt(0) = coord_x;
    pt(1) = coord_y;
    pt(2) = coord_z;
    idx = coord2gridIndex(pt);

    int idx_x = idx(0);
    int idx_y = idx(1);
    int idx_z = idx(2);

	int x_ob = extend_x, y_ob = extend_y, z_ob = extend_z;
	bool observed_obstacle = (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && 
														idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE);
														
	if (!observed_obstacle) 
	return observed_obstacle;

	for (int ii = idx_x - x_ob; ii < idx_x + x_ob; ii++) {
		if (ii < 0 || ii >= GLX_SIZE) 
			break;
		if (!observed_obstacle)
		 	return false;
		for (int jj = idx_y - y_ob; jj < idx_y + y_ob; jj++) {
			if (jj < 0 || jj >= GLY_SIZE) 
				break;
			if (!observed_obstacle) 
				return false;
			for (int kk = idx_z - z_ob; kk < idx_z + z_ob; kk++) {
				if (kk < 0 || kk >= GLZ_SIZE)
				 	break;
				if (!observed_obstacle) 
				 	return false;
				observed_obstacle = data[ii * GLYZ_SIZE + jj * GLZ_SIZE + kk] < isObs;
			}
		}
	}

	return observed_obstacle;
}

// grid索引转为坐标索引
Vector3d RRTstarPreparatory::gridIndex2coord(const Vector3i & index) {
	Vector3d pt;

	pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
	pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
	pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

	return pt;
}

// 
Vector3i RRTstarPreparatory::coord2gridIndex(const Vector3d & pt) {
	Vector3i idx;
	idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
	return idx;
}

Eigen::Vector3d RRTstarPreparatory::coordRounding(const Eigen::Vector3d & coord){
	return gridIndex2coord(coord2gridIndex(coord));
}


