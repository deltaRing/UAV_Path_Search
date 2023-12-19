#ifndef _RRT_H_
#define _RRT_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class RRTstarPreparatory{	
	private:

	protected:
		uint8_t * data = NULL; // 记录观测次数
		// 每次都 + 2
		// 更新的时候 每次 - 1
		// 观测达到一定次数的时候 直接不再观测 转为1
		uint8_t confirmObservedFrame = 8;
		uint8_t isObs = 255; // 如果是这个数字 是障碍物

		int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
		int GLXYZ_SIZE, GLYZ_SIZE;

		double resolution, inv_resolution;
		double gl_xl, gl_yl, gl_zl;
		double gl_xu, gl_yu, gl_zu;	

	public:
		RRTstarPreparatory(){};
		~RRTstarPreparatory(){ if (data != NULL) delete [] data; };

		Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index);
		Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt);
		void initGridMap(double _resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id);
		void setObs(const double coord_x, const double coord_y, const double coord_z);
		void updateObs(const int x, const int y, const int z); // 更新观测
		void increaseObs(const double coord_x, const double coord_y, const double coord_z);
		void resetObs() { memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t)); }
		bool isObsFree(const double coord_x, const double coord_y, const double coord_z,
			 const double extend_x, const double extend_y, const double extend_z);
		int getMapSizeX() { return GLX_SIZE; }
		int getMapSizeY() { return GLY_SIZE; }
		int getMapSizeZ() { return GLZ_SIZE; }
		double getMapLowerBoundX() { return gl_xl; }
		double getMapLowerBoundY() { return gl_yl; }
		double getMapLowerBoundZ() { return gl_zl; }
		uint8_t getObsNum() { return isObs; }
		void setConfirmObservedFrame(int number) { confirmObservedFrame = number; }
		uint8_t * getMap() { return data; }
		
		Eigen::Vector3d coordRounding(const Eigen::Vector3d & coord);
};


#endif
