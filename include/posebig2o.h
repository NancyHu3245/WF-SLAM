/*
Modified by guanhuang (2020)
Add VertexSDynamicWeight()    //identify Vertex for dynamic weight
Add EdgeStereoSE3PoseDynamicWeght()  // Edge　for optimize pose and dynamic weight
*/

#ifndef POSEBIG20_H
#define POSEBIG20_H

#include "Thirdparty/g2o/g2o/core/optimizable_graph.h"

#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
//#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

#include <Eigen/Geometry>

namespace g2o 
{

using namespace Eigen;

typedef Matrix<double, 4, 1> Matrix4d;

 class VertexSDynamicWeight : public BaseVertex<4, Matrix4d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW    
	VertexSDynamicWeight();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setToOriginImpl() {//设置初始估计值
      _estimate<< 0,0,0,1.0;
    }

    virtual void oplusImpl(const double* update)//更新
    {
      Matrix4d v;
      for(int i=0;i<4;i++)
      {
        v[i]=*update;
        update++;
      };
      //Eigen::Map<const Matrix4d> v(update);
      _estimate += v;
    }
};

//观测值维度．类型，连接顶点类型．
class  EdgeStereoSE3PoseDynamicWeght: public  BaseBinaryEdge<3, Vector3d,VertexSE3Expmap, VertexSDynamicWeight>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeStereoSE3PoseDynamicWeght(){}

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    const VertexSDynamicWeight* v2 = static_cast<const VertexSDynamicWeight*>(_vertices[1]);     
    Vector3d obs(_measurement);    //_measurement类型为：typedef E Measurement
     //_error类型为：Matrix<double, D, 1> ErrorVector
	Matrix4d w = v2->estimate();
	Vector3d xyz;
  for(int i=0;i<3;i++)
    xyz[i]=w[i];
    Vector3d temp=obs - cam_project(xyz,bf);
    for(int i=0;i<3;i++)
      {
        auto te=w[3]*temp[i];
        _error[i]=w[3]*temp[i];
        //std<<cout<<_error[i]<<std::endl;
      }
    //_error = w[3]*(obs - cam_project(xyz,bf));    //estimate()返回_estimate，类型为T
  }

  bool isDepthPositive() {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    const VertexSDynamicWeight * v2 = static_cast<VertexSDynamicWeight *>(_vertices[1]);
    	Matrix4d w = v2->estimate();
      Vector3d xyz;
      for(int i=0;i<3;i++)
        xyz[i]=w[i];
    return (v1->estimate().map(xyz))(2)>0.0;
  }


  virtual void linearizeOplus();

  Vector3d cam_project(const Vector3d & trans_xyz, const float &bf) const;

  double fx, fy, cx, cy, bf;
};


}

#endif//POSEBIG20_H