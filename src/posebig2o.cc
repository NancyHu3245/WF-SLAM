#include "posebig2o.h"
#include <iostream>

#include "Thirdparty/g2o/g2o/core/factory.h"
#include "Thirdparty/g2o/g2o/stuff/macros.h"


namespace g2o {
using namespace std;

//vertex
    VertexSDynamicWeight::VertexSDynamicWeight():BaseVertex<4, Vector4d>()
    {
    }

    bool VertexSDynamicWeight::read(std::istream& is){
		Vector4d lv;
		for (int i = 0; i < 4; i++)
        is >> _estimate[i];
        return true;
    }

    bool VertexSDynamicWeight::write(std::ostream& os) const{
		Vector4d lv=_estimate;
		for (int i = 0; i < 4; i++)
        os<<lv[i]<<" ";
        return os.good();
    }



//edge
bool EdgeStereoSE3PoseDynamicWeght::write(std::ostream& os) const{
    for (int i=0; i<=3; i++){
        os << measurement()[i] << " ";
    }

    for (int i=0; i<=2; i++)
        for (int j=i; j<=2; j++){
        os << " " <<  information()(i,j);
        }
    return os.good();
    }

bool EdgeStereoSE3PoseDynamicWeght::read(std::istream& is){
    for (int i=0; i<=3; i++){
        is >> _measurement[i];
    }
    for (int i=0; i<=2; i++)
        for (int j=i; j<=2; j++) {//Matrix<double, D, D>
        is >> information()(i,j);
        if (i!=j)
            information()(j,i)=information()(i,j);
        }
    return true;           
}

 Vector3d EdgeStereoSE3PoseDynamicWeght::cam_project(const Vector3d & trans_xyz, const float &bf) const{
    const float invz = 1.0f/trans_xyz[2];
    Vector3d res;
    res[0] = trans_xyz[0]*invz*fx + cx;
    res[1] = trans_xyz[1]*invz*fy + cy;
    res[2] = res[0] - bf*invz;
    return res;
    }

 void EdgeStereoSE3PoseDynamicWeght::linearizeOplus() 
 {
	 VertexSDynamicWeight * vj = static_cast<VertexSDynamicWeight *>(_vertices[1]);
	 Matrix4d w = vj->estimate();
     Vector3d xyz;
     for(int i=0;i<3;i++)
         xyz[i]=w[i];
	 double wi = w[3];
	 VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
	 SE3Quat T(vi->estimate());
	 Vector3d xyz_trans = T.map(xyz);//经过位姿变换之后的3D点

	 double x = xyz_trans[0];
	 double y = xyz_trans[1];
	 double invz = 1.0 / xyz_trans[2];
	 double invz_2 = invz * invz;

	 _jacobianOplusXi(0, 0) = wi*( x * y*invz_2 *fx);
	 _jacobianOplusXi(0, 1) = wi*(-(1 + (x*x*invz_2)) *fx);
	 _jacobianOplusXi(0, 2) = wi * (y * invz *fx );
	 _jacobianOplusXi(0, 3) = wi * (-invz * fx);
	 _jacobianOplusXi(0, 4) = 0;
	 _jacobianOplusXi(0, 5) = wi * (x * invz_2 *fx);

	 _jacobianOplusXi(1, 0) = wi * ((1 + y * y*invz_2) *fy);
	 _jacobianOplusXi(1, 1) = wi * (-x * y*invz_2 *fy);
	 _jacobianOplusXi(1, 2) = wi * (-x * invz *fy);
	 _jacobianOplusXi(1, 3) = 0;
	 _jacobianOplusXi(1, 4) = wi * (-invz * fy);
	 _jacobianOplusXi(1, 5) = wi * (y * invz_2 *fy);

	 _jacobianOplusXi(2, 0) = _jacobianOplusXi(0, 0) - wi * (bf * y*invz_2);
	 _jacobianOplusXi(2, 1) = _jacobianOplusXi(0, 1) + wi * (bf * x*invz_2);
	 _jacobianOplusXi(2, 2) = _jacobianOplusXi(0, 2);
	 _jacobianOplusXi(2, 3) = _jacobianOplusXi(0, 3);
	 _jacobianOplusXi(2, 4) = 0;
	 _jacobianOplusXi(2, 5) = _jacobianOplusXi(0, 5) - wi * (bf * invz_2);


	 Vector3d usobs(_measurement);//误差相对权重的导数
	 _jacobianOplusXj(0, 0) = usobs[0]-(y*invz*fx + cx);
	 _jacobianOplusXj(1, 0) = usobs[1] -(y*invz*fy + cy);
	 _jacobianOplusXj(2, 0) = usobs[2]-(x*invz*fx + cx - bf * invz);
//_jacobianOplusXi为_error对VertexSE3Expmap的导数，_jacobianOplusXj是_error对VertexSE3Expmap的导数
 }
}