#ifndef MAIN_CPP_LINKAGES_H
#define MAIN_CPP_LINKAGES_H

#include <vector>
#include "eigen-3.4.0/Eigen/Eigen"
#include "LMsolver.h"
#include <glm/gtc/matrix_transform.hpp>

using namespace std;
using namespace Eigen;

#ifndef M_PI
#define M_PI 3.1415926535897932384626433
#endif

class Linkages{
public:
    Linkages();
    ~Linkages();

public:
    int     linksCase;  // type of linkages
    int     linksNum;   // number of links/parts

    vector<vector<Vector3d>> verListVec;    // store link mesh vert
    vector<vector<Vector3d>> norListVec;    // store link mesh normal
    vector<vector<Vector3i>> triListVec;    // store link mesh triangle

    double motionSpeed; //motion Speed

    // local affine matrix for a link is: [R(rotateAngle,rotateAxis),transPosition; 0,0,0,1]
    Vector3d            rotateAxis;
    vector<double>      rotateAngle;
    vector<Vector2d>    transPosition;

    int groundLinkID;
    int driverLinkID;
    double driverAngle;

    int                 rJointNum;
    vector<Vector2i>    jointPairLinkID;        // <i,j> is a R joint
    vector<Vector2d>    jointPairLocalPos1;     // joint local pos P_i in link L_i
    vector<Vector2d>    jointPairLocalPos2;     // joint local pos P_j in link L_j

    int         particleLinkID;                 // the link_ID of the link that end-effector point is on
    Vector3d    particleLinkPos;                // the local pos of the end-effector
    vector<Vector3d> particleCurve;             // output curve
    vector<float> particleList;                 // output curve save as a List

    LMSlover lmSlover;                          //  solver
    VectorXd lm_x;                              //  parameters of solver

private:
    int circleFrames;                           // frame = 720, for a whole motion
    vector<double> depth;                       // depth for each link
    vector<Vector3d> tangentPitchCurve;         // tangent of Curve
    vector<Vector3d> normalPitchCurve;          // normals of Curve


public:
    // Initialize Linkage
    void InitLinkages(int caseID, vector<vector<float>> &verList, vector<vector<unsigned >> &triList);
    void CrankRocker(double width, double thickness, Matrix4d &depthM);
    void DragLink(double width, double thickness, Matrix4d &depthM);    
    void DoubleRocker(double width, double thickness, Matrix4d &depthM);
    void Hoecken(double width, double thickness, Matrix4d &depthM);

    // Compute Linkage Motion
    void InitLMSolver();
    void ForwardKinematics(int caseID);
    double InputDriverAngle(int frame);
    void SolveLinkages();
    void UpdateLocalMatrix(vector<glm::mat4> &localM);

private:
    // Create a straight line link mesh with depth
    void CreateLinkMeshwithDepth(double length, double width, double thickness, int linkDepth, Matrix4d &depthM);

    // create a straight line link mesh
    void CreateLinkMesh(vector<Vector3d> &verList, vector<Vector3d> &norList, vector<Vector3i> &triList,
                        double length, double width, double thickness);

    void TransformMesh(vector<Vector3d> &verList, Matrix4d transM);

    // create a triangle shape link mesh
    void CreateTriangleMesh(vector<Vector3d> &verList, vector<Vector3d> &norList, vector<Vector3i> &triList,
                            Vector2d P1, Vector2d P2, Vector2d P3, double thickness);

    void CreateCylinder(vector<Vector3d> &verList, vector<Vector3d> &norList, vector<Vector3i> &triList,
                        Vector3d p1, Vector3d p2, double r);

    void CreateCurveMesh(vector<Vector3d> &verList, vector<Vector3d> &norList, vector<Vector3i> &triList, double rTube);
};


#endif //MAIN_CPP_LINKAGES_H
