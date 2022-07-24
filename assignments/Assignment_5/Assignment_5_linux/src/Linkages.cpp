
#include "Linkages.h"
#include "eigen-3.4.0/unsupported/Eigen/NonLinearOptimization"
#include <iostream>

///=========================================================================================///
///                                       Initialization
///=========================================================================================///

Linkages::Linkages()
{
    linksCase = 1;
    linksNum = 4;
    rotateAxis = Vector3d(0, 0, 1);
    circleFrames = 720;
    motionSpeed = 1.0;
}

Linkages::~Linkages()
{
}

///=========================================================================================///
///                                     Initialize Linkage
///=========================================================================================///

void Linkages::InitLinkages(int caseID, vector<vector<float>> &verList, vector<vector<unsigned>> &triList)
{
    linksCase = caseID;

    // clear all
    verListVec.clear();
    triListVec.clear();
    norListVec.clear();
    rotateAngle.clear();
    transPosition.clear();
    jointPairLinkID.clear();
    jointPairLocalPos2.clear();
    jointPairLocalPos1.clear();
    particleCurve.clear();
    depth.clear();
    Matrix4d depthM;
    depthM.setIdentity();

    linksNum = 4;
    particleLinkID = 3;

    double width = 0.2;
    double thickness = width / 2;
    double length;

    // init linkages
    if (caseID == 1)
        CrankRocker(width, thickness, depthM);
    else if (caseID == 2)
        DragLink(width, thickness, depthM);
    else if (caseID == 3)
        Hoecken(width, thickness, depthM);
    else if (caseID == 4)
        DoubleRocker(width, thickness, depthM);

    rJointNum = jointPairLinkID.size();

    // Create joint pin cylinder
    double tol = 0.07;
    for (int i = 0; i < jointPairLinkID.size(); i++)
    {
        vector<Vector3d> verL, norL;
        vector<Vector3i> triL;
        double r = 0.06;

        double depth1 = depth[jointPairLinkID[i].x()];
        double depth2 = depth[jointPairLinkID[i].y()];
        double depMin = depth1 < depth2 ? depth1 : depth2;
        double depMax = depth1 > depth2 ? depth1 : depth2;
        CreateCylinder(verL, norL, triL, Vector3d(jointPairLocalPos1[i].x(), jointPairLocalPos1[i].y(), depMin - tol),
                       Vector3d(jointPairLocalPos1[i].x(), jointPairLocalPos1[i].y(), depMax + tol), r);
        verListVec.push_back(verL);
        triListVec.push_back(triL);
        norListVec.push_back(norL);
    }

    // Compute linkage motion
    InitLMSolver();
    ForwardKinematics(caseID);

    // for rendering
    verList.clear();
    triList.clear();
    for (int i = 0; i < linksNum + rJointNum + 1; i++)
    {
        vector<float> verL;
        vector<unsigned> triL;
        for (int j = 0; j < verListVec[i].size(); j++)
        {
            verL.push_back(verListVec[i][j].x());
            verL.push_back(verListVec[i][j].y());
            verL.push_back(verListVec[i][j].z());

            verL.push_back(norListVec[i][j].x());
            verL.push_back(norListVec[i][j].y());
            verL.push_back(norListVec[i][j].z());
        }
        for (auto tri : triListVec[i])
        {
            triL.push_back(tri.x());
            triL.push_back(tri.y());
            triL.push_back(tri.z());
        }
        verList.push_back(verL);
        triList.push_back(triL);
    }
}

void Linkages::CrankRocker(double width, double thickness, Matrix4d &depthM)
{
    double length;

    // Ground link
    groundLinkID = 0;
    rotateAngle.push_back(0);
    transPosition.emplace_back(-1, 0);
    length = 2;
    CreateLinkMeshwithDepth(length, width, thickness, 0, depthM);

    // Driver (Left) link
    driverLinkID = 1;
    rotateAngle.push_back(M_PI / 2);
    driverAngle = M_PI / 2;
    transPosition.emplace_back(-1, 0);
    length = 1;
    CreateLinkMeshwithDepth(length, width, thickness, 1, depthM);

    // Right link
    rotateAngle.push_back(M_PI / 2);
    transPosition.emplace_back(1, 0);
    length = 2;
    CreateLinkMeshwithDepth(length, width, thickness, 1, depthM);

    // Top link
    rotateAngle.push_back(0);
    transPosition.emplace_back(-1, 1);
    length = 3;
    CreateLinkMeshwithDepth(length, width, thickness, 2, depthM);

    // End-effector position
    particleLinkPos = Vector3d(3, 0, 0);

    // Joints
    jointPairLinkID.emplace_back(0, 1);
    jointPairLocalPos1.emplace_back(0, 0);
    jointPairLocalPos2.emplace_back(0, 0);

    jointPairLinkID.emplace_back(0, 2);
    jointPairLocalPos1.emplace_back(2, 0);
    jointPairLocalPos2.emplace_back(0, 0);

    jointPairLinkID.emplace_back(1, 3);
    jointPairLocalPos1.emplace_back(1, 0);
    jointPairLocalPos2.emplace_back(0, 0);

    jointPairLinkID.emplace_back(2, 3);
    jointPairLocalPos1.emplace_back(2, 0);
    jointPairLocalPos2.emplace_back(2, 0);
}

///=========================================================================================///
///
///                       Functions to be filled in for Assignment 5
///
///           IMPORTANT: you ONLY need to work on functions with TODO in this section
///
///=========================================================================================///

//// TODO: fill this function to design Drag-Link
void Linkages::DragLink(double width, double thickness, Matrix4d &depthM)
{
    double length;

    // Ground link
    groundLinkID = 0;
    rotateAngle.push_back(0);
    transPosition.emplace_back(-1, 0);
    length = 1;
    CreateLinkMeshwithDepth(length, width, thickness, 0, depthM);

    // Driver (Left) link
    driverLinkID = 1;
    rotateAngle.push_back(M_PI / 2);
    driverAngle = M_PI / 2;
    transPosition.emplace_back(-1, 0);
    length = 1.5;
    CreateLinkMeshwithDepth(length, width, thickness, 1, depthM);

    // Right link
    rotateAngle.push_back(M_PI / 2);
    transPosition.emplace_back(0, 0);
    length = 2;
    CreateLinkMeshwithDepth(length, width, thickness, -1, depthM);

    // Top link
    rotateAngle.push_back(0);
    transPosition.emplace_back(-1, 1);
    length = 3;
    CreateLinkMeshwithDepth(length, width, thickness, 0, depthM);

    // End-effector position
    particleLinkPos = Vector3d(3, 0, 0);

    //// TODO: fill the link joints information
    jointPairLinkID.emplace_back(0, 1);
    jointPairLocalPos1.emplace_back(0, 0);
    jointPairLocalPos2.emplace_back(0, 0);

    jointPairLinkID.emplace_back(0, 2);
    jointPairLocalPos1.emplace_back(1, 0);
    jointPairLocalPos2.emplace_back(0, 0);

    jointPairLinkID.emplace_back(1, 3);
    jointPairLocalPos1.emplace_back(1.5, 0);
    jointPairLocalPos2.emplace_back(0, 0);

    jointPairLinkID.emplace_back(2, 3);
    jointPairLocalPos1.emplace_back(2, 0);
    jointPairLocalPos2.emplace_back(2, 0);
}

//// TODO: fill this function to design Hoecken
void Linkages::Hoecken(double width, double thickness, Matrix4d &depthM)
{
    double length;

    // Ground link
    groundLinkID = 0;
    rotateAngle.push_back(0);
    transPosition.emplace_back(-1, 0);
    length = 2;
    CreateLinkMeshwithDepth(length, width, thickness, 0, depthM);

    // Driver (Left) link
    driverLinkID = 1;
    rotateAngle.push_back(M_PI / 2);
    driverAngle = M_PI / 2;
    transPosition.emplace_back(-1, 0);
    length = 1;
    CreateLinkMeshwithDepth(length, width, thickness, 1, depthM);

    // Right link
    rotateAngle.push_back(M_PI / 2);
    transPosition.emplace_back(1, 0);
    length = 2.5;
    CreateLinkMeshwithDepth(length, width, thickness, 1, depthM);

    // Top link
    rotateAngle.push_back(0);
    transPosition.emplace_back(-1, 1);
    length = 5;
    CreateLinkMeshwithDepth(length, width, thickness, 2, depthM);

    // End-effector position
    particleLinkPos = Vector3d(5, 0, 0);

    //// TODO: fill the link joints information
    jointPairLinkID.emplace_back(0, 1);
    jointPairLocalPos1.emplace_back(0, 0);
    jointPairLocalPos2.emplace_back(0, 0);

    jointPairLinkID.emplace_back(0, 2);
    jointPairLocalPos1.emplace_back(2, 0);
    jointPairLocalPos2.emplace_back(0, 0);

    jointPairLinkID.emplace_back(1, 3);
    jointPairLocalPos1.emplace_back(1, 0);
    jointPairLocalPos2.emplace_back(0, 0);

    jointPairLinkID.emplace_back(2, 3);
    jointPairLocalPos1.emplace_back(2.5, 0);
    jointPairLocalPos2.emplace_back(2.5, 0);
}

//// TODO: fill this function to design DoubleRocker
void Linkages::DoubleRocker(double width, double thickness, Matrix4d &depthM)
{
    double length;

    // Ground link
    groundLinkID = 0;
    rotateAngle.push_back(0);
    transPosition.emplace_back(-1, 0);
    length = 1.2;
    CreateLinkMeshwithDepth(length, width, thickness, 0, depthM);

    // Driver (Left) link
    driverLinkID = 1;
    rotateAngle.push_back(M_PI / 2);
    driverAngle = M_PI / 2;
    transPosition.emplace_back(-1, 0);
    length = 2;
    CreateLinkMeshwithDepth(length, width, thickness, 1, depthM);

    // Right link
    rotateAngle.push_back(M_PI / 2);
    transPosition.emplace_back(0.2, 0);
    length = 1.4;
    CreateLinkMeshwithDepth(length, width, thickness, -1, depthM);

    // Top link
    rotateAngle.push_back(0);
    transPosition.emplace_back(-1, 2);
    length = 2;
    CreateLinkMeshwithDepth(length, width, thickness, 0, depthM);

    // End-effector position
    particleLinkPos = Vector3d(2, 0, 0);

    //// TODO: fill the link joints information
    jointPairLinkID.emplace_back(0, 1);
    jointPairLocalPos1.emplace_back(0, 0);
    jointPairLocalPos2.emplace_back(0, 0);

    jointPairLinkID.emplace_back(0, 2);
    jointPairLocalPos1.emplace_back(1.2, 0);
    jointPairLocalPos2.emplace_back(0, 0);

    jointPairLinkID.emplace_back(1, 3);
    jointPairLocalPos1.emplace_back(2, 0);
    jointPairLocalPos2.emplace_back(0, 0);

    jointPairLinkID.emplace_back(2, 3);
    jointPairLocalPos1.emplace_back(1.4, 0);
    jointPairLocalPos2.emplace_back(1, 0);
}

///=========================================================================================///
///                                   Compute Linkage Motion
///=========================================================================================///

void Linkages::InitLMSolver()
{
    lmSlover.n = linksNum * 3;
    lmSlover.m = rJointNum * 2 + 3 + 1;
    if (lmSlover.n > lmSlover.m)
        cout << "lack constraints" << endl;
    else if (lmSlover.n < lmSlover.m)
        cout << "over constraints" << endl;

    lmSlover.linksNum = linksNum;
    lmSlover.jointPairLinkID = jointPairLinkID;
    lmSlover.jointPairLocalPos1 = jointPairLocalPos1;
    lmSlover.jointPairLocalPos2 = jointPairLocalPos2;
    lmSlover.groundLinkID = groundLinkID;
    lmSlover.driverLinkID = driverLinkID;
    lmSlover.initRotateAngle = rotateAngle;
    lmSlover.initTransPosition = transPosition;
    lm_x.resize(lmSlover.n);
    for (int i = 0; i < linksNum; i++)
    {
        lm_x(i * 3) = rotateAngle[i];
        lm_x(i * 3 + 1) = transPosition[i].x();
        lm_x(3 * i + 2) = transPosition[i].y();
    }
}

void Linkages::ForwardKinematics(int caseID)
{
    particleList.clear();
    int Times = 1;

    for (int i = 1; i <= circleFrames * Times; i++)
    {
        InputDriverAngle(i);
        SolveLinkages();
    }

    for (auto p : particleCurve)
    {
        particleList.push_back(p.x());
        particleList.push_back(p.y());
        particleList.push_back(p.z());
    }

    // curve Mesh
    vector<Vector3d> verL, norL;
    vector<Vector3i> triL;
    CreateCurveMesh(verL, norL, triL, 0.02);
    verListVec.push_back(verL);
    norListVec.push_back(norL);
    triListVec.push_back(triL);

    // reset
    particleCurve.clear();
    InputDriverAngle(1);
    SolveLinkages();
}

double Linkages::InputDriverAngle(int frame)
{
    if (linksCase == 1 || linksCase == 2 || linksCase == 3)
    {
        driverAngle += 2.0 * M_PI / circleFrames * motionSpeed;
    }

    /// for Double-rocker, input is an oscillatory motion
    /// hint: adjust 'motionSpeed' direction at the appropriate time
    else if (linksCase == 4)
    {
        if (driverAngle < 0)
        {
            motionSpeed = abs(motionSpeed);
        }
        else if (driverAngle > M_PI_2)
        {
            motionSpeed = -abs(motionSpeed);
        }
        driverAngle += M_PI / circleFrames * motionSpeed;
    }

    return driverAngle;
}

void Linkages::SolveLinkages()
{
    lmSlover.driverAngle = driverAngle;
    Eigen::LevenbergMarquardt<LMSlover, double> lm(lmSlover);
    int status = lm.minimize(lm_x);
    for (int i = 0; i < linksNum; i++)
    {
        rotateAngle[i] = lm_x(3 * i);
        transPosition[i].x() = lm_x(3 * i + 1);
        transPosition[i].y() = lm_x(3 * i + 2);
    }

    // update curve
    Matrix4d particleM;
    particleM.setIdentity();
    particleM(0, 0) = cos(rotateAngle[particleLinkID]);
    particleM(0, 1) = -sin(rotateAngle[particleLinkID]);
    particleM(1, 0) = sin(rotateAngle[particleLinkID]);
    particleM(1, 1) = cos(rotateAngle[particleLinkID]);
    particleM(0, 3) = transPosition[particleLinkID].x();
    particleM(1, 3) = transPosition[particleLinkID].y();
    Vector4d particleP;
    particleP << particleLinkPos.x(), particleLinkPos.y(), depth[particleLinkID], 1;
    particleP = particleM * particleP;

    particleCurve.emplace_back(particleP.x(), particleP.y(), particleP.z());
}

void Linkages::UpdateLocalMatrix(vector<glm::mat4> &localM)
{
    localM.clear();

    for (int i = 0; i < linksNum; i++)
    {
        glm::mat4 rotateMatrix = glm::mat4(1.0f);
        rotateMatrix = glm::rotate(rotateMatrix, float(rotateAngle[i]), glm::vec3(0.f, 0.f, 1.f));
        glm::mat4 translateMatrix = glm::mat4(1.0f);
        translateMatrix = glm::translate(translateMatrix, glm::vec3(float(transPosition[i].x()), float(transPosition[i].y()), 0.f));

        glm::mat4 lM;
        lM = translateMatrix * rotateMatrix;
        localM.push_back(lM);
    }

    for (auto &i : jointPairLinkID)
    {
        glm::mat4 lM = localM[i.x()];
        localM.push_back(lM);
    }

    localM.push_back(glm::mat4(1.0f));
}

///=========================================================================================///
///                               Create Linkage Geometry for Rendering
///=========================================================================================///

void Linkages::CreateLinkMeshwithDepth(double length, double width, double thickness, int linkDepth, Matrix4d &depthM)
{
    vector<Vector3d> verL, norL;
    vector<Vector3i> triL;

    CreateLinkMesh(verL, norL, triL, length, width, thickness);

    depthM(2, 3) = linkDepth * thickness;
    depth.push_back(depthM(2, 3));

    TransformMesh(verL, depthM); // move mesh to (0,0,thickness)
    verListVec.push_back(verL);
    norListVec.push_back(norL);
    triListVec.push_back(triL);
}

void Linkages::CreateLinkMesh(vector<Vector3d> &verL, vector<Vector3d> &norL, vector<Vector3i> &triL,
                              double length, double width, double thickness)
{
    int N = 20;
    Vector3d endP1 = Vector3d(0, 0, 0);
    Vector3d endP2 = Vector3d(length, 0, 0);

    Vector3d line;
    line = (endP2 - endP1).normalized();
    Vector3d normL = Vector3d(0, 0, 1);
    Vector3d arcL = line.cross(normL);

    verL.clear();
    triL.clear();
    norL.clear();

    verL.emplace_back(length, 0, thickness / 2);
    norL.push_back(normL);
    double alpha = atan(2 * width / thickness);
    width = width / 2;
    for (int i = 0; i <= N; i++)
    {
        double theta = i * 2 * M_PI / N / 2;
        Vector3d verP = width * (cos(theta) * arcL + sin(theta) * line) + endP2 + Vector3d(0, 0, thickness / 2);
        Vector3d norP = cos(alpha) * (cos(theta) * arcL + sin(theta) * line) + sin(alpha) * normL;
        verL.push_back(verP);
        norL.push_back(norP);
    }
    for (int i = N; i <= 2 * N; i++)
    {
        double theta = i * 2 * M_PI / N / 2;
        Vector3d verP = width * (cos(theta) * arcL + sin(theta) * line) + endP1 + Vector3d(0, 0, thickness / 2);
        Vector3d norP = cos(alpha) * (cos(theta) * arcL + sin(theta) * line) + sin(alpha) * normL;
        verL.push_back(verP);
        norL.push_back(norP);
    }
    verL.emplace_back(0, 0, thickness / 2);
    norL.push_back(normL);

    for (int i = 0; i < 2 * N + 4; i++)
    {
        verL.push_back(verL[i] - Vector3d(0, 0, thickness));
        norL.emplace_back(norL[i].x(), norL[i].y(), -norL[i].z());
    }

    triL.emplace_back(1, 0, 2 * N + 2);
    triL.emplace_back(0, 2 * N + 3, 2 * N + 2);
    triL.emplace_back(0, N + 2, 2 * N + 3);
    triL.emplace_back(0, N + 1, N + 2);
    for (int i = 1; i < N + 1; i++)
        triL.emplace_back(0, i, i + 1);
    for (int i = N + 2; i < 2 * N + 2; i++)
        triL.emplace_back(2 * N + 3, i, i + 1);

    triL.emplace_back(4 * N + 7, 2 * N + 5, 4 * N + 6);
    triL.emplace_back(4 * N + 7, 2 * N + 4, 2 * N + 5);
    triL.emplace_back(4 * N + 7, 3 * N + 5, 2 * N + 4);
    triL.emplace_back(4 * N + 7, 3 * N + 6, 3 * N + 5);
    for (int i = 2 * N + 5; i < 3 * N + 5; i++)
        triL.emplace_back(2 * N + 4, i + 1, i);
    for (int i = 3 * N + 6; i < 4 * N + 6; i++)
        triL.emplace_back(4 * N + 7, i + 1, i);

    for (int i = 1; i < 2 * N + 2; i++)
    {
        triL.emplace_back(i, i + 2 * N + 4, i + 2 * N + 5);
        triL.emplace_back(i, i + 2 * N + 5, i + 1);
    }
    triL.emplace_back(2 * N + 2, 4 * N + 6, 2 * N + 5);
    triL.emplace_back(2 * N + 2, 2 * N + 5, 1);
}

void Linkages::TransformMesh(vector<Vector3d> &verList, Matrix4d transM)
{
    for (auto &v : verList)
    {
        Vector4d vAffine = Vector4d(v.x(), v.y(), v.z(), 1);
        vAffine = transM * vAffine;
        v.x() = vAffine.x();
        v.y() = vAffine.y();
        v.z() = vAffine.z();
    }
}

void Linkages::CreateTriangleMesh(vector<Vector3d> &verList, vector<Vector3d> &norList, vector<Vector3i> &triList,
                                  Vector2d P1, Vector2d P2, Vector2d P3, double thickness)
{
    verList.clear();
    norList.clear();
    triList.clear();

    verList.emplace_back(P1.x(), P1.y(), thickness / 2);
    verList.emplace_back(P2.x(), P2.y(), thickness / 2);
    verList.emplace_back(P3.x(), P3.y(), thickness / 2);
    verList.emplace_back(P1.x(), P1.y(), -thickness / 2);
    verList.emplace_back(P2.x(), P2.y(), -thickness / 2);
    verList.emplace_back(P3.x(), P3.y(), -thickness / 2);

    norList.emplace_back(0, 0, 1);
    norList.emplace_back(0, 0, 1);
    norList.emplace_back(0, 0, 1);
    norList.emplace_back(0, 0, -1);
    norList.emplace_back(0, 0, -1);
    norList.emplace_back(0, 0, -1);

    triList.emplace_back(0, 1, 2);
    triList.emplace_back(3, 5, 4);
    triList.emplace_back(0, 2, 5);
    triList.emplace_back(0, 5, 3);
    triList.emplace_back(0, 3, 4);
    triList.emplace_back(0, 4, 1);
    triList.emplace_back(1, 4, 5);
    triList.emplace_back(1, 5, 2);
}

void Linkages::CreateCylinder(vector<Vector3d> &verList, vector<Vector3d> &norList, vector<Vector3i> &triList,
                              Vector3d p1, Vector3d p2, double r)
{
    verList.clear();
    norList.clear();
    triList.clear();

    Vector3d line = p1 - p2;
    line.normalize();
    Vector3d arc1 = Vector3d(1, 0, 0);
    Vector3d arc2 = line.cross(arc1);

    int N = 20;
    verList.push_back(p1);
    norList.emplace_back(line);
    for (int i = 0; i < N; i++)
    {
        double theta = i * 2.0 * M_PI / N;
        Vector3d vp = (arc1 * cos(theta) + arc2 * sin(theta)) * r + p1;
        verList.push_back(vp);
        norList.emplace_back(line);
    }
    for (int i = 0; i < N; i++)
    {
        double theta = i * 2.0 * M_PI / N;
        Vector3d vp = (arc1 * cos(theta) + arc2 * sin(theta)) * r + p2;
        verList.push_back(vp);
        norList.emplace_back(-line);
    }
    verList.push_back(p2);
    norList.emplace_back(-line);

    for (int i = 1; i < N; i++)
    {
        triList.emplace_back(0, i, i + 1);
        triList.emplace_back(2 * N + 1, N + i + 1, N + i);
        triList.emplace_back(i, N + i, N + i + 1);
        triList.emplace_back(i, i + N + 1, i + 1);
    }
    triList.emplace_back(0, N, 1);
    triList.emplace_back(2 * N + 1, N + 1, 2 * N);
    triList.emplace_back(N, 2 * N, N + 1);
    triList.emplace_back(N, N + 1, 1);
}

void Linkages::CreateCurveMesh(vector<Vector3d> &verList, vector<Vector3d> &norList, vector<Vector3i> &triList, double rTube)
{
    tangentPitchCurve.clear();
    normalPitchCurve.clear();
    Vector3d verCenter = Vector3d(0, 0, 0);
    for (const auto &p : particleCurve)
        verCenter += p;
    verCenter = verCenter / particleCurve.size();
    Vector3d tanCurve, norCurve;
    tangentPitchCurve.clear();
    normalPitchCurve.clear();

    int NSize = particleCurve.size();
    for (int i = 0; i < NSize; i++)
    {
        if (i < NSize - 1)
            tanCurve = (particleCurve[i + 1] - particleCurve[i]).normalized();
        else if (i == NSize - 1)
            tanCurve = (particleCurve[0] - particleCurve[i]).normalized();
        tangentPitchCurve.push_back(tanCurve);

        norCurve = (particleCurve[i] - verCenter);
        norCurve = norCurve - norCurve.dot(tanCurve) * tanCurve;
        norCurve.normalize();
        normalPitchCurve.push_back(norCurve);
    }
    tangentPitchCurve.push_back((particleCurve[0] - particleCurve[NSize - 1]).normalized());
    normalPitchCurve.push_back((particleCurve[NSize - 1] - verCenter).normalized());

    /// verList
    vector<Vector3d> G_verList, G_norList;
    int numM = 30;
    for (int i = 0; i < NSize; i++)
    {
        Vector3d N_1, N_2;
        N_1 = normalPitchCurve[i];
        N_2 = -tangentPitchCurve[i].cross(N_1);

        for (int j = 0; j < numM; j++)
        {
            Vector3d TubeP;
            double theta = j * 2.0 * M_PI / numM;

            TubeP = particleCurve[i] + (rTube) * (N_1 * cos(theta) + N_2 * sin(theta));
            G_verList.push_back(TubeP);
            G_norList.push_back(N_1 * cos(theta) + N_2 * sin(theta));
        }
    }

    /// triList
    vector<Vector3i> G_triList;
    int StartId = 0;
    for (int itr = 0; itr < NSize - 1; itr++)
    {
        for (int j = 0; j < numM - 1; j++)
        {
            G_triList.emplace_back(StartId + j, StartId + numM + j, StartId + numM + 1 + j);
            G_triList.emplace_back(StartId + j, StartId + numM + 1 + j, StartId + 1 + j);
        }
        G_triList.emplace_back(StartId + numM - 1, StartId + 2 * numM - 1, StartId + numM);
        G_triList.emplace_back(StartId + numM - 1, StartId + numM, StartId);
        StartId += numM;
    }

    for (int j = 0; j < numM - 1; j++)
    {
        G_triList.emplace_back(StartId + j, j, j + 1);
        G_triList.emplace_back(StartId + j, j + 1, StartId + j + 1);
    }
    G_triList.emplace_back(StartId + numM - 1, numM - 1, 0);
    G_triList.emplace_back(StartId + numM - 1, 0, StartId);

    verList.clear();
    triList.clear();
    norList.clear();
    verList = G_verList;
    norList = G_norList;
    triList = G_triList;
}
