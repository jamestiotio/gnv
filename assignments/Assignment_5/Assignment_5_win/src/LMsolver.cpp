
#include "LMsolver.h"
#include <iostream>

// fvec = f(x) = ( ,fi(x), )
int LMSlover::operator()(const VectorXd &x, VectorXd &fvec) const
{
    fvec.resize(m);
    vector<double> rcf, scf;
    rcf = rotateConstraints(x);
    scf = sliderConstraints(x);
    for(int i=0;i<rcf.size();i++)
        fvec(i) = rcf[i];
    for(int i=0;i<scf.size();i++)
        fvec(rcf.size()+i) = scf[i];

    // ground
    fvec(rcf.size()+scf.size()) = x(3*groundLinkID) - initRotateAngle[groundLinkID];
    fvec(rcf.size()+scf.size()+1) = x(3*groundLinkID+1) - initTransPosition[groundLinkID].x();
    fvec(rcf.size()+scf.size()+2) = x(3*groundLinkID+2) - initTransPosition[groundLinkID].y();

    // driver
    fvec(rcf.size()+scf.size()+3) = x(3*driverLinkID) - driverAngle;

    return 0;
}

// fjac = Jco = (dfi/dxj)
int LMSlover::df(const VectorXd &x, MatrixXd &fjac) const
{
    fjac.resize(m, n);
    fjac.setZero();

    VectorXd epc;
    double tol = 0.00001;
    epc.resize(n);
    int rfSize, sfSize;

    for(int itr=0;itr<n;itr++)
    {
        epc.setZero();
        epc(itr) = tol;
        vector<double> rfPLUS = rotateConstraints(x + epc);
        vector<double> rfMINI = rotateConstraints(x - epc);
        rfSize = rfPLUS.size();
        for(int i=0;i<rfSize;i++)
        {
            fjac(i,itr) = (rfPLUS[i] - rfMINI[i])/(2*tol);
        }
        vector<double> sfPLUS = sliderConstraints(x + epc);
        vector<double> sfMINI = sliderConstraints(x - epc);
        sfSize = sfPLUS.size();
        for(int i=0;i<sfSize;i++)
        {
            fjac(i+rfSize,itr) = (rfPLUS[i] - rfMINI[i])/(2*tol);
        }
    }

    fjac(rfSize+sfSize, 3*groundLinkID) = 1;
    fjac(rfSize+sfSize+1, 3*groundLinkID+1) = 1;
    fjac(rfSize+sfSize+2, 3*groundLinkID+2) = 1;
    fjac(rfSize+sfSize+3, 3*driverLinkID) = 1;

    return 0;
}

vector<double> LMSlover::rotateConstraints(const VectorXd &x) const
{
    vector<double> fx;
    vector<Matrix2d> rMVec;
    vector<Vector2d> tMVec;
    for(int i=0;i<linksNum;i++)
    {
        Matrix2d rM;
        rM(0,0) = cos(x(i*3));
        rM(0,1) = -sin(x(i*3));
        rM(1,0) = sin(x(i*3));
        rM(1,1) = cos(x(i*3));
        rMVec.push_back(rM);
        tMVec.emplace_back(x(3*i+1), x(3*i+2));
    }

    int jointNum = jointPairLinkID.size();
    for(int i=0;i<jointNum;i++)
    {
        int linkA_Id = jointPairLinkID[i].x();
        int linkB_Id = jointPairLinkID[i].y();
        Vector2d diff = rMVec[linkA_Id]*jointPairLocalPos1[i] + tMVec[linkA_Id]
                - rMVec[linkB_Id]*jointPairLocalPos2[i] - tMVec[linkB_Id];
        fx.push_back(diff.x());
        fx.push_back(diff.y());
    }
    return fx;
}

vector<double> LMSlover::sliderConstraints(const VectorXd &x) const
{
    vector<double> fx;
    vector<Matrix2d> rMVec;
    vector<Vector2d> tMVec;
    for(int i=0;i<linksNum;i++)
    {
        Matrix2d rM;
        rM(0,0) = cos(x(i*3));
        rM(0,1) = -sin(x(i*3));
        rM(1,0) = sin(x(i*3));
        rM(1,1) = cos(x(i*3));
        rMVec.push_back(rM);
        tMVec.emplace_back(x(3*i+1), x(3*i+2));
    }

    int sliderNum = sliderPairLinkID.size();
    for(int i=0;i<sliderNum;i++)
    {
        int linkA_Id = sliderPairLinkID[i].x();
        int linkB_Id = sliderPairLinkID[i].y();
        Vector2d pA, pB1, pB2;
        pA = rMVec[linkA_Id]*sliderPairLocalPos1[i] + tMVec[linkA_Id];
        pB1 = rMVec[linkB_Id]*sliderPairLocalPos2[i] + tMVec[linkB_Id];
        pB2 = rMVec[linkB_Id]*sliderPairLocalPos3[i] + tMVec[linkB_Id];
        Vector2d line1 = pB1 - pA;
        Vector2d line2 = pB2 - pA;
        fx.push_back(line1.x()*line2.y() - line1.y()*line2.x());
    }
    return fx;
}
