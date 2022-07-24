////////////////////////////////////////
///
///     a non-linear solver
///     to min sum(f_i^2)_{i=1...m}
///     with f_i = f_i(x_1, .., x_n)
///
////////////////////////////////////////

#ifndef LMSOLVER_H
#define LMSOLVER_H

#include "eigen-3.4.0/Eigen/Eigen"
#include <vector>

using namespace std;
using namespace Eigen;

struct LMSlover
{
    int linksNum;
    vector<Vector2i> jointPairLinkID;
    vector<Vector2d> jointPairLocalPos1;
    vector<Vector2d> jointPairLocalPos2;

    vector<Vector2i> sliderPairLinkID;
    vector<Vector2d> sliderPairLocalPos1;
    vector<Vector2d> sliderPairLocalPos2;
    vector<Vector2d> sliderPairLocalPos3;

    int groundLinkID;
    int driverLinkID;

    vector<double> initRotateAngle;
    vector<Vector2d> initTransPosition;

    double driverAngle;

    // fvec = f(x) = ( ,fi(x), )
    int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const;

    // Jco = dfi/dxj
    int df(const Eigen::VectorXd &x, Eigen::MatrixXd &fjac) const;

    // Number of data points, i.e. values.
    int m;

    // Returns 'm', the number of values.
    [[nodiscard]] int values() const { return m; }

    // The number of parameters, i.e. inputs.
    int n;

    // Returns 'n', the number of inputs.
    [[nodiscard]] int inputs() const { return n; }

    // for a revolve joint, R_1*P_1 - R_2*P_2 = 0
    vector<double> rotateConstraints(const VectorXd &x) const;

    // for a slider, point P1 is on link L1, points P2&P3 are on link L2
    // P1, P2&P3 are co-line:
    // (M1*P1 - M2*P2) // (M1*P1 - M2*P3)
    vector<double> sliderConstraints(const VectorXd &x) const;
};

#endif // LMSOLVER_H
