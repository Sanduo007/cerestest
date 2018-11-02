#include <ceres/ceres.h>
#include <ceres/rotation.h>

struct CURVE_FITTING_COST
{
    CURVE_FITTING_COST(double _k1, double _k2, double _z) : k1(_k1), k2(_k2), z(_z) {}

    template <typename T>
    bool operator()(const T *const xy, T *residuals) const
    {
        residuals[0] = xy[0] * xy[0] * T(k1) + T(k2) * xy[1] * xy[1] * xy[1] - T(z);
        //cout << "r: " << residuals[0] << " " << xy[0] << " " << xy[1] << " " << k1 << " " << k2 << " " << z << endl;
        return true;
    }

    static ceres::CostFunction *Create(const double _k1, const double _k2, const double _z)
    {
        return (new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 2>(new CURVE_FITTING_COST(_k1, _k2, _z)));
    }

    const double k1;
    const double k2;
    const double z;
};

//input: normalized uv
struct REPROJECT_COST
{
    REPROJECT_COST(double _observed_norm_u, double _observed_norm_v)
        : observed_norm_u(_observed_norm_u), observed_norm_v(_observed_norm_u)
    {
    }

    template <typename T>
    bool operator()(const T *const Rwc, const T *const Twc, const T *pw, T *residuals) const
    {
        T pc[3];
        ceres::QuaternionRotatePoint(Rwc, pw, pc);
        pc[0] += Twc[0];
        pc[1] += Twc[1];
        pc[2] += Twc[2];

        T up = pc[0] / pc[2];
        T vp = pc[1] / pc[2];

        residuals[0] = up - T(observed_norm_u);
        residuals[1] = vp - T(observed_norm_v);

        return true;
    }

    static ceres::CostFunction *Create(const double _observed_norm_u, const double _observed_norm_v)
    {
        return (new ceres::AutoDiffCostFunction<REPROJECT_COST, 2, 4, 3, 3>(new REPROJECT_COST(_observed_norm_u, _observed_norm_v)));
    }

    double observed_norm_u;
    double observed_norm_v;
};