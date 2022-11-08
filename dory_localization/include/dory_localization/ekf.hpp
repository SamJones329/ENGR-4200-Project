#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

namespace BlueROV2
{
  class EKF
  {
  private:
    /** Prediction Noise */
    Matrix<float, 4, 4> Q;

    /** Measurement Noise*/
    Matrix<float, 4, 4> R;

    /** State Prediction */
    Vector<float, 4> x;

    /** State Uncertainty */
    Matrix<float, 4, 4> P;

  public:
    /**
     * Creates a new Extended Kalman Filter for localizing
     * a BlueROV2 using Pixhawk IMU odometry changes as the
     * action, and using odometry from the DVL A50 and
     * pressure sensor as measurement.
     */
    EKF();

    /**
     * Prediction step of EKF. Updates state prediction
     * and uncertainty based on action.
     *
     * @param u length 4 float array of the format
     * [xDelta, yDelta, zDelta, yawDelta]
     */
    void predict(float *u);

    /**
     * Data association and prediction update steps of EKF.
     * Updates state prediction and uncertainty based on
     * measurements.
     *
     * @param u length 4 float array of the format
     * [xDelta, yDelta, zDelta, yawDelta]
     */
    void updatePrediction(float *u);
  };
}