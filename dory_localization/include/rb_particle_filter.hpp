#include <vector>
#include <Eigen/Dense>
#include "./localizer.hpp"
#include "./low_pass_filter.hpp"
#include "./high_pass_filter.hpp"

#define IMU_EFFECTIVE_SAMPLE_RATE 10f // Hz
#define LPF_FREQ 3f // Hz
#define INIT_TIME_DELTA 1. / IMU_EFFECTIVE_SAMPLE_RATE
#define HPF_FREQ 3f //Hz
#define HPF_OMEGA_C 1f / (2f * M_PI * HPF_FREQ)
#define RAD_TO_DEGR 180f / M_PI
#define mG_TO_m 1000 * 9.81

using namespace Eigen;
using namespace std;

namespace DoryLoc
{
  /**
   * Rao-Blackwellized Particle Filter for BlueRov2 Pixhawk IMU and DVL A-50 over ROS topics.
   * Reports distance in meters and angle in Yaw-Pitch-Roll/Euler angle/ZYX rotation order 
  */
  template <int numParticles>
  class RBPF : public Localizer<6> // x, y, z, roll, pitch, yaw
  {
  private:
    // Low pass filters for acceleration measurements for attitude calculation 
    // SCALED_IMU2 ros topic publishes at ~10Hz so we go for 3Hz as our cutoff freq 
    // traditionally you would use a fixed time delta, but since we only need 10Hz we 
    // will dynamically populate the time delta
    LowPassFilter lpfx(LPF_FREQ, INIT_TIME_DELTA);
    LowPassFilter lpfy(LPF_FREQ, INIT_TIME_DELTA);
    LowPassFilter lpfz(LPF_FREQ, INIT_TIME_DELTA);

    // High pass filters for integrated gyro measurements (angle)
    HighPassFilter lpfx(INIT_TIME_DELTA, HPF_OMEGA_C);
    HighPassFilter lpfy(INIT_TIME_DELTA, HPF_OMEGA_C);
    HighPassFilter lpfz(INIT_TIME_DELTA, HPF_OMEGA_C);

    /** 
     * Complementary filter gain. Used as follows:
     * Output = A * GyroOutput + (1 - A) * Accelerometer&MagnetometerOutput
    */
    const double A = 0.98;
    // complement of A
    const double cA = 1 - A;

    uint16_t lastAccX;
    uint16_t lastAccY;
    uint16_t lastAccZ;
    double lastVelX;
    double lastVelY;
    double lastVelZ;
    double lastAccRoll;
    double lastAccPitch;
    double lastMagYaw;
    uint16_t lastGyroVelX;
    uint16_t lastGyroVelY;
    uint16_t lastGyroVelZ;

    /** 
     * Particles w/ state of form {
     * 0, 1, 2: x, y, z, 
     * 3, 4, 5: roll, pitch, yaw, 
     * 6, 7, 8: xvel, yvel, zvel, 
     * 9, 10, 11: rollvel, pitchvel, yawvel, 
     * 12, 13, 14: xacc, yacc, zacc
     * } */
    Matrix<double, numParticles, 6> x;

    uint32_t time;

  public:
    /**
     * @param m0 Initial state
     *
     */
    RBPF(Vector<double, DoF> m0, Matrix<double, DoF, DoF> P0, VectorXd ys, MatrixXd Q, boolean verbose = false)
    {
    }

    /**
     * SIR Particle Filter prediction step.
     * 
     * @param u The Action input to the prediction step. Should be a length 10 vector containing a 
     * 9DOF IMU input SCALED_IMU2 MAVLINK message (https://mavlink.io/en/messages/common.html#SCALED_IMU2).
     * @param timestamp The timestamp from the SCALED_IMU2 MAVLINK message. Represents time since boot.
    */
    void predict(vector<uint16_t> u, uint32_t timestamp)
    {
      uint32_t timeDelta = newTime - time;
      time = newTime;
      const double tic = timeDelta * 0.5; // trapezoidal integration coefficient for current timeDelta

      // x, y, z, acc in mG, convert to m/s^2
      double xVelDelta = tic * (acc[0] + x[12]);
      double yVelDelta = tic * (acc[1] + x[13]);
      double zVelDelta = tic * (acc[2] + x[14]);
      lastAccX = u.at(0);
      lastAccY = u.at(1);
      lastAccZ = u.at(2);
      double nextVelX = lastVelX + xVelDelta;
      double nextVelY = lastVelY + yVelDelta;
      double nextVelZ = lastVelZ + zVelDelta;
      const double pic = tic * mG_TO_m; // integrates to position and converts to meters
      Vector3d posDelta {
        pic * (lastVelX + nextVelX),
        pic * (lastVelY + nextVelY),
        pic * (lastVelZ + nextVelZ)
      }
      lastvelX = nextVelX;
      lastvelY = nextVelY;
      lastvelZ = nextVelZ;

      // cooperative fusion of acceleromter and magnetometer to get global attitude deltas
      // calculate angle deltas from accelerometer and magnetometer
      // filtered acceleration vector NOTE - we assume the acceleration vector approximates direction of gravity
      // source: https://wiki.dfrobot.com/How_to_Use_a_Three-Axis_Accelerometer_for_Tilt_Sensing
      Vector3d down {
        lpfx.update(acc[0], timeDelta, LPF_FREQ), 
        lpfy.update(acc[0], timeDelta, LPF_FREQ), 
        lpfz.update(acc[0], timeDelta, LPF_FREQ)
      };

      double accRoll = atan2(down[1], down[0]) * RAD_TO_DEGR;
      double accDeltaRoll = accRoll - lastAccRoll;
      lastAccRoll = accRoll;

      double accPitch = atan2(-down[0], sqrt(down[1] * down[1] + down[2] * down[2])) * RAD_TO_DEGR;
      double accDeltaPitch = accPitch - lastAccPitch;
      lastAccPitch = accPitch;

      Vector3d mag {u.at(6), u.at(7), u.at(8)};
      Vector3d north = mag - ((mag.dot(down) / down.dot(down)) * down);
      
      // heading in degrees counted clockwise from y axis (NOTE i dont think this comment is correct)
      double magYaw = atan2(north[0], north[1]) * RAD_TO_DEGR;
      double magDeltaYaw = magYaw - lastMagYaw;
      lastMagYaw = magYaw;

      // calculate angle deltas from gyro
      // integral of vel from from prev time to cur time
      // we linearize this to calculate by trapezoidal rule
      // angleDelta =  deltaT * (v1 + v2) / 2
      Vector3d gyroDelta {
        gic * (lastGyroVelX + u.at(3)),
        gic * (lastGyroVelY + u.at(4)),
        gic * (lastGyroVelZ + u.at(5))
      };
      lastGyroVelX = u.at(3);
      lastGyroVelY = u.at(4);
      lastGyroVelZ = u.at(5);

      // get rotation matrix for each particle
      // and rotate each delta to align with 
      // robot's axes
      // https://semath.info/src/euler-angle.html
      // Z/Psi/Yaw, Y/Pitch/Theta, X/Phi/Roll
      // transform gyro deltas from local XYZ to global RPY for each particle
      for(int i = 0; i < numParticles; i++) {
        // consts to avoid recalculation
        const double cosX = cos(x(i,3)), cosY = cos(x(i,4)), cosZ = cos(x(i,5)),
          sinX = sin(x(i,3)), sinY = sin(x(i,4)), sinZ = sin(x(i,5));
        Matrix3d rot = {
          cosZ*cosY*cosX - sinZ*sinX, -sinZ*cosY*cosX - cosZ*sinX, sinY*cosX,
          cosZ*cosY*sinX + sinZ*cosX, -sinZ*cosY*sinX + cosZ*cosX, sinY*sinX,
          -cosZ*sinY,                 sinZ*sinY,                   cosY
        };
        
        // put in particle's ref frame
        Vector3d particlePosDelta = rot.dot(posDelta);
        Vector3d particleAngDelta = rot.dot(gyroDelta);
        
        // complementary filter to combine accel+mag attitude with gyro angleDeltas
        particleAngDelta(0) = A * particleAngDelta(0) + cA * accDeltaRoll;
        particleAngDelta(1) = A * particleAngDelta(1) + cA * accDeltaPitch;
        particleAngDelta(2) = A * particleAngDelta(2) + cA * magDeltaYaw;

        // update particle
        x(i, 0) += particlePosDelta(0);
        x(i, 1) += particlePosDelta(1);
        x(i, 2) += particlePosDelta(2);
        x(i, 3) += particleAngDelta(0);
        x(i, 4) += particleAngDelta(1);
        x(i, 5) += particleAngDelta(2);
      }

    }


    /**
     * This update method is based on the Kalman Filter update step.
     * 
     * @param z The Measurement input to the update step. Should be a 
     * 
    */
    void update(vector<double> z)
    {
      // """
      // Kalman filter update step for scalar measurement
      // with precomputed innovation moments
      // """
      // K = P @ H.T * (S**-1)
      // m_u = m + K @ v[None, :]
      // P_u = P - S * K @ K.T

      // return m_u, P_u
    }

    /**
     * Performs the stratified resampling algorithm used by particle filters.
     * This algorithms aims to make selections relatively uniformly across the
     * divisions, and then selects one particle randomly from each division. This
     * particles. It divides the cumulative sum of the weights into N equal
     * guarantees that each sample is between 0 and 2/N apart.
     */
    void resample()
    {
      // N = len(weights)
      // make N subdivisions, and chose a random position within each one
      // positions = (random(N) + range(N)) / N
      VectorXd positions = ArrayXd::Zeros(numParticles);

      // array of indexes into the weights defining the resample. i.e. the
      // index of the zeroth resample is indexes[0], etc.
      VectorXd indexes = ArrayXd::Zeros(numParticles); // np.zeros(N, 'i')
      // cumulative_sum = np.cumsum(weights)
      VectorXd cumSum = ArrayXd::Zeros(numParticles);
      cumSum(0) = pWei(0);
      for (int i = 1; i < numParticles; i++)
      {
        positions(i) = (i + random_particle(generator)) / numParticles;
        cumSum(i) = cumSum(i - 1) + pWei(i);
      }

      int i = 0, j = 0;
      while (i < numParticles)
      {
        if (positions(i) < cumSum(i))
        {
          indexes(i) = j;
          i++;
        }
        else
        {
          j++;
        }
      }
      // return indexes

      for (int i = 0; i < numParticles; i++)
      {
        // particle(i) = particle(indexes);
      }
    }
  };
}