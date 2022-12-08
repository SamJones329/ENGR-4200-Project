#include <vector>
#include <Eigen/Dense>
#include <random>
#include "./low_pass_filter.hpp"
#include <dory_localization/high_pass_filter.hpp>
#include <dory_localization/low_pass_filter.hpp>

#define RAD_TO_DEGR 180. / M_PI
#define mG_TO_m 1000 * 9.81
#define NUM_PARTICLES 500

using namespace Eigen;
using namespace std;

namespace DoryLoc
{
  /**
   * Rao-Blackwellized Particle Filter for BlueRov2 Pixhawk IMU and DVL A-50 over ROS topics.
   * Reports distance in meters and angle in Yaw-Pitch-Roll/Euler angle/ZYX rotation order 
  */
  class RBPF 
  {
  private:

    const double IMU_EFFECTIVE_SAMPLE_RATE = 10; // Hz
    const double LPF_FREQ = 3; // Hz
    const double INIT_TIME_DELTA = 1. / IMU_EFFECTIVE_SAMPLE_RATE;
    const double HPF_FREQ = 1; // Hz
    const double HPF_OMEGA_C  = 1. / (2 * M_PI * HPF_FREQ);

    // Low pass filters for acceleration measurements for attitude calculation 
    // SCALED_IMU2 ros topic publishes at ~10Hz so we go for 3Hz as our cutoff freq 
    // traditionally you would use a fixed time delta, but since we only need 10Hz we 
    // will dynamically populate the time delta
    LowPassFilter lpfx;
    LowPassFilter lpfy;
    LowPassFilter lpfz;

    // High pass filters for integrated gyro measurements (angle)
    HighPassFilter hpfx;
    HighPassFilter hpfy;
    HighPassFilter hpfz;

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

    // generator random dists
    std::random_device rd;
    std::mt19937 generator;

    // distribution for picking random particle during resampling
    std::uniform_real_distribution<double> random_particle;

    /** 
     * Particles w/ state of form {
     * 0, 1, 2: x, y, z, 
     * 3, 4, 5: roll, pitch, yaw, 
     * 6, 7, 8: xvel, yvel, zvel, 
     * 9, 10, 11: rollvel, pitchvel, yawvel, 
     * 12, 13, 14: xacc, yacc, zacc
     * } */
    Matrix<double, NUM_PARTICLES, 6> x; // x, y, z, roll, pitch, yaw
    
    /** Particle weights */
    Matrix<double, NUM_PARTICLES, 1> wei;

    uint32_t time;

  public:
    /**
     * @param m0 Initial state
     *
     */
    RBPF(bool verbose = false)
    : lpfx(LPF_FREQ, INIT_TIME_DELTA)
    , lpfy(LPF_FREQ, INIT_TIME_DELTA)
    , lpfz(LPF_FREQ, INIT_TIME_DELTA)
    , hpfx(INIT_TIME_DELTA, HPF_OMEGA_C)
    , hpfy(INIT_TIME_DELTA, HPF_OMEGA_C)
    , hpfz(INIT_TIME_DELTA, HPF_OMEGA_C)
    , rd()
    , generator(rd())
    , random_particle(0, NUM_PARTICLES - 1)
    , x(ArrayXXd::Zero(NUM_PARTICLES, 6))
    , wei(ArrayXd::Zero(NUM_PARTICLES))
    {
    }

    /**
     * SIR Particle Filter prediction step.
     * 
     * @param u The Action input to the prediction step. Should be a length 10 vector containing a 
     * 9DOF IMU input SCALED_IMU2 MAVLINK message (https://mavlink.io/en/messages/common.html#SCALED_IMU2).
     * @param timestamp The timestamp (ms) from the SCALED_IMU2 MAVLINK message. Represents time since boot.
    */
    void predict(vector<uint16_t> u, uint32_t timestamp)
    {
      uint32_t timeDelta = timestamp - time;
      time = timestamp;
      const double tic = timeDelta * 0.5; // trapezoidal integration coefficient for current timeDelta

      // x, y, z, acc in mG, convert to m/s^2
      double xVelDelta = tic * (u.at(0) + lastAccX);
      double yVelDelta = tic * (u.at(1) + lastAccY);
      double zVelDelta = tic * (u.at(2) + lastAccZ);
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
      };
      lastVelX = nextVelX;
      lastVelY = nextVelY;
      lastVelZ = nextVelZ;



      // TODO Mason did not include data for magnetometer so this is currently unusable
      // cooperative fusion of acceleromter and magnetometer to get global attitude deltas
      // calculate angle deltas from accelerometer and magnetometer
      // filtered acceleration vector NOTE - we assume the acceleration vector approximates direction of gravity
      // source: https://wiki.dfrobot.com/How_to_Use_a_Three-Axis_Accelerometer_for_Tilt_Sensing
      // Vector3d down {
      //   lpfx.update(acc[0], timeDelta, LPF_FREQ), 
      //   lpfy.update(acc[0], timeDelta, LPF_FREQ), 
      //   lpfz.update(acc[0], timeDelta, LPF_FREQ)
      // };

      // double accRoll = atan2(down[1], down[0]) * RAD_TO_DEGR;
      // double accDeltaRoll = accRoll - lastAccRoll;
      // lastAccRoll = accRoll;

      // double accPitch = atan2(-down[0], sqrt(down[1] * down[1] + down[2] * down[2])) * RAD_TO_DEGR;
      // double accDeltaPitch = accPitch - lastAccPitch;
      // lastAccPitch = accPitch;

      // Vector3d mag {u.at(6), u.at(7), u.at(8)};
      // Vector3d north = mag - ((mag.dot(down) / down.dot(down)) * down);
      
      // // heading in degrees counted clockwise from y axis (NOTE i dont think this comment is correct)
      // double magYaw = atan2(north[0], north[1]) * RAD_TO_DEGR;
      // double magDeltaYaw = magYaw - lastMagYaw;
      // lastMagYaw = magYaw;



      // calculate angle deltas from gyro
      // integral of vel from from prev time to cur time
      // we linearize this to calculate by trapezoidal rule
      // angleDelta =  deltaT * (v1 + v2) / 2
      Vector3d gyroDelta {
        tic * (lastGyroVelX + u.at(3)),
        tic * (lastGyroVelY + u.at(4)),
        tic * (lastGyroVelZ + u.at(5))
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
      for(int i = 0; i < NUM_PARTICLES; i++) {
        // consts to avoid recalculation
        const double cosX = cos(x(i,3)), cosY = cos(x(i,4)), cosZ = cos(x(i,5)),
          sinX = sin(x(i,3)), sinY = sin(x(i,4)), sinZ = sin(x(i,5));
        Matrix3d rot; 
        rot(0,0) = cosZ*cosY*cosX - sinZ*sinX;
        rot(0,1) = -sinZ*cosY*cosX - cosZ*sinX;
        rot(0,2) = sinY*cosX;
        rot(1,0) = cosZ*cosY*sinX + sinZ*cosX;
        rot(1,1) = -sinZ*cosY*sinX + cosZ*cosX;
        rot(1,2) = sinY*sinX;
        rot(2,0) = -cosZ*sinY;
        rot(2,1) = sinZ*sinY;
        rot(2,2) = cosY;
        
        // put in particle's ref frame
        Vector3d particlePosDelta = rot*posDelta;        
        Vector3d particleAngDelta = rot*gyroDelta;
        

        // TODO see above about lack of mag data
        // complementary filter to combine accel+mag attitude with gyro angleDeltas
        // particleAngDelta(0) = A * particleAngDelta(0) + cA * accDeltaRoll;
        // particleAngDelta(1) = A * particleAngDelta(1) + cA * accDeltaPitch;
        // particleAngDelta(2) = A * particleAngDelta(2) + cA * magDeltaYaw;


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
      weight();
      resample();
    }

    void weight() {

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
      VectorXd positions = ArrayXd::Zero(NUM_PARTICLES);

      // array of indexes into the weights defining the resample. i.e. the
      // index of the zeroth resample is indexes[0], etc.
      VectorXd indexes = ArrayXd::Zero(NUM_PARTICLES); // np.zeros(N, 'i')
      // cumulative_sum = np.cumsum(weights)
      VectorXd cumSum = ArrayXd::Zero(NUM_PARTICLES);
      cumSum(0) = wei(0);
      for (int i = 1; i < NUM_PARTICLES; i++)
      {
        positions(i) = (i + this->random_particle(this->generator)) / NUM_PARTICLES;
        cumSum(i) = cumSum(i - 1) + wei(i);
      }

      int i = 0, j = 0;
      while (i < NUM_PARTICLES)
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

      for (int i = 0; i < NUM_PARTICLES; i++)
      {
        // particle(i) = particle(indexes);
      }
    }
  };
}