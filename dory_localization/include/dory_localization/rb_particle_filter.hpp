#include <vector>
#include <Eigen/Dense>
#include <random>
#include "./low_pass_filter.hpp"
#include <dory_localization/high_pass_filter.hpp>
#include <dory_localization/low_pass_filter.hpp>
#include <boost/format.hpp>

#define RAD_TO_DEGR 180. / M_PI
// #define mG_TO_mps2 1000 * 9.81
#define G 9.80665
#define mG_TO_mps2 0.00980665 // milligravity to m/s^2
// #define mG_TO_mps2 0.00001 // milliGal to m/s^2
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

    bool verbose;

    const double IMU_EFFECTIVE_SAMPLE_RATE = 10; // Hz
    const double LPF_FREQ = 3; // Hz
    const double INIT_TIME_DELTA = 1. / IMU_EFFECTIVE_SAMPLE_RATE;
    const double HPF_FREQ = 1; // Hz
    const double HPF_OMEGA_C  = 1. / (2 * M_PI * HPF_FREQ);
    const double Measurement_Long_term_Accuracy = .0101;  // %  (DVL)
    const double rngSigSq2 = 2. * pow(Measurement_Long_term_Accuracy, 2);
    const double valrng = (1.0 / (Measurement_Long_term_Accuracy* sqrt(2 * M_PI)));

    // Low pass filters for acceleration measurements for attitude calculation 
    // SCALED_IMU2 ros topic publishes at ~10Hz so we go for 3Hz as our cutoff freq 
    // traditionally you would use a fixed time delta, but since we only need 10Hz we 
    // will dynamically populate the time delta
    LowPassFilter lpfx;
    LowPassFilter lpfy;
    LowPassFilter lpfz;

    // High pass filters for extracting non-gravity acceleration
    HighPassFilter hpfx;
    HighPassFilter hpfy;
    HighPassFilter hpfz;

    // High pass filters for integrated gyro measurements (angle)
    // HighPassFilter hpfx;
    // HighPassFilter hpfy;
    // HighPassFilter hpfz;

    /** 
     * Complementary filter gain. Used as follows:
     * Output = A * GyroOutput + (1 - A) * Accelerometer&MagnetometerOutput
    */
    const double A = 0.98;
    // complement of A
    const double cA = 1 - A;

    double lastAccX = 0; // m/s^2
    double lastAccY = 0;
    double lastAccZ = 0;
    double lastJerkX = 0; // m/s^3
    double lastJerkY = 0;
    double lastJerkZ = 0;
    double lastAccLinX = 0; // m/s^2, gravity removed
    double lastAccLinY = 0;
    double lastAccLinZ = 0;
    double lastVelX = 0; // m/s
    double lastVelY = 0;
    double lastVelZ = 0;
    double lastAccRoll = 0; // rad
    double lastAccPitch = 0;
    double lastMagYaw = 0;
    double lastGyroVelX = 0; // rad/s
    double lastGyroVelY = 0;
    double lastGyroVelZ = 0;

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

    /**
     * Log string at info level for RBPF. Outputs 
     * a string to std::cout of the form 
     * <RBPF@address>: {fString}
    */
    void logInfo(boost::format fString) {
      if(!verbose) return;
      std::cout << "<RBPF@" << this << ">: " << fString.str() << std::endl;
    }

    /**
     * Log string at info level for RBPF. Outputs 
     * a string to std::cout of the form 
     * <RBPF@address>: {str}
    */
    void logInfo(std::string str) {
      if(!verbose) return;
      std::cout << "<RBPF@" << this << ">: " << str << std::endl;
    }

    void logWarn(boost::format fString) {
      std::cout << "\033[33m<RBPF@" << this << ":WARN>: " << fString << "\033[0m" << std::endl;
    }

    void logWarn(std::string str) {
      std::cout << "\033[33m<RBPF@" << this << ":WARN>: " << str << "\033[0m" << std::endl;
    }


  public:
    /**
     * @param m0 Initial state
     *
     */
    RBPF(bool verbose = false)
    : verbose(verbose)
    , lpfx(LPF_FREQ, INIT_TIME_DELTA)
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
      logInfo(boost::format("Creating Rao-Blackwellized Particle Filter with %1% particles") % NUM_PARTICLES);
    
      for (int i = 0; i < NUM_PARTICLES; i++) {wei(i) = 1/NUM_PARTICLES;}

    }

    

    /**
     * Set an initial time to compare to and propogate from timestamps passed through predict
     * 
     * @param timestamp time in ms
    */
    void initTime(uint32_t timestamp) {
      time = timestamp;
      logInfo(boost::format("Internal time set: %1%ms") % time);
    }

    /**
     * SIR Particle Filter prediction step.
     * 
     * @param u The Action input to the prediction step. Should be a length 10 vector containing a 
     * 9DOF IMU input SCALED_IMU2 MAVLINK message (https://mavlink.io/en/messages/common.html#SCALED_IMU2).
     * @param timestamp The timestamp (ms) from the SCALED_IMU2 MAVLINK message. Represents time since boot.
    */
    vector<double> predict(vector<double> u, uint32_t timestamp)
    {

      uint32_t timeDelta = timestamp - time;
      // if(timeDelta < 500) return;
      time = timestamp;


      // ignore extremely large time gaps. this is both a 
      // heuristic to handle scenarios where the info is 
      // not received because the robot is turned off but 
      // the node is still runinng, but also to handle time jumps from starting bag files during testing.
      if(timeDelta > 2000) { // 2s
        logWarn(boost::format("Found large time gap dTime=%1%ms. Ignoring measurements and resetting internal time.") % timeDelta);
        return vector<double>{0,0,0};
      } else if (timeDelta <= 0) {
        logWarn(boost::format("Found time gap <=0 dTime=%1%. Ignoring measurements and resetting internal time.") % timeDelta);
        return vector<double>{0,0,0};
      }

      const double timeDeltaSec = timeDelta * 0.001;
      const double tdc = 1 / timeDeltaSec; // time derivative constant, reciprocal deltaTime in seconds
      const double tic = timeDelta * 0.0005; // (1/2) / 1000 // trapezoidal integration coefficient for current timeDelta (converted to s from ms)

      // x, y, z, acc in mG, convert to m/s^2
      const double xAcc = u.at(0) * mG_TO_mps2;
      const double yAcc = u.at(1) * mG_TO_mps2;
      const double zAcc = u.at(2) * mG_TO_mps2 + 9.81;
      // derivate to jerk and reintegrate to remove constant (gravity)
      double xJerk = tdc * (xAcc - lastAccX);
      double yJerk = tdc * (yAcc - lastAccY);
      double zJerk = tdc * (zAcc - lastAccZ);
      double xAccLin = xAcc; //tic * (xJerk + lastJerkX);
      double yAccLin = yAcc; //tic * (yJerk + lastJerkY);
      double zAccLin = zAcc; //tic * (zJerk + lastJerkZ);
      // update stored jerk
      lastJerkX = xJerk;
      lastJerkY = yJerk;
      lastJerkZ = zJerk;

      // use high pass filter to filter out low frequency noise
      // hpfx.update(xAcc);
      // hpfy.update(yAcc);
      // hpfz.update(zAcc);
      // double xAccLin = hpfx.getOutput();
      // double yAccLin = hpfy.getOutput();
      // double zAccLin = hpfz.getOutput();


      // TODO data for magnetometer was not recorded so this is currently unusable
      // cooperative fusion of acceleromter and magnetometer to get global attitude deltas
      // calculate angle deltas from accelerometer and magnetometer
      // filtered acceleration vector NOTE - we assume the acceleration vector approximates direction of gravity
      // source: https://wiki.dfrobot.com/How_to_Use_a_Three-Axis_Accelerometer_for_Tilt_Sensing
      // Vector3d down {
      //   lpfx.update(xAcc, timeDeltaSec, LPF_FREQ), 
      //   lpfy.update(yAcc, timeDeltaSec, LPF_FREQ), 
      //   lpfz.update(zAcc, timeDeltaSec, LPF_FREQ)
      // };
      // Vector3d gravity = down * G;

      // double xAccLin = xAcc - down(0);
      // double yAccLin = yAcc - down(1);
      // double zAccLin = zAcc - down(2);

      // // update stored acc
      double xVelDelta = tic * (lastAccLinX + xAccLin);
      double yVelDelta = tic * (lastAccLinY + xAccLin);
      double zVelDelta = tic * (lastAccLinZ + xAccLin);
      lastAccX = xAcc;
      lastAccY = yAcc;
      lastAccZ = zAcc;
      // update stored linear acceleration
      lastAccLinX = xAccLin;
      lastAccLinY = yAccLin;
      lastAccLinZ = zAccLin;

      double nextVelX = lastVelX + xVelDelta;
      double nextVelY = lastVelY + yVelDelta;
      double nextVelZ = lastVelZ + zVelDelta;
      vector<double> vels {nextVelX, nextVelY, nextVelZ};
      Vector3d posDelta {
        tic * (lastVelX + nextVelX),
        tic * (lastVelY + nextVelY),
        tic * (lastVelZ + nextVelZ)
      };
      // update stored linear velocity
      lastVelX = nextVelX;
      lastVelY = nextVelY;
      lastVelZ = nextVelZ;


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
      const double gic = tic * 0.001; // gyro integration constant, convert from mrad/s to rad/s
      Vector3d gyroDelta { // switch roll and pitch *******
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
      for(int i = 0; i < NUM_PARTICLES; i++) { // switch roll and pitch **************
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
        // Vector3d particleAngDelta = rot*gyroDelta;
        

        // TODO see above about lack of mag data
        // complementary filter to combine accel+mag attitude with gyro angleDeltas
        // particleAngDelta(0) = A * particleAngDelta(0) + cA * accDeltaRoll;
        // particleAngDelta(1) = A * particleAngDelta(1) + cA * accDeltaPitch;
        // particleAngDelta(2) = A * particleAngDelta(2) + cA * magDeltaYaw;


        // update particle
        x(i, 0) += particlePosDelta(0);
        x(i, 1) += particlePosDelta(1);
        x(i, 2) += particlePosDelta(2);
        // x(i, 3) += particleAngDelta(0);
        // x(i, 4) += particleAngDelta(1);
        x(i, 5) += gyroDelta(2);// particleAngDelta(2);

      }
      logInfo(boost::format("\nPredicting @ time %1%ms w/ dTime %2%ms") % timestamp % timeDelta);
      logInfo(boost::format("tic: %1% \n\tXYZacc: {%2%, %3%, %4%} \n\tXYZlinAcc: {%5%, %6%, %7%} \n\tXYZdvel: {%8%, %9%, %10%} \n\tXYZvel: {%11%, %12%, %13%}") 
        % tic
        % xAcc % yAcc % zAcc
        % xAccLin % yAccLin % zAccLin
        % xVelDelta % yVelDelta % zVelDelta
        % nextVelX % nextVelY % nextVelZ);
      logInfo(boost::format("Got XYZRPY deltas \n\t{%1%, %2%, %3%, %4%, %5%, %6%} from inputs \n\t{%7%, %8%, %9%, %10%, %11%, %12%}") 
        % posDelta(0) 
        % posDelta(1) 
        % posDelta(2) 
        % gyroDelta(0) 
        % gyroDelta(1) 
        % gyroDelta(2)
        % u.at(0)
        % u.at(1)
        % u.at(2)
        % u.at(3)
        % u.at(4)
        % u.at(5));

      return vels;

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
      weight(z);
      resample();
    }

    /**
     * Weight each particle based on measurement
     * 
     * @param z Vector of measurements from DVL's internal filter in form {x, y, z, roll, pitch, yaw}
    */
    void weight(vector<double> z) {
      double x_pos = z[0], y_pos = z[1], z_pos = z[2], roll = z[3], pitch = z[4], yaw = z[5];
      Matrix<double, NUM_PARTICLES, 1> weights;
      for (int i = 0; i < NUM_PARTICLES; i++)
      {
        double px = x(i, 0), py = x(1, i), pz = x(2, i), proll = x(3,i), ppitch = x(4,i), pyaw = x(5,i);
        double wx = valrng * exp(-pow(x_pos - px, 2) / rngSigSq2);
        double wy = valrng * exp(-pow(y_pos - py, 2) / rngSigSq2);
        double wz = valrng * exp(-pow(z_pos - pz, 2) / rngSigSq2);
        double yawDiff = abs(yaw - pyaw);
        if(yawDiff > M_PI) {
          yawDiff -= M_PI;
        }
        double wyaw = valrng * exp(-pow(yawDiff, 2) / rngSigSq2);
        double rollDiff = abs(roll - proll);
        if(rollDiff > M_PI) {
          rollDiff -= M_PI;
        }
        double wroll = valrng * exp(-pow(rollDiff, 2) / rngSigSq2);
        double pitchDiff = abs(pitch - ppitch);
        if(pitchDiff > M_PI) {
          pitchDiff -= M_PI;
        }
        double wpitch = valrng * exp(-pow(pitchDiff, 2) / rngSigSq2);
        weights(i) = wx * wy * wz * wyaw * wroll * wpitch;
      }
      double wsum = weights.sum();
      weights /= wsum;
      wei = weights;
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


    /**
     * Get the mean particle of the filter based on particle states and weights.
    */
    Matrix<double, 6, 1> getBelief() {
      Matrix<double, 6, 1> belief = ArrayXd::Zero(6);
      for(int i = 0; i < NUM_PARTICLES; i++) {
        belief(0) += x(i,0); 
        belief(1) += x(i,1); 
        belief(2) += x(i,2); 
        belief(3) += x(i,3); 
        belief(4) += x(i,4); 
        belief(5) += x(i,5); 
      }
      belief(0) /= NUM_PARTICLES;
      belief(1) /= NUM_PARTICLES;
      belief(2) /= NUM_PARTICLES;
      belief(3) /= NUM_PARTICLES;
      belief(4) /= NUM_PARTICLES;
      belief(5) /= NUM_PARTICLES;
      logInfo(boost::format("Getting belief {%1%, %2%, %3%, %4%, %5%, %6%}") % belief(0) % belief(1) % belief(2) % belief(3) % belief(4) % belief(5));
      return belief;
    }

    /**
     * Get all particle states
     * 
     * @return The state matrix of particles. 1 row per particle of the form {x, y, z, roll, pitch, yaw, weight}
     * with x, y, z being in m and roll, pitch, yaw being euler angles (rotated in YPR or ZYX order).
    */
   Matrix<double, NUM_PARTICLES, 7> getParticles() {
    logInfo("Getting particles");
    MatrixXd particles = x;
    particles.conservativeResize(NoChange, 7);
    particles.col(6) = wei;
    return particles;
   }
  };
}