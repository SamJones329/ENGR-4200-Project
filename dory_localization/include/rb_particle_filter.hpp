#include <vector>
#include <Eigen/Dense>
#include "./localizer.hpp"

using namespace Eigen;
using namespace std;

namespace DoryLoc
{
  template <int numParticles>
  class RBPF : public Localizer<6> // x, y, z, roll, pitch, yaw
  {

    // Vector<double, DoF> state;
    /** 
     * Particles w/ state of form {
     * 0, 1, 2: x, y, z, 
     * 3, 4, 5: roll, pitch, yaw, 
     * 6, 7, 8: xvel, yvel, zvel, 
     * 9, 10, 11: rollvel, pitchvel, yawvel, 
     * 12, 13, 14: xacc, yacc, zacc
     * } */
    Matrix<double, numParticles, 15> x;

    uint32_t time;

    /**
     * @param m0 Initial state
     *
     */
    RBPF(Vector<double, DoF> m0, Matrix<double, DoF, DoF> P0, VectorXd ys, MatrixXd Q, boolean verbose = false)
    {
    }

    /**
     * This prediction method is based on the Kalman Filter prediction step.
     * 
     * @param u The Action input to the prediction step. Should be a length 10 vector containing a 
     * 9DOF IMU input plus timestamp in the form of the SCALED_IMU2 MAVLINK message (https://mavlink.io/en/messages/common.html#SCALED_IMU2).
    */
    void predict(vector<double> u)
    {
      // """Kalman filter prediction step"""
      // m_p = A @ m
      // P_p = A @ P @ A.T + Q
      // return m_p, P_p 
      uint32_t newTime = u.at(0);
      auto timeDelta = newTime - time;
      time = newTime;

      // x, y, z, acc in mG
      int16_t xacc = u.at(1);
      int16_t yacc = u.at(2);
      int16_t zacc = u.at(3);
      int16_t xaccDelta = xacc - x[12];
      int16_t yaccDelta = yacc - x[13];
      int16_t zaccDelta = zacc - x[14];
      
      
      // 
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