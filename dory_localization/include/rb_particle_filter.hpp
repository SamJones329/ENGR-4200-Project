#include <vector>
#include <Eigen/Dense>
#include "./localizer.hpp"

using namespace Eigen;

namespace DoryLoc
{
  template <int numParticles, int DoF>
  class RBPF : public Localizer<DoF>
  {

    /**
     * @param m0 Initial state
     *
     */
    RBPF(Vector<double, DoF> m0, Matrix<double, DoF, DoF> P0, VectorXd ys, MatrixXd Q, boolean verbose = false)
    {
    }

    void predict()
    {
      // """Kalman filter prediction step"""
      // m_p = A @ m
      // P_p = A @ P @ A.T + Q
      // return m_p, P_p
    }

    void update()
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
  }
}