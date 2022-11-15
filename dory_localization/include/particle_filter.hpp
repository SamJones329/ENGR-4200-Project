#include <vector>
#include <Eigen/Dense>
#include <random>
#include <cmath>
#include <iostream>
#include <string>
#include "./localizer.hpp"

using namespace Eigen;

namespace DoryLoc {
    class ParticleFilter { // : public Localizer {
    public:
        int num;
        bool moving;
        double nEff;

        double odomLinSigma;
        double odomAngSigma;
        double measRngNoise;
        double measYawNoise;
        std::mt19937 generator;
        std::uniform_real_distribution<double> distribution;

        VectorXd pWei; 
        VectorXd pYaw; 
        MatrixXd pxyz;

        // Gaussian Constants
        double valRng;
        double rngSigSq2;
        double valAng;
        double angSigSq2;

        void setParticleAngles(VectorXd newAngles);

        ParticleFilter() : ParticleFilter(500, 0.2, 0.174533, 0., 0., 0., 1.5) {
            // DVL A50 WL-21035-2 TODO - find out if have standard or performance version
            // standard has long term sensor (velocity) accuracy of +-1.01%, perf has +-0.1%
        }

        ParticleFilter(int num, double measRngNoise, double measYawNoise, 
                double xInit, double yInit, double yawInit, double particleRange);

        /**
         * Moves particles with the given odometry.
         * 
         * odom - incremental odometry [delta_x, delta_y, delta_yaw] in the 
         * vehicle frame
        */
        void predict(std::vector<double> uk);

        void update(std::vector<double> zk);

        void weight(std::vector<double> odom);

        void resample();

        /**
         * Obtain the mean particle of the particle filter, or the state belief.
         * 
         * @return length 4 vector of the form [x, y, z, yaw]
        */
        std::vector<double> getBelief(); // override;

        /**
         * Get all the particles of the filter.
         * @return A vector with all the particles of the particle filter
         * where each vector element is a length 4 array of the form [x, y, z, yaw, weight]
        */
        std::vector<std::vector<double>> getParticles();
    };
}