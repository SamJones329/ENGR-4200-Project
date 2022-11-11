#include <vector>
#include <Eigen/Dense>
#include <random>
#include <cmath>
#include <iostream>
#include <string>
#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.

using namespace Eigen;

namespace DoryLoc {
    class ParticleFilter {
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

        boost::uuids::uuid uuid;

        void setParticleAngles(VectorXd newAngles);

        ParticleFilter() : ParticleFilter(500, 0.0101, 0.0101, 0., 0., 0.) {
            // DVL A50 WL-21035-2 TODO - find out if have standard or performance version
            // standard has long term sensor (velocity) accuracy of +-1.01%, perf has +-0.1%
        }

        ParticleFilter(int num, double measRngNoise, double measYawNoise, 
                double xInit, double yInit,double yawInit);

        /**
         * Moves particles with the given odometry.
         * 
         * odom - incremental odometry [delta_x, delta_y, delta_yaw] in the 
         * vehicle frame
        */
        void predict(std::vector<double> odom);

        void weight(std::vector<double> odom);

        void resample(double* odom);

        /**
         * Obtain the mean particle of the particle filter, or the state belief.
         * 
         * @return length 4 vector of the form [x, y, z, yaw]
        */
        std::vector<double> getMeanParticle();
    };
}