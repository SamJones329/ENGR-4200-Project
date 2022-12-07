#ifndef DORY_PF_H
#define DORY_PF_H

#include <vector>
#include <Eigen/Dense>
#include <random>
#include <cmath>
#include <iostream>
#include <string>
#include "./localizer.hpp"

#define M_PI_X_2 6.283185307179586232

using namespace Eigen;

namespace DoryLoc
{
    class ParticleFilter : public Localizer<4>
    {
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

        ParticleFilter() : ParticleFilter(500, 0.0101, 0.0101, 0., 0., 0., 1.5)
        { //: ParticleFilter(500, 0.2, 0.174533, 0., 0., 0., 1.5) {
          // DVL A50 WL-21035-2 TODO - find out if have standard or performance version
          // standard has long term sensor (velocity) accuracy of +-1.01%, perf has +-0.1%
        }

        ParticleFilter(int num, double measRngNoise, double measYawNoise,
                       double xInit, double yInit, double yawInit, double particleRange)
            : num(num), measRngNoise(measRngNoise), measYawNoise(measYawNoise), distribution(0, 1. / num), pWei(num), pxyz(ArrayXXd::Zero(3, num)), pYaw(num), valRng(1.0 / (measRngNoise * sqrt(2 * M_PI))), rngSigSq2(2. * pow(measRngNoise, 2)), valAng(1.0 / (measYawNoise * sqrt(2 * M_PI))), angSigSq2(2. * pow(measYawNoise, 2))
        {
            std::uniform_real_distribution<double> posSpread(-particleRange, particleRange);
            std::uniform_real_distribution<double> angSpread(-M_PI, M_PI);
            double startWei = 1. / num;
            for (int i = 0; i < num; i++)
            {
                pWei(i) = startWei;
                for (int j = 0; j < 2; j++)
                {
                    pxyz(j, i) = posSpread(generator);
                }
                pYaw(i) = angSpread(generator);
            }
        }

        void setParticleAngles(VectorXd newAngles)
        {
            for (int i = 0; i < num; i++)
            {
                double yaw = newAngles(i);
                while (yaw > M_PI)
                {
                    yaw -= M_PI_X_2;
                }
                while (yaw <= -M_PI)
                {
                    yaw += M_PI_X_2;
                }
                pYaw(i) = yaw;
            }
        }

        /**
         * Moves particles with the given odometry.
         *
         * odom - incremental odometry [delta_x, delta_y, delta_yaw] in the
         * vehicle frame
         */
        void predict(std::vector<double> uk) override
        {
            if (uk[0] == 1 && uk[1] == 0 && uk[2] == 0 && uk[3] == 0)
            {
                this->moving = false;
                return;
            }

            VectorXd odomAngVec(num);

            for (int i = 0; i < this->num; i++)
            {
                odomAngVec(i) = uk[3];
            }
            setParticleAngles(this->pYaw + odomAngVec);
            // pYaw += odomAngVec;
            Vector3d odomTranslation{uk[0], uk[1], uk[2]};

            // try resizing to vector of matrices so can use vector iterator

            VectorXd angCos;
            angCos.resize(num);
            angCos = pYaw.array().cos();
            VectorXd angSin = pYaw.array().sin();
            Matrix<VectorXd, 2, 2> R;
            R(0, 0) = angCos;
            R(0, 1) = -angSin;
            R(1, 0) = angSin;
            R(1, 1) = angCos;

            Matrix<double, 3, 500> deltaWc; // was dynamic with insertion // = odomTranslation.dot(R);
            for (int i = 0; i < this->num; i++)
            {
                deltaWc(0, i) = R(0, 0)(i) * odomTranslation(0) + R(0, 1)(i) * odomTranslation(1);
                deltaWc(1, i) = R(1, 0)(i) * odomTranslation(0) + R(1, 1)(i) * odomTranslation(1);
                deltaWc(2, i) = odomTranslation(2);
            }

            this->pxyz += deltaWc;

            // update flag for resampling
            this->moving = true;
        }

        void update(std::vector<double> zk) override
        {
            this->weight(zk);
            this->resample();
        }

        void weight(std::vector<double> odom)
        {
            VectorXd weights = ArrayXd::Zero(this->num);

            double x = odom[0], y = odom[1], z = odom[2], yaw = odom[3];
            for (int i = 0; i < this->num; i++)
            {
                double px = pxyz(0, i), py = pxyz(1, i), pz = pxyz(2, i), pyaw = pYaw(i);
                double wx = this->valRng * exp(-pow(x - px, 2) / this->rngSigSq2);
                double wy = this->valRng * exp(-pow(y - py, 2) / this->rngSigSq2);
                double wz = this->valRng * exp(-pow(z - pz, 2) / this->rngSigSq2);
                double wyaw = 1 / (measYawNoise * sqrt(2 * M_PI) * exp(-pow(z - odom[2], 2) / (2 * pow(measYawNoise, 2))));
                weights(i) = wx * wy * wz * wyaw;
            }
            double wsum = weights.sum();
            weights /= wsum;
            this->pWei = weights;
        }

        void resample()
        {
            std::uniform_real_distribution<double> resamplingNoise(-measRngNoise, measRngNoise);
            std::uniform_real_distribution<double> resamplingAngNoise(-measYawNoise, measYawNoise);

            VectorXd X(this->num);
            VectorXd Y(this->num);
            VectorXd Z(num);
            VectorXd Th(this->num);

            int M = this->num;
            double iM = 1. / M;
            double c = this->pWei(0);
            int i = 0;
            double r = distribution(generator);
            double U = 0;
            for (int m = 0; m < M; m++)
            {
                U = r + (m - 1) * iM;
                while (U > c)
                {
                    i++;
                    if (i >= this->num)
                    {
                        i--;
                    }
                    c += this->pWei(i);
                }
                X(m) = this->pxyz(0, i) + resamplingNoise(generator);
                Y(m) = this->pxyz(1, i) + resamplingNoise(generator);
                Z(m) = this->pxyz(2, i) + resamplingNoise(generator);
                Th(m) = this->pYaw(i) + resamplingAngNoise(generator);
            }
            pxyz.row(0) = X;
            pxyz.row(1) = Y;
            pxyz.row(2) = Z;
            pYaw = Th;
        }

        /**
         * Obtain the mean particle of the particle filter, or the state belief.
         *
         * @return length 4 vector of the form [x, y, z, yaw]
         */
        std::vector<double> getBelief() override
        {
            // Weighted mean
            // weig = np.vstack((self.p_wei, self.p_wei))
            // mean = np.sum(self.p_xy * weig, axis=1) / np.sum(self.p_wei)
            // weights should sum to 1
            std::vector<double> mean;
            for (int i = 0; i < 3; i++)
            {
                VectorXd dim = this->pxyz.row(i);
                dim = dim.cwiseProduct(this->pWei);
                double dimSum = dim.sum();
                mean.push_back(dim.sum());
            }

            // ang = np.arctan2( np.sum(self.p_wei * np.sin(self.p_ang)) / np.sum(self.p_wei),
            // np.sum(self.p_wei * np.cos(self.p_ang)) / np.sum(self.p_wei) )
            VectorXd angCos = this->pYaw.array().cos();
            VectorXd angSin = this->pYaw.array().sin();
            double ang = std::atan2(
                (angSin.cwiseProduct(this->pWei)).sum(),
                (angCos.cwiseProduct(this->pWei)).sum());
            mean.push_back(ang);

            // return np.array([mean[0], mean[1], ang])
            return mean;
        }

        /**
         * Get all the particles of the filter.
         * @return A vector with all the particles of the particle filter
         * where each vector element is a length 4 array of the form [x, y, z, yaw, weight]
         */
        std::vector<std::vector<double>> getParticles() override
        {
            std::vector<std::vector<double>> particles;
            for (int i = 0; i < num; i++)
            {
                std::vector<double> particle;
                for (int j = 0; j < 3; j++)
                    particle.push_back(pxyz(j, i));
                particle.push_back(pYaw(i));
                particle.push_back(pWei(i));
                particles.push_back(particle);
            }
            return particles;
        }
    };
}

#endif