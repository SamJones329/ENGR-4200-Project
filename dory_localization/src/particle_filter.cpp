#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <random>

using namespace std;
using namespace Eigen;

namespace DoryLoc {
    class ParticleFilter {
        public:
        int num;
        bool moving = false;
        double nEff = 0;

        private:
        double odomLinSigma;
        double odomAngSigma;
        double measRngNoise;
        default_random_engine generator;
        uniform_real_distribution<double> distribution;
        double measYawNoise;

        double mapXmin;
        double mapXmax;
        double mapYmin;
        double mapYmax;
        VectorXd pWei; 
        VectorXd pYaw; 
        MatrixXd pxyz;

        // Gaussian Constants
        double valRng;
        double rngSigSq2;
        double valAng;
        double angSigSq2;

        void setParticleAngles(VectorXd newAngles) {
            this->pYaw = newAngles;
            //TODO - wrap angles
        }


        public:
        ParticleFilter() {
            // DVL A50 WL-21035-2 TODO - find out if have standard or performance version
            // standard has long term sensor (velocity) accuracy of +-1.01%, perf has +-0.1%
            ParticleFilter(500, 0.0101, 0.0101, 0., 0., 0.);
        }

        ParticleFilter(int num, double measRngNoise, double measYawNoise, 
                double xInit, double yInit,double yawInit) {
            this->num = num;
            this->odomLinSigma = odomLinSigma;
            this->odomAngSigma = odomAngSigma;
            this->measRngNoise = measRngNoise;
            uniform_real_distribution<double> distribution(0,1./num);
            this->distribution = distribution;
            this->measYawNoise = measYawNoise;
            this->mapXmin = mapXmin;
            this->mapXmax = mapXmax;
            this->mapYmin = mapYmin;
            this->mapYmax = mapYmax;
            VectorXd wei(num);
            this->pWei = wei;
            VectorXd ang(num);
            this->pYaw = ang;
            MatrixXd xyz = ArrayXXd::Zero(3,num);
            this->pxyz = xyz;
            

            this->valRng = 1.0 / (measRngNoise * sqrt(2 * M_PI));
            this->rngSigSq2 = 2 * pow(measRngNoise, 2);
            this->valAng = 1.0 / (measYawNoise * sqrt(2 * M_PI));
            this->angSigSq2 = 2 * pow(measYawNoise, 2);
        }

        /**
         * Moves particles with the given odometry.
         * 
         * odom - incremental odometry [delta_x, delta_y, delta_yaw] in the 
         * vehicle frame
        */
        void predict(double* odom) {

            if(odom[0] == 1 && odom[1] == 0 && odom[2] == 0) {
                this->moving = false;
                return;
            }

            VectorXd odomAngVec;
            for(int i = 0; i < this->num; i++) {
                odomAngVec << odom[2];
            }
            setParticleAngles(this->pYaw + odomAngVec);

            Vector2d odomTranslation {odom[0], odom[1]};


            // try resizing to vector of matrices so can use vector iterator
            VectorXd angCos = this->pYaw.array().cos();
            VectorXd angSin = this->pYaw.array().sin();
            Matrix<VectorXd, 2, 2> R;// {angCos, -angSin};//,{angSin, angCos}};
            R(0,0) = angCos;
            R(0,1) = -angSin;
            R(1,0) = angSin;
            R(1,1) = angCos;

            // vector<Eigen::Matrix<

            Matrix<double, 2, Dynamic> deltaWc; // = odomTranslation.dot(R);
            for(int i = 0; i < this->num; i++) {
                deltaWc.row(0) << R(0,0)(i) * odomTranslation(0) + R(0,1)(i) * odomTranslation(1);
                deltaWc.row(1) << R(1,0)(i) * odomTranslation(0) + R(1,1)(i) * odomTranslation(1);
            }

            this->pxyz += deltaWc;

            // update flag for resampling
            this->moving = true;
        }

        void weight(double* odom) {
            VectorXd weights = ArrayXd::Zero(this->num);
            
            double x = odom[0], y = odom[1], z = odom[2], yaw = odom[3];
            for(int i = 0; i < this->num; i++) {
                double px = pxyz(0,i), py = pxyz(1,i), pz = pxyz(2,i), pyaw = pYaw(i);
                double wx = this->valRng * exp(-pow(x-px, 2) / this->rngSigSq2);
                double wy = this->valRng * exp(-pow(y-py, 2) / this->rngSigSq2);
                double wz = this->valRng * exp(-pow(z-pz, 2) / this->rngSigSq2);
                double wyaw = 1 / ( measYawNoise * sqrt(2*M_PI) * exp(-pow(z-odom[2], 2) / (2*pow(measYawNoise, 2))) );
                weights(i) = wx + wy + wz + wyaw;
            }
            double wsum = weights.sum();
            weights /= wsum;
            this->pWei = weights;
        }

        void resample(double* odom) {
            VectorXd X(this->num);

            VectorXd Y(this->num);;

            VectorXd Th(this->num);

            int M = this->num;
            double iM = 1. / M;
            double c = this->pWei(0);
            int i = 0;
            double r = distribution(generator);
            double U = 0;
            for(int m = 0; m < M; m++) {
                U = r + (m-1) * iM;
                while(U > c){
                    i++;
                    if(i >= this->num){
                        i--;
                    }
                    c+= this->pWei(i);
                }
                X(m) = this->pxyz(0,i);
                Y(m) = this->pxyz(1,i);
                Th(m) = this->pYaw(0,i);
            }


        }
    };
}