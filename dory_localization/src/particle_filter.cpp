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
        double measAngNoise;
        default_random_engine generator;
        uniform_real_distribution<double> distribution;

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
            ParticleFilter(500, 0., 0., 0., 0., 0., 0., 0.);
        }

        ParticleFilter(int num, double odomLinSigma, double odomAngSigma, 
                 double measRngNoise, double measAngNoise, double xInit,
                 double yInit,double thetaInit) {
            this->num = num;
            this->odomLinSigma = odomLinSigma;
            this->odomAngSigma = odomAngSigma;
            this->measRngNoise = measRngNoise;
            this->measAngNoise = measAngNoise;
            uniform_real_distribution<double> distribution(0,1./num);
            this->distribution = distribution;
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
            
            
            for(int i = 0; i < this->num; i++) {
                double x = pxyz(0,i);
                double y = pxyz(1,i);
                double z = pxyz(2,i);
                double wx = 1 / ( measRngNoise * sqrt(2*M_PI) * exp(-pow(x-odom[0], 2) / (2*pow(measRngNoise, 2))) );
                double wy = 1 / ( measRngNoise * sqrt(2*M_PI) * exp(-pow(y-odom[1], 2) / (2*pow(measRngNoise, 2))) );
                double wz = 1 / ( measRngNoise * sqrt(2*M_PI) * exp(-pow(z-odom[2], 2) / (2*pow(measRngNoise, 2))) );
                weights(i) = wx + wy + wz;
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