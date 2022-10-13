#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

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

        double mapXmin;
        double mapXmax;
        double mapYmin;
        double mapYmax;
        VectorXd pWei; 
        VectorXd pAng; 
        MatrixXd pxy;

        // Gaussian Constants
        double valRng;
        double rngSigSq2;
        double valAng;
        double angSigSq2;


        public:
        ParticleFilter() {
            ParticleFilter(500, 0., 0., 0., 0., 0., 0., 0.);
        }

        ParticleFilter(int num, double odomLinSigma, double odomAngSigma, 
                 double measRngNoise, double measAngNoise, double xInit,double yInit,double thetaInit) {
            this->num = num;
            this->odomLinSigma = odomLinSigma;
            this->odomAngSigma = odomAngSigma;
            this->measRngNoise = measRngNoise;
            this->measAngNoise = measAngNoise;
            this->mapXmin = mapXmin;
            this->mapXmax = mapXmax;
            this->mapYmin = mapYmin;
            this->mapYmax = mapYmax;
            VectorXd wei(num);
            this->pWei = wei;
            VectorXd ang(num);
            this->pAng = ang;
            MatrixXd xy(2,num);
            this->pxy = xy;

        }
    };
}