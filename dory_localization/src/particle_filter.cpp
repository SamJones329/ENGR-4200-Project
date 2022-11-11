#include "../include/particle_filter.hpp"

void DoryLoc::ParticleFilter::setParticleAngles(VectorXd newAngles) {
    this->pYaw = newAngles;
    //TODO - wrap angles
}

DoryLoc::ParticleFilter::ParticleFilter(int num, double measRngNoise, double measYawNoise, 
        double xInit, double yInit,double yawInit) 
        : num(num) 
        , measRngNoise(measRngNoise)
        , measYawNoise(measYawNoise)
        , distribution(0, 1./num)
        , pWei(num)
        , pxyz(ArrayXXd::Zero(3,num))
        , pYaw(num)
        , valRng(1.0 / (measRngNoise * sqrt(2 * M_PI)))
        , rngSigSq2(2. * pow(measRngNoise, 2))
        , valAng(1.0 / (measYawNoise * sqrt(2 * M_PI)))
        , angSigSq2(2. * pow(measYawNoise, 2))
        , uuid(boost::uuids::random_generator()())
    {
    // this->num = num;
    // this->measRngNoise = measRngNoise;
    // this->measYawNoise = measYawNoise;
    // std::uniform_real_distribution<double> distribution(0,1./num);
    // this->distribution = distribution;

    // VectorXd wei(num);
    double startWei = 1. / num;
    for(int i = 0; i < num; i++) {
        pWei(i) = startWei;
    }
    // this->pWei = wei;
    // VectorXd ang(num);
    // this->pYaw = ang;
    // MatrixXd xyz = ArrayXXd::Zero(3,num);
    // this->pxyz = xyz;
    

    // this->valRng = 1.0 / (measRngNoise * sqrt(2 * M_PI));
    // this->rngSigSq2 = 2 * pow(measRngNoise, 2);
    // this->valAng = 1.0 / (measYawNoise * sqrt(2 * M_PI));
    // this->angSigSq2 = 2 * pow(measYawNoise, 2);
    std::cout << uuid << std::endl;

    std::cout << "Constructed Particle Filter with params: " << std::endl;
    std::cout << "num: " << num << std::endl;
    std::cout << "ParticleFilter" << uuid << "@" << this << " pxyz@" << &pxyz << std::endl;
    std::cout << "xyz shape (" << pxyz.rows() << "," << pxyz.cols() << ") @ " << &pxyz << std::endl;
}

void DoryLoc::ParticleFilter::predict(std::vector<double> odom) {
    std::cout << "angdiff: " << odom[3] << std::endl;

    if(odom[0] == 1 && odom[1] == 0 && odom[2] == 0 && odom[3] == 0) {
        this->moving = false;
        return;
    }

    VectorXd odomAngVec(num);
    for(int i = 0; i < this->num; i++) {
        odomAngVec(i) = odom[3];
    }
    // setParticleAngles(this->pYaw + odomAngVec);
    pYaw += odomAngVec;
    Vector3d odomTranslation {odom[0], odom[1], odom[2]};


    // try resizing to vector of matrices so can use vector iterator
    
    VectorXd angCos;
    angCos.resize(num);
    angCos = pYaw.array().cos();
    VectorXd angSin = pYaw.array().sin();
    Matrix<VectorXd, 2, 2> R;// {angCos, -angSin};//,{angSin, angCos}};
    R(0,0) = angCos;
    R(0,1) = -angSin;
    R(1,0) = angSin;
    R(1,1) = angCos;

    // std::cout << "butt: " << R(0).size() << std::endl;

    // std::vector<Eigen::Matrix<double,1,500>> R1;
    // std::vector<Eigen::Matrix<double,1,500>> R2;
    // std::vector<Eigen::Matrix<double,1,500>> R3;
    // std::vector<Eigen::Matrix<double,1,500>> R4;
    // for (int i = 0; i<500; i++) {
    //     R1.push_back(cos(pYaw(i)));
    // }

    


    // vector<Eigen::Matrix<

    Matrix<double, 3, 500> deltaWc; // was dynamic with insertion // = odomTranslation.dot(R);
    for(int i = 0; i < this->num; i++) {
        deltaWc(0,i) = R(0,0)(i) * odomTranslation(0) + R(0,1)(i) * odomTranslation(1);
        deltaWc(1,i) = R(1,0)(i) * odomTranslation(0) + R(1,1)(i) * odomTranslation(1);
        deltaWc(2,i) = 0.;
    }

    // std::cout << "xyz shape before predict (" << this->pxyz.rows() << "," << this->pxyz.cols() << ")" << std::endl;
    this->pxyz += deltaWc;
    // std::cout << "xyz shape after predict (" << this->pxyz.rows() << "," << this->pxyz.cols() << ")" << std::endl;

    // update flag for resampling
    this->moving = true;
}

void DoryLoc::ParticleFilter::weight(std::vector<double> odom) {
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

void DoryLoc::ParticleFilter::resample(double* odom) {
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

std::vector<double> DoryLoc::ParticleFilter::getMeanParticle() {
    // Weighted mean
    // weig = np.vstack((self.p_wei, self.p_wei))
    // mean = np.sum(self.p_xy * weig, axis=1) / np.sum(self.p_wei)
    // // weights should sum to 1
    // std::cout << "Calculating mean particle..." << std::endl;
    // std::cout << "Dims: 0-x, 1-y, 2-z, 3-yaw, ParticleFilter" << uuid << "@" << this << " pxyz@" << &pxyz << std::endl;
    // std::cout << "pWei shape (" << pWei.rows() << "," << pWei.cols() << ")" << std::endl; 
    std::vector<double> mean;
    for(int i = 0; i < 3; i++) {
        // std::cout << "xyz has " << pxyz.rows() << " rows and " << pxyz.cols() << " cols " << std::endl;
        // std::cout << "Getting mean for dim " << i << std::endl;
        VectorXd dim = this->pxyz.row(i);
        // std::cout << "pxyz row " << " shape (" << dim.rows() << "," << dim.cols() << ")" << std::endl;
        // std::cout << "Weighting each particle" << std::endl;
        dim = dim.cwiseProduct(this->pWei);
        // std::cout << "Summing" << std::endl;
        double dimSum = dim.sum();
        // std::cout << "Pushing onto vector" << std::endl;
        mean.push_back(dim.sum());
    }
    // std::cout << "Got xyz ";
    // for(auto i: mean) std::cout << i << ' ';
    // std::cout << std::endl;

    // ang = np.arctan2( np.sum(self.p_wei * np.sin(self.p_ang)) / np.sum(self.p_wei),
                        // np.sum(self.p_wei * np.cos(self.p_ang)) / np.sum(self.p_wei) )
    VectorXd angCos = this->pYaw.array().cos();
    VectorXd angSin = this->pYaw.array().sin();
    // std::cout << "angCos shape (" << angCos.rows() << "," << angCos.cols() << ")" << std::endl;
    double ang = std::atan2(
        (angSin.cwiseProduct(this->pWei)).sum(), 
        (angCos.cwiseProduct(this->pWei)).sum()
    );
    mean.push_back(ang);

    // std::cout << "Got mean particle ";
    // for(auto i: mean) std::cout << i << ' ';
    // std::cout << std::endl;

    // return np.array([mean[0], mean[1], ang])
    return mean;
}