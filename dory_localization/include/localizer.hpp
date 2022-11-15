#include <vector>
#include <iostream>

namespace DoryLoc {
    // template <typename T>
    class Localizer {
    public:
        virtual void predict(std::vector<double> uk); // {std::cout<< "wrong predict" << std::endl;}

        virtual void update(std::vector<double> zk); // {std::cout<< "wrong update" << std::endl;}
        
        virtual std::vector<double> getBelief(); // {std::cout<< "wrong getBelief" << std::endl;}

        virtual std::vector<std::vector<double>> getParticles(); // {std::cout<< "wrong getParticles" << std::endl;}
    };
}