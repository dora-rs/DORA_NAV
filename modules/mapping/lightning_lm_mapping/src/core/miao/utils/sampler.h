//
// Created by xiang on 24-5-8.
//

#ifndef SAMPLER_H
#define SAMPLER_H

#include <ctime>
#include <memory>

#include "common/eigen_types.h"
#include "core/common/macros.h"

namespace lightning::miao {

double sampleUniform(double min = 0, double max = 1, std::mt19937* generator = 0);
double sampleGaussian(std::mt19937* generator = 0);

template <class SampleType, class CovarianceType>
class GaussianSampler {
   public:
    DISALLOW_COPY(GaussianSampler)

    explicit GaussianSampler(bool hasGenerator = true) : generator_(hasGenerator ? new std::mt19937 : nullptr) {}
    void setDistribution(const CovarianceType& cov) {
        Eigen::LLT<CovarianceType> cholDecomp;
        cholDecomp.compute(cov);
        if (cholDecomp.info() == Eigen::NumericalIssue) {
            assert(false && "Cholesky decomposition on the covariance matrix failed");
            return;
        }
        cholesky_ = cholDecomp.matrixL();
    }

    //! return a sample of the Gaussian distribution
    SampleType generateSample() {
        SampleType s;
        for (int i = 0; i < s.size(); i++) {
            s(i) = (generator_) ? sampleGaussian(generator_.get()) : sampleGaussian();
        }
        return cholesky_ * s;
    }

    //! seed the random number generator, returns false if not having an own
    //! generator.
    bool seed(unsigned int s) {
        if (!generator_) {
            return false;
        }

        generator_->seed(s);
        return true;
    }

   protected:
    CovarianceType cholesky_;
    std::unique_ptr<std::mt19937> generator_;
};

class Sampler {
   public:
    /**
     * Gaussian random with a mean and standard deviation. Uses the
     * Polar method of Marsaglia.
     */
    static double gaussRand(double mean, double sigma) {
        double y, r2;
        do {
            double x = -1.0 + 2.0 * uniformRand(0.0, 1.0);
            y = -1.0 + 2.0 * uniformRand(0.0, 1.0);
            r2 = x * x + y * y;
        } while (r2 > 1.0 || r2 == 0.0);
        return mean + sigma * y * std::sqrt(-2.0 * log(r2) / r2);
    }

    /**
     * sample a number from a uniform distribution
     */
    static double uniformRand(double lowerBndr, double upperBndr) {
        return lowerBndr + ((double)std::rand() / (RAND_MAX + 1.0)) * (upperBndr - lowerBndr);
    }
    /**
     * default seed function using the current time in seconds
     */
    static void seedRand() { seedRand(static_cast<unsigned int>(std::time(nullptr))); }

    /** seed the random number generator */
    static void seedRand(unsigned int seed) { std::srand(seed); }
};

}  // namespace lightning::miao

#endif  // SAMPLER_H
