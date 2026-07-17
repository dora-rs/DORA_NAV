//
// Created by xiang on 24-5-8.
//

#include "sampler.h"

namespace lightning::miao {
static std::normal_distribution<double> uni_variate_sampler_(0., 1.);
static std::uniform_real_distribution<double> uniform_real_;
static std::mt19937 gen_real_;

double sampleUniform(double min, double max, std::mt19937* generator) {
    if (generator) {
        return uniform_real_(*generator) * (max - min) + min;
    }

    return uniform_real_(gen_real_) * (max - min) + min;
}

double sampleGaussian(std::mt19937* generator) {
    if (generator) {
        return uni_variate_sampler_(*generator);
    }

    return uni_variate_sampler_(gen_real_);
}

}  // namespace lightning::miao