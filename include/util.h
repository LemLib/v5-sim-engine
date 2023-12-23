#pragma once

#include "units.hpp"
#include <vector>
#include "vdml/vdml.h"

namespace sim {
/**
 * @brief Return the mean value of a vector of quantities
 *
 * @param values
 * @return the average
 */
    template <isQuantity Q> Q avg(std::vector<Q> values) {
        Q sum = Q(0);
        for (Q value : values) { sum += value; }
        return sum / values.size();
    }
}