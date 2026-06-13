#pragma once

#include <cmath>

namespace vnav {

struct ISAData {
    double temperature;
    double pressure;
    double density;
    double speed_of_sound;
};

class InternationalStandardAtmosphere {
public:
    static constexpr double SEA_LEVEL_TEMP = 288.15;
    static constexpr double SEA_LEVEL_PRESSURE = 101325.0;
    static constexpr double SEA_LEVEL_DENSITY = 1.225;
    static constexpr double TROPOPAUSE_ALT = 11000.0;
    static constexpr double TROPOPAUSE_TEMP = 216.65;
    static constexpr double TROPOPAUSE_PRESSURE = 22632.0;
    static constexpr double LAPSE_RATE = -0.0065;
    static constexpr double GAS_CONSTANT = 287.05;
    static constexpr double GRAVITY = 9.80665;
    static constexpr double GAMMA = 1.4;

    static ISAData compute(double altitude_m) {
        ISAData data;
        if (altitude_m <= TROPOPAUSE_ALT) {
            data.temperature = SEA_LEVEL_TEMP + LAPSE_RATE * altitude_m;
            double ratio = data.temperature / SEA_LEVEL_TEMP;
            double exponent = -GRAVITY / (LAPSE_RATE * GAS_CONSTANT);
            data.pressure = SEA_LEVEL_PRESSURE * std::pow(ratio, exponent);
        } else {
            data.temperature = TROPOPAUSE_TEMP;
            double delta_h = altitude_m - TROPOPAUSE_ALT;
            double exponent = -GRAVITY * delta_h / (GAS_CONSTANT * TROPOPAUSE_TEMP);
            data.pressure = TROPOPAUSE_PRESSURE * std::exp(exponent);
        }
        data.density = data.pressure / (GAS_CONSTANT * data.temperature);
        data.speed_of_sound = std::sqrt(GAMMA * GAS_CONSTANT * data.temperature);
        return data;
    }

    static double pressureAltitude(double pressure_pa) {
        if (pressure_pa >= TROPOPAUSE_PRESSURE) {
            double ratio = std::pow(pressure_pa / SEA_LEVEL_PRESSURE,
                                    LAPSE_RATE * GAS_CONSTANT / -GRAVITY);
            return (ratio * SEA_LEVEL_TEMP - SEA_LEVEL_TEMP) / LAPSE_RATE;
        } else {
            return TROPOPAUSE_ALT - (GAS_CONSTANT * TROPOPAUSE_TEMP / GRAVITY) *
                   std::log(pressure_pa / TROPOPAUSE_PRESSURE);
        }
    }

    static double machToTas(double mach, double altitude_m) {
        ISAData data = compute(altitude_m);
        return mach * data.speed_of_sound;
    }

    static double tasToMach(double tas_ms, double altitude_m) {
        ISAData data = compute(altitude_m);
        return tas_ms / data.speed_of_sound;
    }

    static double casToTas(double cas_ms, double altitude_m) {
        ISAData data = compute(altitude_m);
        double dp = SEA_LEVEL_PRESSURE * (std::pow(
            1.0 + (GAMMA - 1.0) / (2.0 * GAMMA) * SEA_LEVEL_DENSITY * cas_ms * cas_ms / SEA_LEVEL_PRESSURE,
            GAMMA / (GAMMA - 1.0)) - 1.0);
        double q = dp / data.pressure;
        return std::sqrt(2.0 * GAMMA / (GAMMA - 1.0) * GAS_CONSTANT * data.temperature *
                         (std::pow(1.0 + q, (GAMMA - 1.0) / GAMMA) - 1.0));
    }
};

}
