#ifndef _ATMOSPHERE_GOST_4401_81_
#define _ATMOSPHERE_GOST_4401_81_

#include <type_traits>
#include <stdexcept>
#include <cmath>

template <std::floating_point T>
class Atmosphere_GOST_4401_81
{
private:
    static constexpr T RZ = 6356767.0;
    static constexpr T g_c = 9.80665;
    static constexpr T R = 287.05287;

    struct LayerParams
    {
        T T_;
        T H_;
        T beta;
        T p_;
    };

    static LayerParams getLayerParams(T H);
    static inline T calculateGeopotentialHeight(T altitude);

public:
    static T get_g(T altitude);
    static T Temperature(T altitude);
    static T SoundSpeed(T altitude);
    static T Pressure(T altitude);
    static T Density(T altitude);
    static T DynamicPressure(T altitude, T Mach);
};

template <std::floating_point T>
typename Atmosphere_GOST_4401_81<T>::LayerParams Atmosphere_GOST_4401_81<T>::getLayerParams(T H)
{
    if (H >= -2000.0 && H < 0.0)
    {
        return { 301.15, -2000.0, -0.0065, 1.27783E+5 };
    }
    else if (H >= 0.0 && H < 11000.0)
    {
        return { 288.15, 0.0, -0.0065, 1.01325E+5 };
    }
    else if (H >= 11000.0 && H < 20000.0)
    {
        return { 216.65, 11000.0, 0.0, 2.2632E+4 };
    }
    else if (H >= 20000.0 && H < 32000.0)
    {
        return { 216.65, 20000.0, 0.001, 5.47487E+3 };
    }
    else if (H >= 32000.0 && H < 47000.0)
    {
        return { 228.65, 32000.0, 0.0028, 8.68014E+2 };
    }
    else if (H >= 47000.0 && H < 51000.0)
    {
        return { 270.65, 47000.0, 0.0, 1.10906E+2 };
    }
    else if (H >= 51000.0 && H <= 71000.0)
    {
        return { 270.65, 51000.0, -0.0028, 6.6938404E+1 };
    }
    else
    {
        throw std::runtime_error("Exception in typename Atmosphere_GOST_4401_81<T>::LayerParams Atmosphere_GOST_4401_81<T>::getLayerParams(T H) const: value is out of range!");
    }
}

template <std::floating_point T>
inline T Atmosphere_GOST_4401_81<T>::calculateGeopotentialHeight(T altitude)
{
    if ((RZ + altitude) == 0.0)
    {
        throw std::runtime_error("Exception in T Atmosphere_GOST_4401_81<T>::calculateGeopotentialHeight(T altitude) const: division by zero!");
    }
    return RZ * altitude / (RZ + altitude);
}

template <std::floating_point T>
T Atmosphere_GOST_4401_81<T>::Temperature(T altitude)
{
    T H = calculateGeopotentialHeight(altitude);
    LayerParams params = getLayerParams(H);
    return params.T_ + params.beta * (H - params.H_);
}

template <std::floating_point T>
T Atmosphere_GOST_4401_81<T>::SoundSpeed(T altitude)
{
    T temperature = Temperature(altitude);
    if (temperature < 0.0)
    {
        throw std::runtime_error("Exception in T Atmosphere_GOST_4401_81<T>::SoundSpeed(T altitude) const: negative argument in sqrt() function!");
    }
    return 20.046796 * sqrt(temperature);
}

template <std::floating_point T>
T Atmosphere_GOST_4401_81<T>::Pressure(T altitude)
{
    T H = calculateGeopotentialHeight(altitude);
    LayerParams params = getLayerParams(H);
    if (params.beta == 0.0)
    {
        return std::pow(10.0, std::log10(params.p_) - 0.434294 * g_c / (R * params.T_) * (H - params.H_));
    }
    else
    {
        return std::pow(10.0, std::log10(params.p_) - g_c / (params.beta * R) * std::log10((params.T_ + params.beta * (H - params.H_)) / params.T_));
    }
}

template <std::floating_point T>
T Atmosphere_GOST_4401_81<T>::Density(T altitude)
{
    T temperature = Temperature(altitude);
    if (temperature == 0.0)
    {
        throw std::runtime_error("Exception in T Atmosphere_GOST_4401_81<T>::Density(T altitude) const: division by zero!");
    }
    return Pressure(altitude) / (R * temperature);
}

template <std::floating_point T>
T Atmosphere_GOST_4401_81<T>::DynamicPressure(T altitude, T Mach)
{
    T soundSpeed = SoundSpeed(altitude);
    return 0.5 * Density(altitude) * Mach * Mach * soundSpeed * soundSpeed;
}

template <std::floating_point T>
T Atmosphere_GOST_4401_81<T>::get_g(T altitude)
{
    return g_c;
}


#endif // _ATMOSPHERE_GOST_4401_81_