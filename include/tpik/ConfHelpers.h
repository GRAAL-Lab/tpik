#include <iostream>
#include <libconfig.h++>

namespace tpik {

namespace tc
{
const char* const none = "\033[0m";
const char* const redL = "\033[1;31m";
}

/**
 * @brief SetParam functor
 *
 * An utility templated functor to set a pram from ConfigFIle
 */
template <typename A>
bool GetParam(const libconfig::Setting& confObj, A& param, const std::string& name) noexcept(false)
{
    if (!confObj.lookupValue(name, param)){
        std::cerr << tc::redL << "GetParam() Error: <" << name << "> lookup failed." << tc::none << std::endl;
        return false;
    }


    return true;
}

template <typename A>
bool GetParam(const libconfig::Config& confObj, A& param, const std::string& name) noexcept(false)
{
    if (!confObj.lookupValue(name, param)){
        std::cerr << tc::redL << "GetParam() Error: <" << name << "> lookup failed." << tc::none << std::endl;
        return false;
    }

    return true;
}

/**
 * @brief SetParam functor
 *
 * An utility templated functor to set a vector pram from ConfigFIle
 */
template <typename A>
bool GetParamVector(const libconfig::Setting& confObj, A& param, const std::string& name) noexcept(false)
{
    try {
        const libconfig::Setting& settings = confObj.lookup(name);
        param.resize(settings.getLength());
        for (int n = 0; n < settings.getLength(); n++) {

            param(n) = settings[n];
        }
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << tc::redL << "GetParamVector() Error: <" << name << "> lookup failed." << std::endl;
        return false;
    }


    return true;
}

template <typename A>
bool GetParamVector(const libconfig::Config& confObj, A& param, const std::string& name) noexcept(false)
{
    try {
        const libconfig::Setting& settings = confObj.lookup(name);
        param.resize(settings.getLength());
        for (int n = 0; n < settings.getLength(); n++) {

            param(n) = settings[n];
        }
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << tc::redL << "GetParamVector() Error: <" << name << "> lookup failed." << std::endl;
        return false;
    }


    return true;
}

}