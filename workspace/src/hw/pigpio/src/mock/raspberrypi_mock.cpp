#include "mock/raspberrypi_mock.hpp"

RaspberryPi::RaspberryPi() : gpios({})
{
}

void RaspberryPi::addGpio(uint8_t gpioId)
{
    gpioPwm gpioPwm({false, 0, 0});
    gpioCallback gpioCallback({false, 0, neitherEdge});
    gpio newGpio({gpioType::input, gpioResistor::off, gpioLevel::low, gpioPwm, gpioCallback});

    gpios.insert({gpioId, newGpio});
}

void RaspberryPi::setGpioType(uint8_t gpioId, gpioType type)
{
    auto gpio = gpios.find(gpioId);
    if (gpio != gpios.end()) 
    {
        gpio->second.type = type;
    }
}

void RaspberryPi::setGpioResistor(uint8_t gpioId, gpioResistor resistorConfiguration)
{
    auto gpio = gpios.find(gpioId);
    if (gpio != gpios.end()) 
    {
        gpio->second.resistorConfiguration = resistorConfiguration;
    }
}

void RaspberryPi::setGpioLevel(uint8_t gpioId, gpioLevel level)
{
    auto gpio = gpios.find(gpioId);
    if (gpio != gpios.end()) 
    {
        gpio->second.level = level;
    }
}

void RaspberryPi::setGpioPwm(uint8_t gpioId, gpioPwm pwm)
{
    auto gpio = gpios.find(gpioId);
    if (gpio != gpios.end()) 
    {
        gpio->second.pwm = pwm;
    }
}

void RaspberryPi::setGpioCallback(uint8_t gpioId, gpioCallback callback)
{
    auto gpio = gpios.find(gpioId);
    if (gpio != gpios.end()) 
    {
        gpio->second.callback = callback;
    }
}

std::tuple<bool, gpioType> RaspberryPi::getGpioType(uint8_t gpioId)
{
    auto gpio = gpios.find(gpioId);
    if (gpio != gpios.end()) 
    {
        return std::make_tuple(true, gpio->second.type);
    }
    else
    {
        return std::make_tuple(false, gpioType::input);
    }
}

std::tuple<bool, gpioResistor> RaspberryPi::getGpioResistor(uint8_t gpioId)
{
    auto gpio = gpios.find(gpioId);
    if (gpio != gpios.end()) 
    {
        return std::make_tuple(true, gpio->second.resistorConfiguration);
    }
    else
    {
        return std::make_tuple(false, gpioResistor::off);
    }
}

std::tuple<bool, gpioLevel> RaspberryPi::getGpioLevel(uint8_t gpioId)
{
    auto gpio = gpios.find(gpioId);
    if (gpio != gpios.end()) 
    {
        return std::make_tuple(true, gpio->second.level);
    }
    else
    {
        return std::make_tuple(false, gpioLevel::low);
    }
}

std::tuple<bool, gpioPwm> RaspberryPi::getGpioPwm(uint8_t gpioId)
{
    gpioPwm gpioPwm({false, 0, 0});

    auto gpio = gpios.find(gpioId);
    if (gpio != gpios.end()) 
    {
        return std::make_tuple(true, gpio->second.pwm);
    }
    else
    {
        return std::make_tuple(false, gpioPwm);
    }
}

std::tuple<bool, gpioCallback> RaspberryPi::getGpioCallback(uint8_t gpioId)
{
    gpioCallback gpioCallback({false, 0, neitherEdge});

    auto gpio = gpios.find(gpioId);
    if (gpio != gpios.end()) 
    {
        return std::make_tuple(true, gpio->second.callback);
    }
    else
    {
        return std::make_tuple(false, gpioCallback);
    }
}