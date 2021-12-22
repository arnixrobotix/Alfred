#include "ros/ros.h"
#include "std_msgs/String.h"

// Pigpio library
#include <pigpiod_if2.h>

// Services headers (generated)
#include "hal_pigpio/hal_pigpioGetHandle.h"
#include "hal_pigpio/hal_pigpioSetPwmDutycycle.h"
#include "hal_pigpio/hal_pigpioSetPwmFrequency.h"

static int pigpio_handle;

bool setPwmDutycycle(hal_pigpio::hal_pigpioSetPwmDutycycle::Request &req,
                     hal_pigpio::hal_pigpioSetPwmDutycycle::Response &res)
{
    if ((req.dutycycle != 0) && (set_PWM_dutycycle(pigpio_handle, req.gpioId, req.dutycycle) == 0))
    {
        res.result = true;
    }
    else
    {
        res.result = false;
    }
    return true;
}

bool setPwmFrequency(hal_pigpio::hal_pigpioSetPwmFrequency::Request &req,
                     hal_pigpio::hal_pigpioSetPwmFrequency::Response &res)
{
    int pwmSettingResult = set_PWM_dutycycle(pigpio_handle, req.gpioId, req.frequency);

    if ((pwmSettingResult != PI_NOT_PERMITTED) && (pwmSettingResult != PI_BAD_USER_GPIO))
    {
        res.result = true;
    }
    else
    {
        res.result = false;
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_pigpioI2c");
    ros::NodeHandle node;

    ros::ServiceClient getPigpioHandle = node.serviceClient<hal_pigpio::hal_pigpioGetHandle>("hal_pigpioGetHandle");
    hal_pigpio::hal_pigpioGetHandle pigpioHandleRequest;
    getPigpioHandle.call(pigpioHandleRequest);
    pigpio_handle = pigpioHandleRequest.response.handle;

    ros::ServiceServer setPwmDutycycleService = node.advertiseService("hal_pigpioSetPwmDutycycle", setPwmDutycycle);
    ros::ServiceServer setPwmFrequencyService = node.advertiseService("hal_pigpioSetPwmFrequency", setPwmFrequency);

    ros::spin();

    return 0;
}