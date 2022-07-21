#include "hal_imuI2cInit.hpp"

using namespace std::placeholders;

ImuI2cInit::ImuI2cInit() : rclcpp_lifecycle::LifecycleNode("hal_imuI2cInit_node"),
                           imuHandle(MPU6050_I2C_NO_HANDLE)
{
}

LifecycleCallbackReturn_t ImuI2cInit::on_configure(const rclcpp_lifecycle::State & previous_state)
{
    i2cOpenClient = this->create_client<hal_pigpio_interfaces::srv::HalPigpioI2cOpen>("hal_pigpioI2cOpen");
    i2cCloseClient = this->create_client<hal_pigpio_interfaces::srv::HalPigpioI2cClose>("hal_pigpioI2cClose");

    getHandleService = this->create_service<hal_imu_interfaces::srv::HalImuGetHandle>("hal_imuGetHandle", std::bind(&ImuI2cInit::getHandle, this, _1, _2));

    RCLCPP_INFO(get_logger(), "hal_imuI2cInit node configured!");

    return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t ImuI2cInit::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    initI2cCommunication();

    RCLCPP_INFO(get_logger(), "hal_imuI2cInit node activated!");

    return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t ImuI2cInit::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    auto i2cCloseRequest = std::make_shared<hal_pigpio_interfaces::srv::HalPigpioI2cClose::Request>();

    i2cCloseRequest->handle = imuHandle;

    auto i2cCloseCallback = [this](I2cCloseFuture_t future) 
    {
        if(!future.get()->has_succeeded)
        {
            RCLCPP_ERROR(get_logger(), "Error while closing I2C communication with IMU");
        }
    };
    auto i2cCloseFuture = i2cCloseClient->async_send_request(i2cCloseRequest, i2cCloseCallback);

    imuHandle = MPU6050_I2C_NO_HANDLE;

    RCLCPP_INFO(get_logger(), "hal_imuI2cInit node deactivated!");

    return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t ImuI2cInit::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(get_logger(), "hal_imuI2cInit node unconfigured!");

    return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t ImuI2cInit::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
    if(previous_state.label().c_str() == "Active")
    {
        auto i2cCloseRequest = std::make_shared<hal_pigpio_interfaces::srv::HalPigpioI2cClose::Request>();

        i2cCloseRequest->handle = imuHandle;

        auto i2cCloseCallback = [this](I2cCloseFuture_t future) 
        {
            if(!future.get()->has_succeeded)
            {
                RCLCPP_ERROR(get_logger(), "Error while closing I2C communication with IMU");
            }
        };
        auto i2cCloseFuture = i2cCloseClient->async_send_request(i2cCloseRequest, i2cCloseCallback);

        imuHandle = MPU6050_I2C_NO_HANDLE;
    }

    RCLCPP_INFO(get_logger(), "hal_imuI2cInit node shutdown!");

    return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t ImuI2cInit::on_error(const rclcpp_lifecycle::State & previous_state)
{
    return LifecycleCallbackReturn_t::FAILURE;
}

void ImuI2cInit::getHandle(const std::shared_ptr<hal_imu_interfaces::srv::HalImuGetHandle::Request> request,
                           std::shared_ptr<hal_imu_interfaces::srv::HalImuGetHandle::Response> response)
{
    (void)request;

    response->handle = imuHandle;
}

void ImuI2cInit::initI2cCommunication(void)
{
    auto i2cOpenRequest = std::make_shared<hal_pigpio_interfaces::srv::HalPigpioI2cOpen::Request>();

    i2cOpenRequest->bus = IMU_I2C_BUS;
    i2cOpenRequest->address = MPU6050_I2C_ADDRESS;

    auto i2cOpenCallback = [this](I2cOpenFuture_t future) 
    {
        if(future.get()->has_succeeded)
        {
            imuHandle = future.get()->handle;
            RCLCPP_INFO(get_logger(), "IMU I2C handle received.");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Unable to receive IMU I2C handle!");
        }   
    };
    auto i2cOpenFuture = i2cOpenClient->async_send_request(i2cOpenRequest, i2cOpenCallback);
}