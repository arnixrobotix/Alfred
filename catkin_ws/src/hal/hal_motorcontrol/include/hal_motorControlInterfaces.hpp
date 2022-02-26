#ifndef HAL_MOTORCONTROLINTERFACES
#define HAL_MOTORCONTROLINTERFACES

#include "hal_motorControlVirtuals.hpp"

class MotorControlPublisherRos : public MotorControlPublisher
{
private:
    ros::Publisher motorControlPubRos;

public:
    MotorControlPublisherRos(ros::NodeHandle *node);
    ~MotorControlPublisherRos() = default;
    void publish(hal_motorcontrol::hal_motorcontrolMsg message) override;
};

class MotorControlSubscriberRos : public MotorControlSubscriber
{
private:
    ros::Subscriber motorControlSubRos;
    ros::NodeHandle *nodeHandle;

public:
    MotorControlSubscriberRos(ros::NodeHandle *node);
    ~MotorControlSubscriberRos() = default;
    void subscribe(MotorControl *motorControl) override;
};

class MotorControlClientsRos : public MotorControlClients
{
private:
    ros::ServiceClient gpioSetInputClientRos;
    ros::ServiceClient gpioSetCallbackClientRos;
    ros::ServiceClient gpioSetOutputClientRos;
    ros::ServiceClient gpioSetPwmFrequencyClientRos;
    ros::ServiceClient gpioSetPwmDutycycleClientRos;

public:
    MotorControlClientsRos(ros::NodeHandle *node);
    ~MotorControlClientsRos() = default;
    ros::ServiceClient *getSetInputClientHandle() override;
    ros::ServiceClient *getSetCallbackClientHandle() override;
    ros::ServiceClient *getSetOutputClientHandle() override;
    ros::ServiceClient *getSetPwmFrequencyClientHandle() override;
    ros::ServiceClient *getSetPwmDutycycleClientHandle() override;
};

#endif