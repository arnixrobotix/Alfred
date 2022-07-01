#include "hal_imuI2cInit.hpp"
#include "hal_imuI2cInitInterfaces.hpp"

/* Publisher interface implementation */
ImuI2cInitPublisherRos::ImuI2cInitPublisherRos(ros::NodeHandle *node) : imuI2cInitPubRos(node->advertise<hal_imu::hal_imuI2cHeartbeatMsg>("hal_imuI2cHeartbeatMsg", 1000))
{
}

void ImuI2cInitPublisherRos::publish(hal_imu::hal_imuI2cHeartbeatMsg message)
{
    imuI2cInitPubRos.publish(message);
}

/* Subscriber interface implementation */
ImuI2cInitSubscribersRos::ImuI2cInitSubscribersRos(ros::NodeHandle *node) : nodeHandle(node)
{
}

void ImuI2cInitSubscribersRos::subscribe(ImuI2cInit *imuI2cInit)
{
    imuI2cInitPigpioHBSubRos = nodeHandle->subscribe("hal_pigpioHeartbeat", 1000, &ImuI2cInit::pigpioHeartbeatCallback, imuI2cInit);
}

/* Clients interface implementation */
ImuI2cInitClientsRos::ImuI2cInitClientsRos(ros::NodeHandle *node) : i2cOpenClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cOpen>("hal_pigpioI2cOpen")),
                                                                    i2cCloseClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cClose>("hal_pigpioI2cClose")),
                                                                    i2cReadByteDataClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cReadByteData>("hal_pigpioI2cReadByteData")),
                                                                    i2cWriteByteDataClientRos(node->serviceClient<hal_pigpio::hal_pigpioI2cWriteByteData>("hal_pigpioI2cWriteByteData"))
{
}

ros::ServiceClient *ImuI2cInitClientsRos::getI2cOpenHandle()
{
    return &i2cOpenClientRos;
}

ros::ServiceClient *ImuI2cInitClientsRos::getI2cCloseHandle()
{
    return &i2cCloseClientRos;
}

ros::ServiceClient *ImuI2cInitClientsRos::getI2cReadByteDataClientHandle()
{
    return &i2cReadByteDataClientRos;
}

ros::ServiceClient *ImuI2cInitClientsRos::getI2cWriteByteDataClientHandle()
{
    return &i2cWriteByteDataClientRos;
}

/* Servers interface implementation */
ImuI2cInitServersRos::ImuI2cInitServersRos(ros::NodeHandle *node) : nodeHandle(node)
{
}

void ImuI2cInitServersRos::advertiseGetHandleService(ImuI2cInit *imuI2cInit)
{
    imuGetHandleServerRos = nodeHandle->advertiseService("hal_imuGetHandle", &ImuI2cInit::getHandle, imuI2cInit);
}

/* Imu I2c Init implementation */
ImuI2cInit::ImuI2cInit(ImuI2cInitClients *imuI2cInitServiceClients, ImuI2cInitServers *imuI2cInitServiceServers, ImuI2cInitPublisher *imuI2cInitPublisher, ImuI2cInitSubscribers *imuI2cInitSubscribers) : imuI2cInitClients(imuI2cInitServiceClients),
                                                                                                                                                                                                           imuI2cInitServers(imuI2cInitServiceServers),
                                                                                                                                                                                                           imuI2cInitPub(imuI2cInitPublisher),
                                                                                                                                                                                                           imuI2cInitSubs(imuI2cInitSubscribers),
                                                                                                                                                                                                           pigpioNodeStarted(false),
                                                                                                                                                                                                           isStarted(false)
{
    imuI2cInitServiceServers->advertiseGetHandleService(this);
    imuI2cInitSubs->subscribe(this);
}

ImuI2cInit::~ImuI2cInit()
{
    hal_pigpio::hal_pigpioI2cClose i2cCloseSrv;
    i2cCloseSrv.request.handle = imuHandle;
    imuI2cInitClients->getI2cCloseHandle()->call(i2cCloseSrv);
}

bool ImuI2cInit::getHandle(hal_imu::hal_imuGetHandle::Request &req,
                           hal_imu::hal_imuGetHandle::Response &res)
{
    res.handle = imuHandle;
    return true;
}

void ImuI2cInit::initI2cCommunication(void)
{
    hal_pigpio::hal_pigpioI2cOpen i2cOpenSrv;

    i2cOpenSrv.request.bus = IMU_I2C_BUS;
    i2cOpenSrv.request.address = MPU6050_I2C_ADDRESS;

    imuI2cInitClients->getI2cOpenHandle()->call(i2cOpenSrv);
    if (i2cOpenSrv.response.hasSucceeded)
    {
        imuHandle = i2cOpenSrv.response.handle;
        ROS_INFO("Handle %u received for communication with device %u on bus %u.", imuHandle, i2cOpenSrv.request.address, i2cOpenSrv.request.bus);
    }
    else
    {
        ROS_ERROR("Unable to receive handle for communication with device %u on bus %u!", i2cOpenSrv.request.address, i2cOpenSrv.request.bus);
    }
}

void ImuI2cInit::pigpioHeartbeatCallback(const hal_pigpio::hal_pigpioHeartbeatMsg &msg)
{
    pigpioNodeStarted = msg.isAlive;
}

bool ImuI2cInit::isPigpioNodeStarted(void)
{
    return pigpioNodeStarted;
}

void ImuI2cInit::publishHeartbeat(const ros::TimerEvent &timerEvent)
{
    hal_imu::hal_imuI2cHeartbeatMsg heartbeat;
    heartbeat.isAlive = true;
    imuI2cInitPub->publish(heartbeat);
}

void ImuI2cInit::starts(void)
{
    isStarted = true;
}

bool ImuI2cInit::isNotStarted(void)
{
    return !isStarted;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_imuI2cInit");
    ros::NodeHandle node;
    ros::Timer heartbeatTimer;

    ImuI2cInitClientsRos imuI2cInitServiceClients(&node);
    ImuI2cInitServersRos imuI2cInitServers(&node);
    ImuI2cInitPublisherRos imuI2cInitPublishers(&node);
    ImuI2cInitSubscribersRos imuI2cInitSubscribers(&node);

    ImuI2cInit imuI2cInit(&imuI2cInitServiceClients, &imuI2cInitServers, &imuI2cInitPublishers, &imuI2cInitSubscribers);

    ROS_INFO("imuI2cInit node waiting for pigpio node to start...");
    while (ros::ok())
    {
        if (imuI2cInit.isNotStarted() && imuI2cInit.isPigpioNodeStarted())
        {
            ROS_INFO("imuI2cInit node initialising...");
            imuI2cInit.initI2cCommunication();
            imuI2cInit.starts();
            heartbeatTimer = node.createTimer(ros::Duration(0.1), &ImuI2cInit::publishHeartbeat, &imuI2cInit);
            ROS_INFO("imuI2cInit node initialised.");
        }

        ros::spinOnce();
    }

    return 0;
}