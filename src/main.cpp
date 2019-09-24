/*
 * MIT License (MIT)
 *
 * Copyright (c) 2018 Dereck Wonnacott <dereck@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 */

#include <iostream>
#include <cmath>
<<<<<<< HEAD
//#define PI 3.14159265358979323846  /* pi */
=======
#define PI 3.14159265358979323846  /* pi */
>>>>>>> dc0cdce06f72e96ee1fd0c51a1837f95ae34f378

// ROS Libraries
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/FluidPressure.h"
#include "std_srvs/Empty.h"
<<<<<<< HEAD
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
# include <tf/transform_broadcaster.h>
#include "vn/conversions.h"
#include "tf/transform_datatypes.h"





=======
>>>>>>> dc0cdce06f72e96ee1fd0c51a1837f95ae34f378

ros::Publisher pubIMU, pubMag, pubGPS, pubOdom, pubTemp, pubPres;
ros::ServiceServer resetOdomSrv;

//Unused covariances initilized to zero's
boost::array<double, 9ul> linear_accel_covariance = { };
boost::array<double, 9ul> angular_vel_covariance = { };
boost::array<double, 9ul> orientation_covariance = { };
XmlRpc::XmlRpcValue rpc_temp;

<<<<<<< HEAD
=======
// Custom user data to pass to packet callback function
struct UserData {
    int device_family;
};

>>>>>>> dc0cdce06f72e96ee1fd0c51a1837f95ae34f378
// Include this header file to get access to VectorNav sensors.
#include "vn/sensors.h"
#include "vn/compositedata.h"
#include "vn/util.h"
<<<<<<< HEAD
#include "vn/conversions.h"
=======
>>>>>>> dc0cdce06f72e96ee1fd0c51a1837f95ae34f378

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

<<<<<<< HEAD
float correction = 0.0;


=======
>>>>>>> dc0cdce06f72e96ee1fd0c51a1837f95ae34f378
// Method declarations for future use.
void BinaryAsyncMessageReceived(void* userData, Packet& p, size_t index);

std::string frame_id;
// Boolean to use ned or enu frame. Defaults to enu which is data format from sensor.
bool tf_ned_to_enu;
// Initial position after getting a GPS fix.
vec3d initial_position;
bool initial_position_set = false;

// Basic loop so we can initilize our covariance parameters above
boost::array<double, 9ul> setCov(XmlRpc::XmlRpcValue rpc){
<<<<<<< HEAD
        // Output covariance vector
        boost::array<double, 9ul> output = { 0.0 };

        // Convert the RPC message to array
        ROS_ASSERT(rpc.getType() == XmlRpc::XmlRpcValue::TypeArray);

        for(int i = 0; i < 9; i++) {
                ROS_ASSERT(rpc[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
                output[i] = (double)rpc[i];
        }
        return output;
=======
    // Output covariance vector
    boost::array<double, 9ul> output = { 0.0 };

    // Convert the RPC message to array
    ROS_ASSERT(rpc.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for(int i = 0; i < 9; i++){
        ROS_ASSERT(rpc[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        output[i] = (double)rpc[i];
    }
    return output;
>>>>>>> dc0cdce06f72e96ee1fd0c51a1837f95ae34f378
}

// Reset initial position to current position
bool resetOdom(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
<<<<<<< HEAD
        initial_position_set = false;
        return true;
}


float yaw_correction;

int main(int argc, char *argv[])
{

        // ROS node init
        ros::init(argc, argv, "vectornav");
        ros::NodeHandle n;
        ros::NodeHandle pn("~");

        pubIMU = n.advertise<sensor_msgs::Imu>("vectornav/IMU", 1000);
        pubMag = n.advertise<sensor_msgs::MagneticField>("vectornav/Mag", 1000);
        pubGPS = n.advertise<sensor_msgs::NavSatFix>("vectornav/GPS", 1000);
        pubOdom = n.advertise<nav_msgs::Odometry>("vectornav/Odom", 1000);
        pubTemp = n.advertise<sensor_msgs::Temperature>("vectornav/Temp", 1000);
        pubPres = n.advertise<sensor_msgs::FluidPressure>("vectornav/Pres", 1000);

        resetOdomSrv = n.advertiseService("reset_odom", resetOdom);

        // Serial Port Settings
        string SensorPort;
        int SensorBaudrate;
        int async_output_rate;

        // Load all params
        pn.param<std::string>("frame_id", frame_id, "vectornav");
        pn.param<bool>("tf_ned_to_enu", tf_ned_to_enu, false);
        pn.param<int>("async_output_rate", async_output_rate, 40);
        pn.param<std::string>("serial_port", SensorPort, "/dev/ttyS0");
        pn.param<int>("serial_baud", SensorBaudrate, 115200);
        pn.getParam("yaw_correction", yaw_correction);

        //Call to set covariances
        if(pn.getParam("linear_accel_covariance",rpc_temp))
        {
                linear_accel_covariance = setCov(rpc_temp);
        }
        if(pn.getParam("angular_vel_covariance",rpc_temp))
        {
                angular_vel_covariance = setCov(rpc_temp);
        }
        if(pn.getParam("orientation_covariance",rpc_temp))
        {
                orientation_covariance = setCov(rpc_temp);
        }

        //This is a place holder for the yaw correction to align imu and global positioning



        ROS_INFO("Connecting to : %s @ %d Baud", SensorPort.c_str(), SensorBaudrate);

        // Create a VnSensor object and connect to sensor
        VnSensor vs;

        // Default baudrate variable
        int defaultBaudrate;
        // Run through all of the acceptable baud rates until we are connected
        // Looping in case someone has changed the default
        bool baudSet = false;
        while(!baudSet) {
                // Make this variable only accessible in the while loop
                static int i = 0;
                defaultBaudrate = vs.supportedBaudrates()[i];
                ROS_INFO("Connecting with default at %d", defaultBaudrate);
                // Default response was too low and retransmit time was too long by default.
                // They would cause errors
                vs.setResponseTimeoutMs(1000); // Wait for up to 1000 ms for response
                vs.setRetransmitDelayMs(50); // Retransmit every 50 ms

                // Acceptable baud rates 9600, 19200, 38400, 57600, 128000, 115200, 230400, 460800, 921600
                // Data sheet says 128000 is a valid baud rate. It doesn't work with the VN100 so it is excluded.
                // All other values seem to work fine.
                try{
                        // Connect to sensor at it's default rate
                        if(defaultBaudrate != 128000 && SensorBaudrate != 128000)
                        {
                                vs.connect(SensorPort, defaultBaudrate);
                                // Issues a change baudrate to the VectorNav sensor and then
                                // reconnects the attached serial port at the new baudrate.
                                vs.changeBaudRate(SensorBaudrate);
                                // Only makes it here once we have the default correct
                                ROS_INFO("Connected baud rate is %d",vs.baudrate());
                                baudSet = true;
                        }
                }
                // Catch all oddities
                catch(...) {
                        // Disconnect if we had the wrong default and we were connected
                        vs.disconnect();
                        ros::Duration(0.2).sleep();
                }
                // Increment the default iterator
                i++;
                // There are only 9 available data rates, if no connection
                // made yet possibly a hardware malfunction?
                if(i > 8)
                {
                        break;
                }
        }

        // Now we verify connection (Should be good if we made it this far)
        if(vs.verifySensorConnectivity())
        {
                ROS_INFO("Device connection established");
        }else{
                ROS_ERROR("No device communication");
                ROS_WARN("Please input a valid baud rate. Valid are:");
                ROS_WARN("9600, 19200, 38400, 57600, 115200, 128000, 230400, 460800, 921600");
                ROS_WARN("With the test IMU 128000 did not work, all others worked fine.");
        }
        // Query the sensor's model number.
        string mn = vs.readModelNumber();
        ROS_INFO("Model Number: %s", mn.c_str());

        // Set Data output Freq [Hz]
        vs.writeAsyncDataOutputFrequency(async_output_rate);

        // Configure binary output message
        BinaryOutputRegister bor(
                ASYNCMODE_PORT1,
                1000 / async_output_rate, // update rate [ms]
                COMMONGROUP_QUATERNION
                | COMMONGROUP_ANGULARRATE
                | COMMONGROUP_POSITION
                | COMMONGROUP_ACCEL
                | COMMONGROUP_MAGPRES,
                TIMEGROUP_NONE,
                IMUGROUP_NONE,
                GPSGROUP_NONE,
                ATTITUDEGROUP_YPRU, //<-- returning yaw pitch roll uncertainties
                INSGROUP_INSSTATUS
                | INSGROUP_POSLLA
                | INSGROUP_POSECEF
                | INSGROUP_VELBODY
                | INSGROUP_ACCELECEF);

        vs.writeBinaryOutput1(bor);

        // Set Data output Freq [Hz]
        vs.writeAsyncDataOutputFrequency(async_output_rate);
        vs.registerAsyncPacketReceivedHandler(NULL, BinaryAsyncMessageReceived);

        // You spin me right round, baby
        // Right round like a record, baby
        // Right round round round
        while (ros::ok())
        {
                ros::spin(); // Need to make sure we disconnect properly. Check if all ok.
        }

        // Node has been terminated
        vs.unregisterAsyncPacketReceivedHandler();
        ros::Duration(0.5).sleep();
        vs.disconnect();
        ros::Duration(0.5).sleep();
        return 0;
=======
    initial_position_set = false;
    return true;
}

int main(int argc, char *argv[])
{

    // ROS node init
    ros::init(argc, argv, "vectornav");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    pubIMU = n.advertise<sensor_msgs::Imu>("vectornav/IMU", 1000);
    pubMag = n.advertise<sensor_msgs::MagneticField>("vectornav/Mag", 1000);
    pubGPS = n.advertise<sensor_msgs::NavSatFix>("vectornav/GPS", 1000);
    pubOdom = n.advertise<nav_msgs::Odometry>("vectornav/Odom", 1000);
    pubTemp = n.advertise<sensor_msgs::Temperature>("vectornav/Temp", 1000);
    pubPres = n.advertise<sensor_msgs::FluidPressure>("vectornav/Pres", 1000);

    resetOdomSrv = n.advertiseService("reset_odom", resetOdom);

    // Serial Port Settings
    string SensorPort;
    int SensorBaudrate;
    int async_output_rate;

    // Sensor IMURATE (800Hz by default, used to configure device)
    int SensorImuRate;

    // Load all params
    pn.param<std::string>("frame_id", frame_id, "vectornav");
    pn.param<bool>("tf_ned_to_enu", tf_ned_to_enu, false);
    pn.param<int>("async_output_rate", async_output_rate, 40);
    pn.param<std::string>("serial_port", SensorPort, "/dev/ttyUSB0");
    pn.param<int>("serial_baud", SensorBaudrate, 115200);
    pn.param<int>("fixed_imu_rate", SensorImuRate, 800);

    //Call to set covariances
    if(pn.getParam("linear_accel_covariance",rpc_temp))
    {
        linear_accel_covariance = setCov(rpc_temp);
    }
    if(pn.getParam("angular_vel_covariance",rpc_temp))
    {
        angular_vel_covariance = setCov(rpc_temp);
    }
    if(pn.getParam("orientation_covariance",rpc_temp))
    {
        orientation_covariance = setCov(rpc_temp);
    }

    ROS_INFO("Connecting to : %s @ %d Baud", SensorPort.c_str(), SensorBaudrate);

    // Create a VnSensor object and connect to sensor
    VnSensor vs;

    // Default baudrate variable
    int defaultBaudrate;
    // Run through all of the acceptable baud rates until we are connected
    // Looping in case someone has changed the default
    bool baudSet = false;
    while(!baudSet){
        // Make this variable only accessible in the while loop
        static int i = 0;
        defaultBaudrate = vs.supportedBaudrates()[i];
        ROS_INFO("Connecting with default at %d", defaultBaudrate);
        // Default response was too low and retransmit time was too long by default.
        // They would cause errors
        vs.setResponseTimeoutMs(1000); // Wait for up to 1000 ms for response
        vs.setRetransmitDelayMs(50);  // Retransmit every 50 ms

        // Acceptable baud rates 9600, 19200, 38400, 57600, 128000, 115200, 230400, 460800, 921600
        // Data sheet says 128000 is a valid baud rate. It doesn't work with the VN100 so it is excluded.
        // All other values seem to work fine.
        try{
            // Connect to sensor at it's default rate
            if(defaultBaudrate != 128000 && SensorBaudrate != 128000)
            {
                vs.connect(SensorPort, defaultBaudrate);
                // Issues a change baudrate to the VectorNav sensor and then
                // reconnects the attached serial port at the new baudrate.
                vs.changeBaudRate(SensorBaudrate);
                // Only makes it here once we have the default correct
                ROS_INFO("Connected baud rate is %d",vs.baudrate());
                baudSet = true;
            }
        }
        // Catch all oddities
        catch(...){
            // Disconnect if we had the wrong default and we were connected
            vs.disconnect();
            ros::Duration(0.2).sleep();
        }
        // Increment the default iterator
        i++;
        // There are only 9 available data rates, if no connection
        // made yet possibly a hardware malfunction?
        if(i > 8)
        {
            break;
        }
    }

    // Now we verify connection (Should be good if we made it this far)
    if(vs.verifySensorConnectivity())
    {
        ROS_INFO("Device connection established");
    }else{
        ROS_ERROR("No device communication");
        ROS_WARN("Please input a valid baud rate. Valid are:");
        ROS_WARN("9600, 19200, 38400, 57600, 115200, 128000, 230400, 460800, 921600");
        ROS_WARN("With the test IMU 128000 did not work, all others worked fine.");
    }
    // Query the sensor's model number.
    string mn = vs.readModelNumber();
    ROS_INFO("Model Number: %s", mn.c_str());

    // Set the device info for passing to the packet callback function
    UserData user_data;
    user_data.device_family = vs.determineDeviceFamily();

    // Set Data output Freq [Hz]
    vs.writeAsyncDataOutputFrequency(async_output_rate);

    // Configure binary output message
    BinaryOutputRegister bor(
            ASYNCMODE_PORT1,
            SensorImuRate / async_output_rate,  // update rate [ms]
            COMMONGROUP_QUATERNION
            | COMMONGROUP_ANGULARRATE
            | COMMONGROUP_POSITION
            | COMMONGROUP_ACCEL
            | COMMONGROUP_MAGPRES,
            TIMEGROUP_NONE,
            IMUGROUP_NONE,
            GPSGROUP_NONE,
            ATTITUDEGROUP_YPRU, //<-- returning yaw pitch roll uncertainties
            INSGROUP_INSSTATUS
            | INSGROUP_POSLLA
            | INSGROUP_POSECEF
            | INSGROUP_VELBODY
            | INSGROUP_ACCELECEF);

    vs.writeBinaryOutput1(bor);

    // Set Data output Freq [Hz]
    vs.writeAsyncDataOutputFrequency(async_output_rate);
    vs.registerAsyncPacketReceivedHandler(&user_data, BinaryAsyncMessageReceived);

    // You spin me right round, baby
    // Right round like a record, baby
    // Right round round round
    while (ros::ok())
    {
        ros::spin(); // Need to make sure we disconnect properly. Check if all ok.
    }

    // Node has been terminated
    vs.unregisterAsyncPacketReceivedHandler();
    ros::Duration(0.5).sleep();
    vs.disconnect();
    ros::Duration(0.5).sleep();
    return 0;
>>>>>>> dc0cdce06f72e96ee1fd0c51a1837f95ae34f378
}

//
// Callback function to process data packet from sensor
//
<<<<<<< HEAD
tf2::Quaternion rotateQuaternion(tf2::Quaternion q_orig, vec3f euclid_rot)
{
        tf2::Quaternion q_rot, q_new;
        q_rot.setRPY(euclid_rot[0],euclid_rot[1],euclid_rot[2]); // form r = x,y,z
        q_new = q_rot * q_orig;
        return q_new.normalize();
}

void BinaryAsyncMessageReceived(void* userData, Packet& p, size_t index)
{
        vn::sensors::CompositeData cd = vn::sensors::CompositeData::parse(p);

        // IMU
        sensor_msgs::Imu msgIMU;
        msgIMU.header.stamp = ros::Time::now();
        msgIMU.header.frame_id = frame_id;





        if (cd.hasQuaternion() && cd.hasAngularRate() && cd.hasAcceleration())
        {


                vec4f q = cd.quaternion();
                //vec4f corrected_q = yaw_correction_quat * q ;
                vec3f ar = cd.angularRate();
                vec3f al = cd.acceleration();





                //Quaternion message comes in as a Yaw (z) Pitch (y) Roll (x) format
                if (tf_ned_to_enu)
                {
                        std::cout << "---------------------------------------------------------------------------------------------------------------------" << std::endl;
                        std::cout << "UNTOUCHED Q " << q[1] << ", " << q[0] << ", " << -q[2] << ", " << q[3] << std::endl;
                        std::cout << "---------------------------------------------------------------------------------------------------------------------" << std::endl;
                        std::cout << "Using ENU "<< (tf_ned_to_enu == 1) << std::endl;


                        tf2::Quaternion imu_trans, q_enu, q_correction;
                        tf2::Quaternion q_orig(q[1],q[0],-q[2],-q[3]); // don't mess with the q values just apply a rotation
                        yaw_correction = 180.0;
                        q_correction.setRPY(deg2rad(yaw_correction), deg2rad(yaw_correction), deg2rad(yaw_correction));

                        vec3f rot;
                        float pitch = deg2rad(0.0), roll = deg2rad(0.0), yaw = deg2rad(0.0);
                        vec3f vn_angles = {yaw, pitch, roll};

                        // convert heading from +-PI to 0 to 2PI
                        vn_angles = quat2YprInRads(q);
                        if (yaw < 0)
                        {
                           std::cout << "Vectonav" << "yaw " << "is" << yaw << "BEFORE TRANSFORMATION" << std::endl;
                           yaw = 2 * M_PI - yaw;
                        }

                        rot= {pitch, roll, yaw};
                        imu_trans = rotateQuaternion(q_orig, rot);
                        imu_trans = (imu_trans * q_correction).normalize();

                        std::cout << "Q COPY: " << q_orig.getX() << ", " << q_orig.getY() << ", " << q_orig.getZ()<< ", "  << q_orig.getW()<< std::endl;
                        std::cout << "Q ROTATED: " << imu_trans.getX() << ", " << imu_trans.getY() << ", " << imu_trans.getZ()<< ", "  << imu_trans.getW()<< std::endl;

                        // update message with transformed data
                        msgIMU.orientation.x = imu_trans.getX();
                        msgIMU.orientation.y = imu_trans.getY();
                        msgIMU.orientation.z = imu_trans.getZ();
                        msgIMU.orientation.w = imu_trans.getW();

                        // msgIMU.orientation.x = q[1];
                        // msgIMU.orientation.y = q[0];
                        // msgIMU.orientation.z = -q[2];
                        // msgIMU.orientation.w = q[3];


                        // Flip x and y then invert z
                        msgIMU.angular_velocity.x = ar[0];
                        msgIMU.angular_velocity.y = -ar[1];
                        msgIMU.angular_velocity.z = -ar[2];

                        // Flip x and y then invert z
                        msgIMU.linear_acceleration.x = al[0];
                        msgIMU.linear_acceleration.y = -al[1];
                        msgIMU.linear_acceleration.z = -al[2];

                        if (cd.hasAttitudeUncertainty())
                        {
                                vec3f orientationStdDev = cd.attitudeUncertainty();
                                msgIMU.orientation_covariance[0] = orientationStdDev[1]*orientationStdDev[1]*M_PI/180; // Convert to radians Pitch
                                msgIMU.orientation_covariance[4] = orientationStdDev[0]*orientationStdDev[0]*M_PI/180; // Convert to radians Roll
                                msgIMU.orientation_covariance[8] = orientationStdDev[2]*orientationStdDev[2]*M_PI/180; // Convert to radians Yaw
                        }

                }
                else
                {

                        // The rpy defition for correction
                        tf2::Quaternion imu_correction,imu_corrected;
                        imu_correction.setRPY(0,0,correction); // add 90 degrees to red lin ??g


                        tf2::Quaternion imu_data(q[0],q[1],-q[2],q[3]);


                        std::cout << "IMU BEFORE CORRECTION: " << tf2::getYaw(imu_data) << std::endl;

                        imu_corrected = imu_correction * imu_data;

                        // imu_corrected.normalize();
                        std::cout << "IMU AFTER CORRECTION: " << tf2::getYaw(imu_corrected) <<std::endl;

                        msgIMU.orientation.x = imu_corrected.getX();
                        msgIMU.orientation.y = imu_corrected.getY();
                        msgIMU.orientation.z = imu_corrected.getZ();
                        msgIMU.orientation.w = imu_corrected.getW();

                        std::cout << "Appllying correction: "<<std::endl;

                        // msgIMU.orientation.x = q[0];
                        // msgIMU.orientation.y = q[1];
                        // msgIMU.orientation.z = q[2];
                        // msgIMU.orientation.w = q[3];
                        //
                        if (cd.hasAttitudeUncertainty())
                        {
                                vec3f orientationStdDev = cd.attitudeUncertainty();
                                msgIMU.orientation_covariance[0] = orientationStdDev[2]*orientationStdDev[2]*M_PI/180; // Convert to radians Roll
                                msgIMU.orientation_covariance[4] = orientationStdDev[1]*orientationStdDev[1]*M_PI/180; // Convert to radians Pitch
                                msgIMU.orientation_covariance[8] = orientationStdDev[0]*orientationStdDev[0]*M_PI/180; // Convert to radians Yaw
                        }
                        msgIMU.angular_velocity.x = ar[0];
                        msgIMU.angular_velocity.y = ar[1];
                        msgIMU.angular_velocity.z = ar[2];
                        msgIMU.linear_acceleration.x = al[0];
                        msgIMU.linear_acceleration.y = al[1];
                        msgIMU.linear_acceleration.z = al[2];
                }
                // Covariances pulled from parameters
                msgIMU.angular_velocity_covariance = angular_vel_covariance;
                msgIMU.linear_acceleration_covariance = linear_accel_covariance;
                pubIMU.publish(msgIMU);
        }

        // Magnetic Field
        if (cd.hasMagnetic())
        {
                vec3f mag = cd.magnetic();
                sensor_msgs::MagneticField msgMag;
                msgMag.header.stamp = msgIMU.header.stamp;
                msgMag.header.frame_id = msgIMU.header.frame_id;
                msgMag.magnetic_field.x = mag[0];
                msgMag.magnetic_field.y = mag[1];
                msgMag.magnetic_field.z = mag[2];
                pubMag.publish(msgMag);
        }

        // GPS
        //if (cd.insStatus() == INSSTATUS_GPS_FIX)
        //{
=======
void BinaryAsyncMessageReceived(void* userData, Packet& p, size_t index)
{
    vn::sensors::CompositeData cd = vn::sensors::CompositeData::parse(p);
    UserData user_data = *static_cast<UserData*>(userData);

    // IMU
    sensor_msgs::Imu msgIMU;
    msgIMU.header.stamp = ros::Time::now();
    msgIMU.header.frame_id = frame_id;

    if (cd.hasQuaternion() && cd.hasAngularRate() && cd.hasAcceleration())
    {

        vec4f q = cd.quaternion();
        vec3f ar = cd.angularRate();
        vec3f al = cd.acceleration();

        //Quaternion message comes in as a Yaw (z) Pitch (y) Roll (x) format
        if (tf_ned_to_enu)
        {
            // Flip x and y then invert z
            msgIMU.orientation.x = q[1];
            msgIMU.orientation.y = q[0];
            msgIMU.orientation.z = -q[2];
            msgIMU.orientation.w = q[3];

            if (cd.hasAttitudeUncertainty())
            {
                vec3f orientationStdDev = cd.attitudeUncertainty();
                msgIMU.orientation_covariance[0] = orientationStdDev[1]*orientationStdDev[1]*PI/180; // Convert to radians Pitch
                msgIMU.orientation_covariance[4] = orientationStdDev[0]*orientationStdDev[0]*PI/180; // Convert to radians Roll
                msgIMU.orientation_covariance[8] = orientationStdDev[2]*orientationStdDev[2]*PI/180; // Convert to radians Yaw
            }
            // Flip x and y then invert z
            msgIMU.angular_velocity.x = ar[1];
            msgIMU.angular_velocity.y = ar[0];
            msgIMU.angular_velocity.z = -ar[2];
            // Flip x and y then invert z
            msgIMU.linear_acceleration.x = al[1];
            msgIMU.linear_acceleration.y = al[0];
            msgIMU.linear_acceleration.z = -al[2];
        }
        else
        {
            msgIMU.orientation.x = q[0];
            msgIMU.orientation.y = q[1];
            msgIMU.orientation.z = q[2];
            msgIMU.orientation.w = q[3];

            if (cd.hasAttitudeUncertainty())
            {
                vec3f orientationStdDev = cd.attitudeUncertainty();
                msgIMU.orientation_covariance[0] = orientationStdDev[2]*orientationStdDev[2]*PI/180; // Convert to radians Roll
                msgIMU.orientation_covariance[4] = orientationStdDev[1]*orientationStdDev[1]*PI/180; // Convert to radians Pitch
                msgIMU.orientation_covariance[8] = orientationStdDev[0]*orientationStdDev[0]*PI/180; // Convert to radians Yaw
            }
            msgIMU.angular_velocity.x = ar[0];
            msgIMU.angular_velocity.y = ar[1];
            msgIMU.angular_velocity.z = ar[2];
            msgIMU.linear_acceleration.x = al[0];
            msgIMU.linear_acceleration.y = al[1];
            msgIMU.linear_acceleration.z = al[2];
        }
        // Covariances pulled from parameters
        msgIMU.angular_velocity_covariance = angular_vel_covariance;
        msgIMU.linear_acceleration_covariance = linear_accel_covariance;
        pubIMU.publish(msgIMU);
    }

    // Magnetic Field
    if (cd.hasMagnetic())
    {
        vec3f mag = cd.magnetic();
        sensor_msgs::MagneticField msgMag;
        msgMag.header.stamp = msgIMU.header.stamp;
        msgMag.header.frame_id = msgIMU.header.frame_id;
        msgMag.magnetic_field.x = mag[0];
        msgMag.magnetic_field.y = mag[1];
        msgMag.magnetic_field.z = mag[2];
        pubMag.publish(msgMag);
    }

    // GPS
    if (user_data.device_family != VnSensor::Family::VnSensor_Family_Vn100)
    {
>>>>>>> dc0cdce06f72e96ee1fd0c51a1837f95ae34f378
        vec3d lla = cd.positionEstimatedLla();

        sensor_msgs::NavSatFix msgGPS;
        msgGPS.header.stamp = msgIMU.header.stamp;
        msgGPS.header.frame_id = msgIMU.header.frame_id;
        msgGPS.latitude = lla[0];
        msgGPS.longitude = lla[1];
        msgGPS.altitude = lla[2];
        pubGPS.publish(msgGPS);

        // Odometry
        if (pubOdom.getNumSubscribers() > 0)
        {
<<<<<<< HEAD
                nav_msgs::Odometry msgOdom;
                msgOdom.header.stamp = msgIMU.header.stamp;
                msgOdom.header.frame_id = msgIMU.header.frame_id;
                vec3d pos = cd.positionEstimatedEcef();

                if (!initial_position_set)
                {
                        initial_position_set = true;
                        initial_position.x = pos[0];
                        initial_position.y = pos[1];
                        initial_position.z = pos[2];
                }

                msgOdom.pose.pose.position.x = pos[0] - initial_position[0];
                msgOdom.pose.pose.position.y = pos[1] - initial_position[1];
                msgOdom.pose.pose.position.z = pos[2] - initial_position[2];

                if (cd.hasQuaternion())
                {


                        vec4f q = cd.quaternion();
                        // The rpy defition for correction


                        tf2::Quaternion imu_data(q[0],q[1],-q[2],q[3]);

                        std::cout << "has quaternion: " << tf2::getYaw(imu_data) << std::endl;

                        tf2::Quaternion imu_correction,imu_corrected;
                        imu_correction.setRPY(0,0, correction); // add 90 degrees to red lin ??g


                        std::cout << "IMU BEFORE CORRECTION: " << tf2::getYaw(imu_data) << std::endl;

                        imu_corrected = imu_correction * imu_data;

                        // imu_corrected.normalize();
                        std::cout << "IMU AFTER CORRECTION: " << tf2::getYaw(imu_corrected) <<std::endl;

                        msgIMU.orientation.x = imu_corrected.getX();
                        msgIMU.orientation.y = imu_corrected.getY();
                        msgIMU.orientation.z = imu_corrected.getZ();
                        msgIMU.orientation.w = imu_corrected.getW();

                        std::cout << "Apllying correction: "<<std::endl;
                        //
                        // msgOdom.pose.pose.orientation.x = q[0];
                        // msgOdom.pose.pose.orientation.y = q[1];
                        // msgOdom.pose.pose.orientation.z = q[2];
                        // msgOdom.pose.pose.orientation.w = q[3];
                }
                if (cd.hasVelocityEstimatedBody())
                {
                        vec3f vel = cd.velocityEstimatedBody();

                        msgOdom.twist.twist.linear.x = vel[0];
                        msgOdom.twist.twist.linear.y = vel[1];
                        msgOdom.twist.twist.linear.z = vel[2];
                }
                if (cd.hasAngularRate())
                {
                        vec3f ar = cd.angularRate();

                        msgOdom.twist.twist.angular.x = ar[0];
                        msgOdom.twist.twist.angular.y = ar[1];
                        msgOdom.twist.twist.angular.z = ar[2];
                }
                pubOdom.publish(msgOdom);
        }
        //}

        // Temperature
        if (cd.hasTemperature())
        {
                float temp = cd.temperature();

                sensor_msgs::Temperature msgTemp;
                msgTemp.header.stamp = msgIMU.header.stamp;
                msgTemp.header.frame_id = msgIMU.header.frame_id;
                msgTemp.temperature = temp;
                pubTemp.publish(msgTemp);
        }

        // Barometer
        if (cd.hasPressure())
        {
                float pres = cd.pressure();

                sensor_msgs::FluidPressure msgPres;
                msgPres.header.stamp = msgIMU.header.stamp;
                msgPres.header.frame_id = msgIMU.header.frame_id;
                msgPres.fluid_pressure = pres;
                pubPres.publish(msgPres);
        }
=======
            nav_msgs::Odometry msgOdom;
            msgOdom.header.stamp = msgIMU.header.stamp;
            msgOdom.header.frame_id = msgIMU.header.frame_id;
            vec3d pos = cd.positionEstimatedEcef();

            if (!initial_position_set)
            {
                initial_position_set = true;
                initial_position.x = pos[0];
                initial_position.y = pos[1];
                initial_position.z = pos[2];
            }

            msgOdom.pose.pose.position.x = pos[0] - initial_position[0];
            msgOdom.pose.pose.position.y = pos[1] - initial_position[1];
            msgOdom.pose.pose.position.z = pos[2] - initial_position[2];

            if (cd.hasQuaternion())
            {
                vec4f q = cd.quaternion();

                msgOdom.pose.pose.orientation.x = q[0];
                msgOdom.pose.pose.orientation.y = q[1];
                msgOdom.pose.pose.orientation.z = q[2];
                msgOdom.pose.pose.orientation.w = q[3];
            }
            if (cd.hasVelocityEstimatedBody())
            {
                vec3f vel = cd.velocityEstimatedBody();

                msgOdom.twist.twist.linear.x = vel[0];
                msgOdom.twist.twist.linear.y = vel[1];
                msgOdom.twist.twist.linear.z = vel[2];
            }
            if (cd.hasAngularRate())
            {
                vec3f ar = cd.angularRate();

                msgOdom.twist.twist.angular.x = ar[0];
                msgOdom.twist.twist.angular.y = ar[1];
                msgOdom.twist.twist.angular.z = ar[2];
            }
            pubOdom.publish(msgOdom);
        }
    }

    // Temperature
    if (cd.hasTemperature())
    {
        float temp = cd.temperature();

        sensor_msgs::Temperature msgTemp;
        msgTemp.header.stamp = msgIMU.header.stamp;
        msgTemp.header.frame_id = msgIMU.header.frame_id;
        msgTemp.temperature = temp;
        pubTemp.publish(msgTemp);
    }

    // Barometer
    if (cd.hasPressure())
    {
        float pres = cd.pressure();

        sensor_msgs::FluidPressure msgPres;
        msgPres.header.stamp = msgIMU.header.stamp;
        msgPres.header.frame_id = msgIMU.header.frame_id;
        msgPres.fluid_pressure = pres;
        pubPres.publish(msgPres);
    }
>>>>>>> dc0cdce06f72e96ee1fd0c51a1837f95ae34f378
}
