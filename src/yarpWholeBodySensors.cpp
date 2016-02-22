/*
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * email: marco.randazzo@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "yarpWholeBodySensors.h"
#include "yarpWbiUtil.h"

#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <string>
#include <sstream>
#include <cassert>

#include <yarp/os/Log.h>

using namespace std;
using namespace wbi;
using namespace yarpWbi;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::skinDynLib;
using namespace iCub::ctrl;

#define MAX_NJ 20 ///< Maximum number of joints for body part (used for buffers to avoid dynamic memory allocation)
#define WAIT_TIME 0.001
#define BLOCKING_SENSOR_TIMEOUT 0.1
#define INITIAL_TIMESTAMP -1000.0

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          YARP WHOLE BODY SENSORS
// *********************************************************************************************************************
// *********************************************************************************************************************
yarpWholeBodySensors::yarpWholeBodySensors(const char* _name, const yarp::os::Property & opt):
initDone(false), name(_name), wbi_yarp_properties(opt), sensorIdList(wbi::SENSOR_TYPE_SIZE)
{
}

bool yarpWholeBodySensors::setYarpWbiProperties(const yarp::os::Property & yarp_wbi_properties)
{
    wbi_yarp_properties = yarp_wbi_properties;
    return true;
}

bool yarpWholeBodySensors::getYarpWbiProperties(yarp::os::Property & yarp_wbi_properties)
{
    yarp_wbi_properties = wbi_yarp_properties;
    return true;
}


bool yarpWholeBodySensors::init()
{
    if( initDone ) return true;

    //Get encoders
    #ifndef NDEBUG
    std::cout << "[INFO] yarpWholeBodySensors: initializing with " << sensorIdList[wbi::SENSOR_ENCODER_POS].size() << " encoders " <<
                                                               sensorIdList[wbi::SENSOR_PWM].size() << " pwm sensors " <<
                                                               sensorIdList[wbi::SENSOR_FORCE_TORQUE].size() << " F/T sensors " <<
                                                               sensorIdList[wbi::SENSOR_IMU].size() << " IMUs " << std::endl;
    #endif
    std::cout << "[INFO] yarpWholeBodySensors: initializing with " << sensorIdList[wbi::SENSOR_ENCODER_POS].size() << " encoders " <<
                                                               sensorIdList[wbi::SENSOR_PWM].size() << " pwm sensors " <<
                                                               sensorIdList[wbi::SENSOR_FORCE_TORQUE].size() << " F/T sensors " <<
                                                               sensorIdList[wbi::SENSOR_IMU].size() << " IMUs " <<
                                                               sensorIdList[wbi::SENSOR_ACCELEROMETER].size() << " Accelerometers " <<
                                                               sensorIdList[wbi::SENSOR_GYROSCOPE].size() << " Gyroscopes " <<
                                                               std::endl;


    //Loading configuration
    if( wbi_yarp_properties.check("robot") )
    {
        robot = wbi_yarp_properties.find("robot").asString().c_str();
    }
    else if (wbi_yarp_properties.check("robotName") )
    {
        std::cerr << "[WARN] yarpWholeBodySensors: robot option not found, using robotName" << std::endl;
        robot = wbi_yarp_properties.find("robotName").asString().c_str();
    }
    else
    {
        std::cerr << "[ERR] yarpWholeBodySensors: robot option not found" << std::endl;
        return false;
    }

    yarp::os::Bottle & joints_config = getWBIYarpJointsOptions(wbi_yarp_properties);
    controlBoardNames.clear();
    initDone = appendNewControlBoardsToVector(joints_config,sensorIdList[wbi::SENSOR_ENCODER_POS],controlBoardNames);
    initDone = initDone && appendNewControlBoardsToVector(joints_config,sensorIdList[wbi::SENSOR_PWM],controlBoardNames);
    initDone = initDone && appendNewControlBoardsToVector(joints_config,sensorIdList[wbi::SENSOR_TORQUE],controlBoardNames);
    if( !initDone )
    {
        return false;
    }

    //Resize all the data structure that depend on the number of controlboards
    int nrOfControlBoards = controlBoardNames.size();
    ienc.resize(nrOfControlBoards);
    iopl.resize(nrOfControlBoards);
    dd.resize(nrOfControlBoards);
    itrq.resize(nrOfControlBoards);

    controlBoardAxes.resize(nrOfControlBoards);
    qLastRead.resize(nrOfControlBoards);
    qStampLastRead.resize(nrOfControlBoards);
    pwmLastRead.resize(nrOfControlBoards);
    torqueSensorsLastRead.resize(nrOfControlBoards);

    getControlBoardAxisList(joints_config,sensorIdList[wbi::SENSOR_ENCODER_POS],controlBoardNames,encoderControlBoardAxisList);
    getControlBoardAxisList(joints_config,sensorIdList[wbi::SENSOR_PWM],controlBoardNames,pwmControlBoardAxisList);
    getControlBoardAxisList(joints_config,sensorIdList[wbi::SENSOR_TORQUE],controlBoardNames,torqueControlBoardAxisList);

    encoderControlBoardList = getControlBoardList(encoderControlBoardAxisList);
    pwmControlBoardList     = getControlBoardList(pwmControlBoardAxisList);
    torqueControlBoardList  = getControlBoardList(torqueControlBoardAxisList);

    for(int i=0; i < (int)encoderControlBoardList.size(); i++ )
    {
        int ctrlBoard = encoderControlBoardList[i];
        initDone = initDone && openEncoder(ctrlBoard);
    }
    if( !initDone )
    {
        std::cerr << "[ERR] yarpWholeBodySensors::init() error: failing in opening encoders." << std::endl;
        return false;
    }

    for(int i=0; i < (int)pwmControlBoardList.size(); i++ )
    {
        int ctrlBoard = pwmControlBoardList[i];
        initDone = initDone && openPwm(ctrlBoard);
    }

    if( !initDone )
    {
        std::cerr << "[ERR] yarpWholeBodySensors::init() error: failing in opening motor pwm inputs." << std::endl;
        return false;
    }

    for(int i=0; i < (int)torqueControlBoardList.size(); i++ )
    {
        int ctrlBoard = torqueControlBoardList[i];
        initDone = initDone && openTorqueSensor(ctrlBoard);
    }

    if( !initDone )
    {
        std::cerr << "[ERR] yarpWholeBodySensors::init() error: failing in opening torques sensors." << std::endl;
        return false;
    }

    //Load accelerometers information: this is tricky
    //as depending on the accelerometer type we have to add some IMU to the system
    std::vector< AccelerometerConfigurationInfo > acc_infos;
    bool ret = this->loadAccelerometerInfoFromConfig(wbi_yarp_properties,sensorIdList[wbi::SENSOR_ACCELEROMETER],acc_infos);
    if( ! ret || sensorIdList[wbi::SENSOR_ACCELEROMETER].size() != acc_infos.size()  )
    {
        std::cerr << "[ERR] yarpWholeBodySensors::init() error: failing in loading configuration of Accelerometer sensors." << std::endl;
        std::cerr<<"Sensor List : "<<sensorIdList[wbi::SENSOR_ACCELEROMETER].size()<<", acc_infos: "<<acc_infos.size();
        return false;
    }

    //Load gyroscopes information
    std::vector< GyroscopeConfigurationInfo > gyro_infos;
    ret = this->loadGyroscopeInfoFromConfig(wbi_yarp_properties,sensorIdList[wbi::SENSOR_GYROSCOPE],gyro_infos);

    //Load imu and ft sensors information
    std::vector<string> imu_ports, ft_ports;
    ret = loadFTSensorPortsFromConfig(wbi_yarp_properties,sensorIdList[wbi::SENSOR_FORCE_TORQUE],ft_ports);
    if( ! ret )
    {
        std::cerr << "[ERR] yarpWholeBodySensors::init() error: failing in loading configuration of FT sensors." << std::endl;
        return false;
    }
    ret = ret && loadIMUSensorPortsFromConfig(wbi_yarp_properties,sensorIdList[wbi::SENSOR_IMU],imu_ports);
    if( ! ret )
    {
        std::cerr << "[ERR] yarpWholeBodySensors::init() error: failing in loading configuration of IMU sensors." << std::endl;
        return false;
    }

    if( !initDone )
    {
        std::cerr << "[ERR] yarpWholeBodySensors::init() error: failing in opening force/torque sensors." << std::endl;
        return false;
    }


    //Resize all the data structure that depend on the number of fts
    int nrOfFtSensors = sensorIdList[wbi::SENSOR_FORCE_TORQUE].size();
    ftSensLastRead.resize(nrOfFtSensors);
    ftStampSensLastRead.resize(nrOfFtSensors);
    portsFTsens.resize(nrOfFtSensors);

    int nrOfImuSensors = sensorIdList[wbi::SENSOR_IMU].size();
    imuLastRead.resize(nrOfImuSensors);
    imuStampLastRead.resize(nrOfImuSensors);
    portsIMU.resize(nrOfImuSensors);


    for(int ft_numeric_id = 0; ft_numeric_id < (int)sensorIdList[wbi::SENSOR_FORCE_TORQUE].size(); ft_numeric_id++)
    {
            initDone = initDone && openFTsens(ft_numeric_id,ft_ports[ft_numeric_id]);
    }

    if( !initDone )
    {
        std::cerr << "[ERR] yarpWholeBodySensors::init() error: failing in opening force/torque sensors." << std::endl;
        return false;
    }

    for(int imu_numeric_id = 0; imu_numeric_id < (int)sensorIdList[wbi::SENSOR_IMU].size(); imu_numeric_id++)
    {
            initDone = initDone && openImu(imu_numeric_id,imu_ports[imu_numeric_id]);
    }

    if( !initDone )
    {
        std::cerr << "[ERR] yarpWholeBodySensors::init() error: failing in opening imu sensors." << std::endl;
        return false;
    }

    int nrOfAccSensors = sensorIdList[wbi::SENSOR_ACCELEROMETER].size();
    accLastRead.resize(nrOfAccSensors);
    accStampLastRead.resize(nrOfAccSensors);
    accelerometersReferenceIndeces.resize(nrOfAccSensors);

    std::cout<<"--------------------\nAccelerometers in config : "<<nrOfAccSensors<<"\n";
    for(int acc_index = 0; acc_index < (int)sensorIdList[wbi::SENSOR_ACCELEROMETER].size(); acc_index++)
    {
            initDone = initDone && openAccelerometer(acc_index,acc_infos[acc_index]);
            std::cout<<"------------------Acc index of opened acc : "<<acc_index<<"\n";
    }

    int nrOfGyroSensors = sensorIdList[wbi::SENSOR_GYROSCOPE].size();
    gyroLastRead.resize(nrOfGyroSensors);
    gyroStampLastRead.resize(nrOfGyroSensors);
    gyroscopesReferenceIndeces.resize(nrOfGyroSensors);

    for(int gyro_index = 0; gyro_index < (int)sensorIdList[wbi::SENSOR_GYROSCOPE].size(); gyro_index++)
    {
            initDone = initDone && openGyroscope(gyro_index,gyro_infos[gyro_index]);
    }
    mtbLastRead.resize(openedMTBPorts.size());
    mtbStampLastRead.resize(openedMTBPorts.size());


    if( !initDone )
    {
        std::cerr << "[ERR] yarpWholeBodySensors::init() error: failing in opening accelerometers." << std::endl;
        return false;
    }

    return initDone;
}

bool yarpWholeBodySensors::close()
{
    bool ok = true;
    for(int i=0; i < (int)encoderControlBoardList.size(); i++ )
    {
        int ctrlBoard = encoderControlBoardList[i];
        if(dd[ctrlBoard])
        {
            ok = ok && dd[ctrlBoard]->close();
            delete dd[ctrlBoard];
            dd[ctrlBoard] = 0;
        }
    }

    for(int i=0; i < (int)pwmControlBoardList.size(); i++ )
    {
        int ctrlBoard = pwmControlBoardList[i];
        if(dd[ctrlBoard])
        {
            ok = ok && dd[ctrlBoard]->close();
            delete dd[ctrlBoard];
            dd[ctrlBoard] = 0;
        }
    }

    for(int i=0; i < (int)torqueControlBoardList.size(); i++ )
    {
        int ctrlBoard = torqueControlBoardList[i];
        if(dd[ctrlBoard])
        {
            ok = ok && dd[ctrlBoard]->close();
            delete dd[ctrlBoard];
            dd[ctrlBoard] = 0;
        }
    }

    for(std::vector<BufferedPort<Vector>*>::iterator it=portsIMU.begin(); it!=portsIMU.end(); it++)
    {
        if( *it != 0 ) {
            (*it)->close();
            delete *it;
            *it=0;
        }
    }

    for(std::vector<BufferedPort<Vector>*>::iterator it=portsFTsens.begin(); it!=portsFTsens.end(); it++)
    {
        if( *it != 0 ) {
            (*it)->close();
            delete *it;
            *it=0;
        }
    }

    return ok;
}

bool yarpWholeBodySensors::addSensor(const SensorType st, const ID &sid)
{
    if( initDone )
    {
        return false;
    }

    if( st >= 0 && st < wbi::SENSOR_TYPE_SIZE )
    {
        return sensorIdList[st].addID(sid);
    }
    else
    {
        return false;
    }
}

int yarpWholeBodySensors::addSensors(const SensorType st, const IDList &sids)
{
    if( initDone )
    {
        return 0;
    }

    if( st >= 0 && st < wbi::SENSOR_TYPE_SIZE )
    {
        return sensorIdList[st].addIDList(sids);
    }
    else
    {
        return 0;
    }
}


int yarpWholeBodySensors::addAllSensors(const SensorType st)
{
    // Current implementation only takes into account Accelerometers and Gyroscopes
    // Method parses config for the appropriate group and loads the info into the SensorIDList

    std::string sensor_info_group_name;

    switch(st)
    {
        case wbi::SENSOR_ACCELEROMETER :
            sensor_info_group_name = std::string("WBI_YARP_ACCELEROMETERS");

            break;
        case wbi::SENSOR_GYROSCOPE :
            sensor_info_group_name = std::string("WBI_YARP_GYROSCOPES");

            break;
        default : // To be implemented in future (other sensors from config files)
            break;
    }

    yarp::os::Bottle info_lists = wbi_yarp_properties.findGroup(sensor_info_group_name);
    if( info_lists.isNull() || info_lists.size() == 0 ) {
        std::cout << "[ERR] yarpWbi::addAllSensors error: group "
                      << sensor_info_group_name.c_str() << " not found or empty in yarpWholeBodyInterface configuration file."  << std::endl;
        return 0;
    }
    for(int temp = 1; temp<info_lists.size();temp++)
    {
        Bottle *tempBottle = info_lists.get(temp).asList();
        std::string idName = (tempBottle->get(0)).asString();
        sensorIdList[st].addID(wbi::ID(idName));
    }
    return(0);
}


bool yarpWholeBodySensors::removeSensor(const SensorType st, const ID &sid)
{
    return false;
}

const IDList& yarpWholeBodySensors::getSensorList(const SensorType st)
{
    if( st >= 0 && st < wbi::SENSOR_TYPE_SIZE )
    {
        return sensorIdList[st];
    }
    else
    {
        return emptyList;
    }
}

int yarpWholeBodySensors::getSensorNumber(const SensorType st)
{
    if( st >= 0 && st < wbi::SENSOR_TYPE_SIZE )
    {
        return sensorIdList[st].size();
    }
    else
    {
        return 0;
    }
}

bool yarpWholeBodySensors::readSensor(const SensorType st, const int sid, double *data, double *stamps, bool blocking)
{
    switch(st)
    {
    case SENSOR_ENCODER_POS:           return readEncoder(ENCODER_POS, sid, data, stamps, blocking);
    case SENSOR_ENCODER_SPEED:         return readEncoder(ENCODER_SPEED, sid, data, stamps, blocking);
    case SENSOR_ENCODER_ACCELERATION:  return readEncoder(ENCODER_ACCELERATION, sid, data, stamps, blocking);
    case SENSOR_PWM:            return readPwm(sid, data, stamps, blocking);
    case SENSOR_IMU:            return readIMU(sid, data, stamps, blocking);
    case SENSOR_FORCE_TORQUE:   return readFTsensor(sid, data, stamps, blocking);
    case SENSOR_TORQUE:         return readTorqueSensor(sid, data, stamps, blocking);
    case SENSOR_ACCELEROMETER:  return readAccelerometer(sid, data, stamps, blocking);
    case SENSOR_GYROSCOPE:      return readGyroscope(sid, data, stamps, blocking);
    default: break;
    }
    return false;
}

bool yarpWholeBodySensors::readSensors(const SensorType st, double *data, double *stamps, bool blocking)
{
    switch(st)
    {
    case SENSOR_ENCODER_POS:           return readEncoders(ENCODER_POS, data, stamps, blocking);
    case SENSOR_ENCODER_SPEED:         return readEncoders(ENCODER_SPEED, data, stamps, blocking);
    case SENSOR_ENCODER_ACCELERATION:  return readEncoders(ENCODER_ACCELERATION, data, stamps, blocking);
    case SENSOR_PWM:            return readPwms(data, stamps, blocking);
    case SENSOR_IMU:            return readIMUs(data, stamps, blocking);
    case SENSOR_FORCE_TORQUE:   return readFTsensors(data, stamps, blocking);
    case SENSOR_TORQUE:         return readTorqueSensors(data, stamps, blocking);
    case SENSOR_ACCELEROMETER:  return readAccelerometers(data, stamps, blocking);
    case SENSOR_GYROSCOPE:      return readGyroscopes(data,stamps,blocking);
    default: break;
    }
    return false;
}

/********************************************************************************************************************************************/
/**************************************************** PRIVATE METHODS ***********************************************************************/
/********************************************************************************************************************************************/

bool yarpWholeBodySensors::openEncoder(const int bp)
{
    // check whether the encoder interface is already open
    if(ienc[bp]!=0) return true;
    // check whether the poly driver is already open (here I assume the elements of dd are initialized to 0)
    if(dd[bp]==0 && !openPolyDriver(name, robot, dd[bp], controlBoardNames[bp])) return false;
    // open the encoder interface
    if(!dd[bp]->view(ienc[bp]))
    {
        fprintf(stderr, "Problem initializing drivers of %s\n", controlBoardNames[bp].c_str());
        return false;
    }
    ///< store the number of joints in this body part
    int nj=0;
    ienc[bp]->getAxes(&nj);
    controlBoardAxes[bp] = nj;

    //allocate lastRead variables
    qLastRead[bp].resize(nj);
    qStampLastRead[bp].resize(nj);
    pwmLastRead[bp].resize(nj);
    torqueSensorsLastRead[bp].resize(nj);

    return true;
}

bool yarpWholeBodySensors::openPwm(const int bp)
{
    ///< check whether the motor PWM interface is already open
    if(iopl[bp]!=0)             return true;

    ///< if necessary open the poly driver
    if(dd[bp]==0 && !openPolyDriver(name, robot, dd[bp], controlBoardNames[bp]))
    {
        return false;
    }

    if(!dd[bp]->view(iopl[bp]))
    {
        fprintf(stderr, "Problem initializing drivers of %s\n", controlBoardNames[bp].c_str());
        return false;
    }

    //allocate lastRead variables
    qLastRead[bp].resize(controlBoardAxes[bp]);
    qStampLastRead[bp].resize(controlBoardAxes[bp]);
    pwmLastRead[bp].resize(controlBoardAxes[bp]);
    torqueSensorsLastRead[bp].resize(controlBoardAxes[bp]);

    return true;
}


bool yarpWholeBodySensors::loadAccelerometerInfoFromConfig(const Searchable& opts, const IDList& list, vector< AccelerometerConfigurationInfo >& infos)
{

    std::string accelerometers_info_group_name = "WBI_YARP_ACCELEROMETERS";
    yarp::os::Bottle info_lists = wbi_yarp_properties.findGroup(accelerometers_info_group_name);
    if( info_lists.isNull() || info_lists.size() == 0 ) {
        if( list.size() == 0 )
        {
            infos.resize(0);
            return true;
        }
        else
        {
            std::cout << "[ERR] yarpWbi::loadAccelerometerInfoFromConfig error: group "
                      << accelerometers_info_group_name << " not found in yarpWholeBodyInterface configuration file."  << std::endl;
            return false;
        }
    }
    infos.resize(list.size());

    for(int acc_index =0; acc_index < (int)list.size(); acc_index++ )
    {
        wbi::ID acc_ID;
        list.indexToID(acc_index,acc_ID);
        yarp::os::Bottle * sensorInfo = info_lists.find(acc_ID.toString()).asList();
        if( sensorInfo == NULL
            || sensorInfo->size() != 3
            || !(sensorInfo->get(0).isString())
            || !(sensorInfo->get(1).isInt())
            || !(sensorInfo->get(2).isString()) )
        {
            std::cout << "yarpWbi::loadAccelerometerInfoFromConfig error: " << info_lists.toString() << " has a malformed element" << std::endl;
            return false;
        }
        infos[acc_index].port_name = std::string("/icub/" ) +
                                                sensorInfo->get(0).asString() +
                                                std::string("/inertialMTB");
        infos[acc_index].data_index = sensorInfo->get(1).asInt();
        std::string accelerometer_type = sensorInfo->get(2).asString();
        if( accelerometer_type == "imu" )
        {
            infos[acc_index].type = IMU_ACCL;
            infos[acc_index].type_option = accelerometer_type;
        } // incorporate the MTB Accelerometers
        else if ( accelerometer_type == "mtb")
        {
            infos[acc_index].type = MTB_ACCL;
            infos[acc_index].type_option = accelerometer_type;
        }
        else
        {
            std::cout << "yarpWbi::loadAccelerometerInfoFromConfig error: "
                       << "accelerometer type " << accelerometer_type << "not recognized" << std::endl;
            return false;
        }
    }
    return true;
}

bool yarpWholeBodySensors::loadGyroscopeInfoFromConfig(const Searchable& opts, const IDList& list, vector< GyroscopeConfigurationInfo >& infos)
{

    std::string gyroscopes_info_group_name = "WBI_YARP_GYROSCOPES";
    yarp::os::Bottle info_lists = wbi_yarp_properties.findGroup(gyroscopes_info_group_name);
    if( info_lists.isNull() || info_lists.size() == 0 ) {
        if( list.size() == 0 )
        {
            infos.resize(0);
            return true;
        }
        else
        {
            std::cout << "[ERR] yarpWbi::loadGyroscopeInfoFromConfig error: group "
                      << gyroscopes_info_group_name << " not found in yarpWholeBodyInterface configuration file."  << std::endl;
            return false;
        }
    }
    infos.resize(list.size());


    std::cout<<"-----------------------\nRecovering IDList IDs and lookinf for rest of info in list of size "<<list.size()<<"\n";
    for(int gyro_index =0; gyro_index < (int)list.size(); gyro_index++ )
    {
        wbi::ID gyro_ID;
        list.indexToID(gyro_index,gyro_ID);
        std::cout <<"---------------- GyroID : "<< gyro_ID.toString() << std::endl;
        yarp::os::Bottle * sensorInfo = info_lists.find(gyro_ID.toString()).asList();
        std::cout <<"---------------- Gyro contents"<< sensorInfo->toString() << std::endl;
        if( sensorInfo == NULL
            || sensorInfo->size() != 3
            || !(sensorInfo->get(0).isString())
            || !(sensorInfo->get(1).isInt())
            || !(sensorInfo->get(2).isString()) )
        {
            std::cout << "yarpWbi::loadGyroscopeInfoFromConfig error: " << info_lists.toString() << " has a malformed element" << std::endl;
            return false;
        }
        infos[gyro_index].port_name = std::string("/icub/" ) +
                                                sensorInfo->get(0).asString() +
                                                std::string("/inertialMTB");
        infos[gyro_index].data_index = sensorInfo->get(1).asInt();
        std::string gyroscope_type = sensorInfo->get(2).asString();
        std::cout<<"--------------------\naccelerometer port : "<<infos[gyro_index].port_name.c_str()<<
                                                 ", data idx :"<<infos[gyro_index].data_index<<"\n";
        if( gyroscope_type == "imu" )
        {
            infos[gyro_index].type = IMU_GYRO;
            infos[gyro_index].type_option = gyroscope_type;
        } // incorporate the MTB Accelerometers
        else if ( gyroscope_type == "mtb")
        {
            infos[gyro_index].type = MTB_GYRO;
            infos[gyro_index].type_option = gyroscope_type;
        }
        else
        {
            std::cout << "yarpWbi::loadGyroscopeInfoFromConfig error: "
                       << "gyroscope type " << gyroscope_type << "not recognized" << std::endl;
            return false;
        }
    }
    return true;
}
bool yarpWholeBodySensors::openAccelerometer(const int acc_index, const AccelerometerConfigurationInfo & info)
{
    bool ret = true;
                int reference_sensor_index;
    switch( info.type )
    {
        case IMU_ACCL:

            ret = sensorIdList[SENSOR_IMU].idToIndex(info.type_option,reference_sensor_index);
            if( !ret )
            {
                std::cerr << "yarpWholeBodySensors::openAccelerometer :  impossible to find IMU "
                          << info.type_option << std::endl;
                return false;
            }
            accelerometersReferenceIndeces[acc_index].type = info.type;
            accelerometersReferenceIndeces[acc_index].runtime_index = reference_sensor_index;
            break;
        case MTB_ACCL:
//             int reference_mtb_sensor_index;
//             ret = sensorIdList[SENSOR_IMU].idToIndex(info.data_index,reference_imu_sensor_index);
                //allocate lastRead variables
            ret = openMTBSensor(info.port_name);
            accLastRead[acc_index].resize(sensorTypeDescriptions[SENSOR_ACCELEROMETER].dataSize,0.0);
            accStampLastRead[acc_index] = INITIAL_TIMESTAMP;

            accelerometersReferenceIndeces[acc_index].type = info.type;
            accelerometersReferenceIndeces[acc_index].data_index = info.data_index;
            accelerometersReferenceIndeces[acc_index].sensor_urdf_name = info.sensor_urdf_name;
            accelerometersReferenceIndeces[acc_index].runtime_index = portsMTBSensors.size()-1;
           // ret = sensorIdList[SENSOR_ACCELEROMETER].idToIndex(info.type_option,reference_sensor_index);
            //accelerometersReferenceIndeces[acc_index].runtime_index = reference_sensor_index;


            break;
        default:
            std::cerr << "yarpWholeBodySensors::openAccelerometer : unknown accelerometer type (only known type is IMU_STYLE: " << IMU_ACCL << " )"
                          << info.type << std::endl;
            ret = false;
            break;
    }
    return ret;
}
bool yarpWholeBodySensors::openGyroscope(const int gyro_index, const GyroscopeConfigurationInfo & info)
{
    bool ret = true;
                int reference_sensor_index;
    switch( info.type )
    {
        case IMU_GYRO:

            ret = sensorIdList[SENSOR_IMU].idToIndex(info.type_option,reference_sensor_index);
            if( !ret )
            {
                std::cerr << "yarpWholeBodySensors::openGyroscope :  impossible to find IMU "
                          << info.type_option << std::endl;
                return false;
            }
            gyroscopesReferenceIndeces[gyro_index].type = info.type;
            gyroscopesReferenceIndeces[gyro_index].runtime_index = reference_sensor_index;
            break;
        case MTB_GYRO:
            ret = openMTBSensor(info.port_name);
            gyroLastRead[gyro_index].resize(sensorTypeDescriptions[SENSOR_GYROSCOPE].dataSize,0.0);
            gyroStampLastRead[gyro_index] = INITIAL_TIMESTAMP;

            gyroscopesReferenceIndeces[gyro_index].type = info.type;
            gyroscopesReferenceIndeces[gyro_index].data_index = info.data_index;
            gyroscopesReferenceIndeces[gyro_index].runtime_index = portsMTBSensors.size()-1;
            gyroscopesReferenceIndeces[gyro_index].sensor_urdf_name = info.sensor_urdf_name;

            break;
        default:
            std::cerr << "yarpWholeBodySensors::openGyroscope : unknown gyroscope type (only known types are IMU_GYRO: " << IMU_GYRO << ", and MTB_GYRO: " << MTB_GYRO <<" ), "
                          << info.type << std::endl;
            ret = false;
            break;
    }
    return ret;
}
bool yarpWholeBodySensors::openImu(const int numeric_id, const std::string & port_name)
{
    if( numeric_id < 0 || numeric_id >= (int)sensorIdList[SENSOR_IMU].size() )
    {
        return false;
    }

    string remotePort = "/" + robot + port_name;
    stringstream localPort;
    wbi::ID wbi_id;
    sensorIdList[SENSOR_IMU].indexToID(numeric_id,wbi_id);
    localPort << "/" << name << "/imu/" <<  wbi_id.toString() << ":i";
    portsIMU[numeric_id] = new BufferedPort<Vector>();
    if(!portsIMU[numeric_id]->open(localPort.str().c_str())) { // open local input port
        std::cerr << "yarpWholeBodySensors::openImu(): Open of localPort " << localPort.str() << " failed " << std::endl;
        return false;
    }
    if(!Network::exists(remotePort.c_str())) {       // check remote output port exists
        std::cerr << "yarpWholeBodySensors::openImu():  " << remotePort << " does not exist " << std::endl;
        return false;
    }
    if(!Network::connect(remotePort.c_str(), localPort.str().c_str(), "udp", true)) {  // connect remote to local port
        std::cerr << "yarpWholeBodySensors::openImu():  could not connect " << remotePort << " to " << localPort.str() << std::endl;
        return false;
    }

    //allocate lastRead variables
    imuLastRead[numeric_id].resize(sensorTypeDescriptions[SENSOR_IMU].dataSize,0.0);
    imuStampLastRead[numeric_id] = INITIAL_TIMESTAMP;

    return true;
}

bool yarpWholeBodySensors::openMTBSensor(const std::string & remotePort)
{
    stringstream localPort;

    // TODO : check portsMTBSensors if this sensors port is already opened. This is because multiple MTB sensors use few remote ports
    for (std::vector<std::string>::iterator it = openedMTBPorts.begin(); it!=openedMTBPorts.end();it++)
    {
        if(remotePort.compare(*it) == 0)
        {
            // remotePort has already been added
            std::cout<<"--------------Remote port : "<<remotePort.c_str()<<" has already been opened \n";
            return true;
        }
    }

    std::cout<<"-------------Trying to open a port to :"<<remotePort.c_str()<<"\n";
    int numeric_id = portsMTBSensors.size();
    localPort << "/" << name << "/mtb_sensor/" <<  numeric_id << ":i";
    portsMTBSensors.push_back(new BufferedPort<Vector>());


    if(!portsMTBSensors[numeric_id]->open(localPort.str().c_str())) { // open local input port
        std::cerr << "yarpWholeBodySensors::openMTBSensors(): Open of localPort " << localPort.str() << " failed " << std::endl;
        return false;
    }
    if(!Network::exists(remotePort.c_str())) {       // check remote output port exists
        std::cerr << "yarpWholeBodySensors::openMTBSensors():  " << remotePort << " does not exist " << std::endl;
        return false;
    }
    if(!Network::connect(remotePort.c_str(), localPort.str().c_str(), "udp", true)) {  // connect remote to local port
        std::cerr << "yarpWholeBodySensors::openImu():  could not connect " << remotePort << " to " << localPort.str() << std::endl;
        return false;
    }
    openedMTBPorts.push_back(remotePort);

    std::cout<<"-----------------Remote port "<<remotePort<<" exists and connected to "<<localPort.str().c_str()<<"\n";
    return true;
}

bool yarpWholeBodySensors::openFTsens(const int ft_sens_numeric_id, const std::string & port_name)
{
    string remotePort = "/" + robot + port_name;
    stringstream localPort;
    wbi::ID wbi_id;
    sensorIdList[SENSOR_FORCE_TORQUE].indexToID(ft_sens_numeric_id,wbi_id);
    localPort << "/" << name << "/ftSens/" << wbi_id.toString() << ":i";
    portsFTsens[ft_sens_numeric_id] = new BufferedPort<Vector>();
    if(!portsFTsens[ft_sens_numeric_id]->open(localPort.str().c_str())) {
        // open local input port
        std::cerr << "yarpWholeBodySensors::openFTsens(): Open of localPort " << localPort.str() << " failed " << std::endl;
        return false;
    }
    if(!Network::exists(remotePort.c_str())) {            // check remote output port exists
        std::cerr << "yarpWholeBodySensors::openFTsens():  " << remotePort << " does not exist " << std::endl;
        return false;
    }
    if(!Network::connect(remotePort.c_str(), localPort.str().c_str(), "udp")) {  // connect remote to local port
        std::cerr << "yarpWholeBodySensors::openFTsens():  could not connect " << remotePort << " to " << localPort.str() << std::endl;
        return false;
    }

    //allocate lastRead variables
    ftSensLastRead[ft_sens_numeric_id].resize(sensorTypeDescriptions[SENSOR_FORCE_TORQUE].dataSize,0.0);
    ftStampSensLastRead[ft_sens_numeric_id] = INITIAL_TIMESTAMP;

    return true;
}

bool yarpWholeBodySensors::openTorqueSensor(const int bp)
{
    ///< check whether the joint control interface is already open
    if(itrq[bp])
        return true;

    ///< if necessary open the poly driver
    if(dd[bp]==0 && !openPolyDriver(name, robot, dd[bp], controlBoardNames[bp]))
        return false;

    if(!dd[bp]->view(itrq[bp]))
    {
        fprintf(stderr, "Problem initializing drivers of %s\n", controlBoardNames[bp].c_str());
        return false;
    }

    //allocate lastRead variables
    qLastRead[bp].resize(controlBoardAxes[bp]);
    qStampLastRead[bp].resize(controlBoardAxes[bp]);
    pwmLastRead[bp].resize(controlBoardAxes[bp]);
    torqueSensorsLastRead[bp].resize(controlBoardAxes[bp]);

    return true;
}

vector< std::string > yarpWholeBodySensors::getSensorNames(const SensorType st)
{
    vector<std::string> ret;
    IDList selectedSensors= sensorIdList[st];
    for(int i=0; i < (int)sensorIdList[st].size(); i++)
    {
        switch(st)
        {
            case SENSOR_ACCELEROMETER :
                    ret.push_back(accelerometersReferenceIndeces[i].sensor_urdf_name);
                break;
            case SENSOR_GYROSCOPE :
                    ret.push_back(gyroscopesReferenceIndeces[i].sensor_urdf_name);
                break;

            default :
                std::cerr<<" To be implemented for all kinds of sensors\n";
                break;
        }
    }
    return(ret);
}


/**************************** READ ************************/

bool yarpWholeBodySensors::getEncodersPosSpeedAccTimed(const EncoderType st, yarp::dev::IEncodersTimed* ienc, double *encs, double *time)
{
    bool result = ienc->getEncodersTimed(encs, time);
//    bool result = true;
    switch (st) {
        case ENCODER_POS:          return result;// ienc->getEncodersTimed(encs, time);
        case ENCODER_SPEED:        return result && ienc->getEncoderSpeeds(encs);// getEncodersTimed(encs, time);
        case ENCODER_ACCELERATION: return result && ienc->getEncoderAccelerations(encs);//(encs, time);
        default:                   return false;
    }
}

bool yarpWholeBodySensors::readEncoders(const EncoderType st, double *data, double *stamps, bool wait)
{
    //std::cout << "|||||||||| Read encoders " << std::endl;
    double dataTemp[MAX_NJ], tTemp[MAX_NJ];
    bool res = true, update=false;

     //Read data from all controlboards
    for(std::vector<int>::const_iterator ctrlBoard = encoderControlBoardList.begin();
        ctrlBoard != encoderControlBoardList.end(); ctrlBoard++ )
    {
        // read data
        // std::cout << "|||||||||| getEncodersTimed " << std::endl;
        dataTemp[0] = -10.0;
        dataTemp[1] = -10.0;
        double waiting_time = 0;
        while( !(update=getEncodersPosSpeedAccTimed(st, ienc[*ctrlBoard], dataTemp, tTemp)) && wait)
        {
            //std::cout << "waitign " << dataTemp[0] << " " << dataTemp[1] << std::endl;
            Time::delay(WAIT_TIME);
            waiting_time += WAIT_TIME;

            if( waiting_time > BLOCKING_SENSOR_TIMEOUT )
            {
                yError("yarpWholeBodySensors::readEncoders failed for timeout");
                return false;
            }
        }

        // if reading has succeeded, update last read data
        if(update)
        {
            for(int axis=0; axis < (int)qLastRead[*ctrlBoard].size(); axis++ )
            {
                //std::cout << "read dataTemp : " << dataTemp[axis] << std::endl;
                qLastRead[*ctrlBoard][axis] = yarpWbi::Deg2Rad*dataTemp[axis];
                qStampLastRead[*ctrlBoard][axis] = tTemp[axis];
            }
        }

        res = res && update;
    }

     //Copy readed data in the output vector
    for(int encNumericId = 0; encNumericId < (int)sensorIdList[SENSOR_ENCODER_POS].size(); encNumericId++)
    {
        int encControlBoard = encoderControlBoardAxisList[encNumericId].first;
        int encAxis = encoderControlBoardAxisList[encNumericId].second;
        data[encNumericId] = qLastRead[encControlBoard][encAxis];
        if(stamps!=0)
                stamps[encNumericId] = qStampLastRead[encControlBoard][encAxis];
    }


    return res || wait;
}

bool yarpWholeBodySensors::readPwms(double *pwm, double *stamps, bool wait)
{
    //Do not support stamps on pwm
    if(stamps != 0)
    {

        return false;
    }

    double pwmTemp[MAX_NJ];
    bool res = true, update=false;

    //Read data from all controlboards
    for(std::vector<int>::iterator ctrlBoard=pwmControlBoardList.begin();
        ctrlBoard != pwmControlBoardList.end(); ctrlBoard++ )
    {
        // read data
        double waiting_time = 0;
        while( !(update=iopl[*ctrlBoard]->getOutputs(pwmTemp)) && wait)
        {
            Time::delay(WAIT_TIME);

            waiting_time += WAIT_TIME;

            if( waiting_time > BLOCKING_SENSOR_TIMEOUT )
            {
                yError("yarpWholeBodySensors::readPwms failed for timeout");
                return false;
            }
        }

        // if reading has succeeded, update last read data
        if(update)
        {
            for(int axis=0; axis < (int) pwmLastRead[*ctrlBoard].size(); axis++ )
            {
                pwmLastRead[*ctrlBoard][axis] = pwmTemp[axis];
            }
        }

        res = res && update;
    }

    //Copy readed data in the output vector
    for(int pwmNumericId = 0; pwmNumericId < (int)sensorIdList[SENSOR_PWM].size(); pwmNumericId++)
    {
        int pwmControlBoard = pwmControlBoardAxisList[pwmNumericId].first;
        int pwmAxes = pwmControlBoardAxisList[pwmNumericId].second;
        pwm[pwmNumericId] = pwmLastRead[pwmControlBoard][pwmAxes];
    }

    return res || wait;
}

bool yarpWholeBodySensors::readAccelerometers(double *accs, double *stamps, bool wait)
{
    bool ret = true;
//     std::cout<<"Reading accelerometer from list of size : "<<(int)sensorIdList[SENSOR_ACCELEROMETER].size()<<"...\n\n";
    for(int i=0; i < (int)sensorIdList[SENSOR_ACCELEROMETER].size(); i++)
    {
        if(stamps!=0)
            ret = ret && this->readAccelerometer(i,accs+(sensorTypeDescriptions[SENSOR_ACCELEROMETER].dataSize)*i,stamps+i,wait);
        else
            ret = ret && this->readAccelerometer(i,accs+(sensorTypeDescriptions[SENSOR_ACCELEROMETER].dataSize)*i,stamps,wait);
    }
    return ret;
}
bool yarpWholeBodySensors::readGyroscopes(double *gyros, double *stamps, bool wait)
{
    bool ret = true;
    for(int i=0; i < (int)sensorIdList[SENSOR_GYROSCOPE].size(); i++)
    {
        if(stamps!=0)
            ret = ret && this->readGyroscope(i,gyros+(sensorTypeDescriptions[SENSOR_GYROSCOPE].dataSize)*i,stamps+i,wait);
        else
            ret = ret && this->readGyroscope(i,gyros+(sensorTypeDescriptions[SENSOR_GYROSCOPE].dataSize)*i,stamps,wait);
    }
    return ret;
}
bool yarpWholeBodySensors::readIMUs(double *inertial, double *stamps, bool wait)
{
    assert(false);
    return false;
    Vector *v;
    for(int i=0; i < (int)sensorIdList[SENSOR_IMU].size(); i++)
    {
        v = portsIMU[i]->read(wait);
        if(v!=0)
        {
            yarp::os::Stamp info;
            imuLastRead[i] = *v;
            portsIMU[i]->getEnvelope(info);
            imuStampLastRead[i] = info.getTime();
        }
        convertIMU(&inertial[sensorTypeDescriptions[SENSOR_IMU].dataSize*i],imuLastRead[i].data());
        if( stamps != 0 ) {
            stamps[i] = imuStampLastRead[i];
        }
    }
    return true;
}


bool yarpWholeBodySensors::readFTsensors(double *ftSens, double *stamps, bool wait)
{
    Vector *v;
    for(int i=0; i < (int)sensorIdList[SENSOR_FORCE_TORQUE].size(); i++)
    {
        v = portsFTsens[i]->read(wait);
        if(v!=0)
        {
            ftSensLastRead[i] = *v;
            yarp::os::Stamp info;
            portsFTsens[i]->getEnvelope(info);
            ftStampSensLastRead[i] = info.getTime();
        }
        memcpy(&ftSens[i*6], ftSensLastRead[i].data(), 6*sizeof(double));
        if( stamps != 0 ) {
                stamps[i] = ftStampSensLastRead[i];
        }
        i++;
    }
    return true;
}

bool yarpWholeBodySensors::readTorqueSensors(double *jointSens, double *stamps, bool wait)
{
    bool res = true, update=false;
   //Do not support stamps on torque sensors

    //Read data from all controlboards
    for(std::vector<int>::iterator ctrlBoard=torqueControlBoardList.begin();
        ctrlBoard != torqueControlBoardList.end(); ctrlBoard++ )
    {
        // read data
        double waiting_time = 0;
        while( !(update=itrq[*ctrlBoard]->getTorques(torqueSensorsLastRead[*ctrlBoard].data())) && wait)
        {
            Time::delay(WAIT_TIME);

            waiting_time += WAIT_TIME;

            if( waiting_time > BLOCKING_SENSOR_TIMEOUT )
            {
                yError("yarpWholeBodySensors::readTorqueSensors failed for timeout");
                return false;
            }
        }

        res = res && update;
    }

    //Copy readed data in the output vector
    for(int torqueNumericId = 0; torqueNumericId < (int)sensorIdList[SENSOR_TORQUE].size(); torqueNumericId++)
    {
        int torqueControlBoard = torqueControlBoardAxisList[torqueNumericId].first;
        int torqueAxis = torqueControlBoardAxisList[torqueNumericId].second;
        jointSens[torqueNumericId] = torqueSensorsLastRead[torqueControlBoard][torqueAxis];
    }

    if(stamps != 0)
    {
        double now = yarp::os::Time::now();
        for(int torqueNumericId = 0; torqueNumericId < (int)sensorIdList[SENSOR_TORQUE].size(); torqueNumericId++)
        {
            stamps[torqueNumericId] = now;
        }
    }

    return res || wait;
}

bool yarpWholeBodySensors::readEncoder(const EncoderType st, const int encoder_numeric_id, double *data, double *stamps, bool wait)
{
    bool update=false;
    int encoderCtrlBoard = encoderControlBoardAxisList[encoder_numeric_id].first;
    int encoderCtrlBoardAxis = encoderControlBoardAxisList[encoder_numeric_id].second;

    double dataTemp[MAX_NJ], tTemp[MAX_NJ];

    // read encoders
    double waiting_time = 0;
    while( !(update=getEncodersPosSpeedAccTimed(st, ienc[encoderCtrlBoard], dataTemp, tTemp)) && wait)
    {
        Time::delay(WAIT_TIME);

        waiting_time += WAIT_TIME;

        if( waiting_time > BLOCKING_SENSOR_TIMEOUT )
        {
            yError("yarpWholeBodySensors::readEncoder failed for timeout");
            return false;
        }
    }

    if( update )
    {
          for(int axis=0; axis < (int)qLastRead[encoderCtrlBoard].size(); axis++ )
          {
                //std::cout << "read dataTemp : " << dataTemp[axis] << std::endl;
                qLastRead[encoderCtrlBoard][axis] = yarpWbi::Deg2Rad*dataTemp[axis];
                qStampLastRead[encoderCtrlBoard][axis] = tTemp[axis];
          }
    }

    // copy most recent data into output variables
    data[0] = qLastRead[encoderCtrlBoard][encoderCtrlBoardAxis];
    if(stamps!=0)
        stamps[0] = qStampLastRead[encoderCtrlBoard][encoderCtrlBoardAxis];

    return update || wait;  // if read failed => return false
}

bool yarpWholeBodySensors::readPwm(const int pwm_numeric_id, double *pwm, double *stamps, bool wait)
{
    if(stamps != 0)
    {
        yWarning("yarpWholeBodySensors::readPwm does not support timestamp reading at the moment");
    }

    bool update=false;
    int pwmCtrlBoard = pwmControlBoardAxisList[pwm_numeric_id].first;
    int pwmCtrlBoardAxis = pwmControlBoardAxisList[pwm_numeric_id].second;

    // read pwm sensors
    double waiting_time = 0.0;
    while( !(update=iopl[pwmCtrlBoard]->getOutputs(pwmLastRead[pwmCtrlBoard].data())) && wait)
    {
        Time::delay(WAIT_TIME);

        waiting_time += WAIT_TIME;

        if( waiting_time > BLOCKING_SENSOR_TIMEOUT )
        {
            yError("yarpWholeBodySensors::readPwm failed for timeout");
            return false;
        }
    }

    // copy most recent data into output variables
    pwm[0] = pwmLastRead[pwmCtrlBoard][pwmCtrlBoardAxis];

    return update || wait;  // if read failed => return false
}

bool yarpWholeBodySensors::convertIMU(double * wbi_imu_readings, const double * yarp_imu_readings)
{
    //wbi orientation is expressed in axis-angle, yarp orientation in euler angles (roll pitch yaw)
    //wbi  : orientation(4) - linear acceleration (3) - angular velocity    (3) - magnetometer (3)
    //yarp : orientation(3) - linear acceleration (3) - angular velocity    (3) - magnetometer (3)
    /// \todo TODO check right semantics of yarp IMU

    //Note: this function stop working if the definition of the IMU message in the wbi changes
    assert(sensorTypeDescriptions[SENSOR_IMU].dataSize == 13);

    Rotation imu_orientation = Rotation::eulerZYX(yarp_imu_readings[0],yarp_imu_readings[1],yarp_imu_readings[2]);
    imu_orientation.getAxisAngle(*wbi_imu_readings,*(wbi_imu_readings+1),*(wbi_imu_readings+2),*(wbi_imu_readings+3));
    memcpy(wbi_imu_readings+4,yarp_imu_readings+3,9*sizeof(double));
    return true;
}

bool yarpWholeBodySensors::readAccelerometer(const int accelerometer_index, double *acc, double *stamps, bool wait)
{
//     std::cout<<"----------------Trying to read accelerometer\n";
    bool ret = true;
    if( accelerometer_index >= (int)sensorIdList[wbi::SENSOR_ACCELEROMETER].size() || accelerometer_index < 0 )
    {
        return false;
    }
//     std::cout<<"----------------reference index size :"<<accelerometersReferenceIndeces.size()<<"\n";
//     std::cout<<"----------------mtb last read size :"<<mtbLastRead.size()<<"\n";
    int accelerometer_runtime_index;
    switch(accelerometersReferenceIndeces[accelerometer_index].type)
    {
        case IMU_ACCL :
            accelerometer_runtime_index = accelerometersReferenceIndeces[accelerometer_index].runtime_index;

            // Process IMU accelerometer data
            if( stamps != 0 )
            {
                *stamps = imuStampLastRead[accelerometer_runtime_index];
            }

            acc[0] = imuLastRead[accelerometer_runtime_index][4];
            acc[1] = imuLastRead[accelerometer_runtime_index][5];
            acc[2] = imuLastRead[accelerometer_runtime_index][6];
            ret = true;
            break;
        case MTB_ACCL :
            // Process MTB accelerometer data
            accelerometer_runtime_index = accelerometersReferenceIndeces[accelerometer_index].runtime_index;
            // Process IMU accelerometer data

            yarp::sig::Vector *vt;// = portsMTBSensors[accelerometer_runtime_index]->read(wait);
            vt = portsMTBSensors[accelerometer_runtime_index]->read(wait);

            if(vt!=NULL) {
                if(mtbLastRead[accelerometer_runtime_index].size()==0)
                {
                    mtbLastRead[accelerometer_runtime_index].resize(vt->size(),0.0);
                }
                mtbLastRead[accelerometer_runtime_index] = *vt;
                yarp::os::Stamp info;
                portsMTBSensors[accelerometer_runtime_index]->getEnvelope(info);
                mtbStampLastRead[accelerometer_runtime_index] = info.getTime();
            }
            if( stamps != 0 )
            {
                *stamps = mtbStampLastRead[accelerometer_runtime_index];
            }
            int mtb_first_data_index;
            mtb_first_data_index = accelerometersReferenceIndeces[accelerometer_index].data_index-1;
            double *tempData;
            tempData = mtbLastRead[accelerometer_runtime_index].data();
            acc[0] = tempData[mtb_first_data_index] * (double)5.9855e-4;
            acc[1] = tempData[mtb_first_data_index+1] * (double)5.9855e-4;
            acc[2] = tempData[mtb_first_data_index+2] * (double)5.9855e-4;
            ret = true;
            break;
        default :
            ret = false;
            break;
    }
    return ret;
}

bool yarpWholeBodySensors::readGyroscope(const int gyroscope_index, double *gyro, double *stamps, bool wait)
{
//     std::cout<<"----------------Trying to read gyroscope\n";
    bool ret = true;
    if( gyroscope_index >= (int)sensorIdList[wbi::SENSOR_GYROSCOPE].size() || gyroscope_index < 0 )
    {
        return false;
    }
//     std::cout<<"----------------reference index size :"<<accelerometersReferenceIndeces.size()<<"\n";
//     std::cout<<"----------------mtb last read size :"<<mtbLastRead.size()<<"\n";
    int gyroscope_runtime_index;
    switch(accelerometersReferenceIndeces[gyroscope_index].type)
    {
        case IMU_GYRO :
            gyroscope_runtime_index = accelerometersReferenceIndeces[gyroscope_index].runtime_index;

            // Process IMU accelerometer data
            if( stamps != 0 )
            {
                *stamps = imuStampLastRead[gyroscope_runtime_index];
            }

            gyro[0] = imuLastRead[gyroscope_runtime_index][7];
            gyro[1] = imuLastRead[gyroscope_runtime_index][8];
            gyro[2] = imuLastRead[gyroscope_runtime_index][9];
            ret = true;
            break;
//
        case MTB_GYRO :
            // Process MTB gyroscope data
            gyroscope_runtime_index = accelerometersReferenceIndeces[gyroscope_index].runtime_index;


            yarp::sig::Vector *vt;// = portsMTBSensors[accelerometer_runtime_index]->read(wait);
            vt = portsMTBSensors[gyroscope_runtime_index]->read(wait);
           if(vt!=NULL) {
                if(mtbLastRead[gyroscope_runtime_index].size()==0)
                {
                    mtbLastRead[gyroscope_runtime_index].resize(vt->size(),0.0);
                }
                mtbLastRead[gyroscope_runtime_index] = *vt;
                yarp::os::Stamp info;
                portsMTBSensors[gyroscope_runtime_index]->getEnvelope(info);
                mtbStampLastRead[gyroscope_runtime_index] = info.getTime();
            }
            if( stamps != 0 )
            {
                *stamps = mtbStampLastRead[gyroscope_runtime_index];
            }
            int mtb_first_data_index;
            mtb_first_data_index = gyroscopesReferenceIndeces[gyroscope_index].data_index-1;
            double *tempData;
            tempData = mtbLastRead[gyroscope_runtime_index].data();
            gyro[0] = tempData[mtb_first_data_index] * (double)7.6274e-3;
            gyro[1] = tempData[mtb_first_data_index+1] * (double)7.6274e-3;
            gyro[2] = tempData[mtb_first_data_index+2] * (double)7.6274e-3;
            ret = true;
            break;
        default :
            ret = false;
            break;
    }
    return ret;
}

bool yarpWholeBodySensors::readIMU(const int imu_sensor_numeric_id, double *inertial, double *stamps, bool wait)
{
    #ifndef NDEBUG
    if( imu_sensor_numeric_id < 0 || imu_sensor_numeric_id >= (int)sensorIdList[SENSOR_IMU].size() ) {
        std::cerr << "yarpWholeBodySensors::readIMU(..) error: no port found for imu with numeric id " << imu_sensor_numeric_id<< std::endl;
        return false;
    }
    #endif

    Vector *v = portsIMU[imu_sensor_numeric_id]->read(wait);
    if(v!=0) {
        yarp::os::Stamp info;
        imuLastRead[imu_sensor_numeric_id] = *v;
        portsIMU[imu_sensor_numeric_id]->getEnvelope(info);
        imuStampLastRead[imu_sensor_numeric_id] = info.getTime();
    }
    if( stamps != 0 ) {
        *stamps = imuStampLastRead[imu_sensor_numeric_id];
    }
    convertIMU(inertial,imuLastRead[imu_sensor_numeric_id].data());

    return true;
}

bool yarpWholeBodySensors::readFTsensor(const int ft_sensor_numeric_id, double *ftSens, double *stamps, bool wait)
{
    if( ft_sensor_numeric_id < 0
        || ft_sensor_numeric_id >= (int)sensorIdList[SENSOR_FORCE_TORQUE].size() )
    {
        std::cerr << "yarpWholeBodySensors::readFTsensor(..) error: no port found for ft sensor "
                  << ft_sensor_numeric_id  << std::endl;
        return false;
    }

    Vector *v = portsFTsens[ft_sensor_numeric_id]->read(wait);
    if(v!=NULL) {
        ftSensLastRead[ft_sensor_numeric_id] = *v;
        yarp::os::Stamp info;
        portsFTsens[ft_sensor_numeric_id]->getEnvelope(info);
        ftStampSensLastRead[ft_sensor_numeric_id] = info.getTime();
    }
    memcpy(&ftSens[0], ftSensLastRead[ft_sensor_numeric_id].data(), 6*sizeof(double));
    if( stamps != 0 ) {
        *stamps = ftStampSensLastRead[ft_sensor_numeric_id];
    }

    return true;
}

bool yarpWholeBodySensors::readTorqueSensor(const int numeric_torque_id, double *jointTorque, double *stamps, bool wait)
{
    double torqueTemp;
    bool update=false;

    int torqueCtrlBoard = torqueControlBoardAxisList[numeric_torque_id].first;
    int torqueCtrlBoardAxis = torqueControlBoardAxisList[numeric_torque_id].second;

    assert(itrq[torqueCtrlBoard]!=0);

    // read joint torque
    double waiting_time = 0.0;
    while(!(update = itrq[torqueCtrlBoard]->getTorque(torqueCtrlBoardAxis, &torqueTemp)) && wait)
    {
        Time::delay(WAIT_TIME);

        waiting_time += WAIT_TIME;

        if( waiting_time > BLOCKING_SENSOR_TIMEOUT )
        {
            yError("yarpWholeBodySensors::readTorqueSensor failed for timeout");
            return false;
        }
    }

    // if read succeeded => update data
    if(update)
        torqueSensorsLastRead[torqueCtrlBoard][torqueCtrlBoardAxis] = torqueTemp;

    // copy most recent data into output variables
    jointTorque[0] = torqueSensorsLastRead[torqueCtrlBoard][torqueCtrlBoardAxis];

    return update || wait;  // if read failed => return false
}
