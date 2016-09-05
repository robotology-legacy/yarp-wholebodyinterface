/*
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * email: silvio.traversaro@iit.it
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

#include "yarpWholeBodyModel.h"
#include "yarpWbiUtil.h"

#include <string>
#include <cmath>

#include <iCub/skinDynLib/common.h>
#include <iCub/ctrl/math.h>

#include <yarp/math/Math.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include <yarp/os/ResourceFinder.h>

#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>

using namespace std;
using namespace wbi;
using namespace yarpWbi;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::skinDynLib;

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          YARP WHOLE BODY MODEL
// *********************************************************************************************************************
// *********************************************************************************************************************
yarpWholeBodyModel::yarpWholeBodyModel(const char* _name,
                                       const yarp::os::Property & _wbi_yarp_conf)
    : initDone(false),
      dof(0),
      wbi_yarp_properties(_wbi_yarp_conf),
      getLimitsFromControlBoard(false)
{
}

yarpWholeBodyModel::~yarpWholeBodyModel()
{
    if( p_model )
    {
        delete p_model;
        p_model = 0;
    }
}

bool yarpWholeBodyModel::init()
{
    if( this->initDone ) return true;

    //Loading configuration
    if( wbi_yarp_properties.check("robot") )
    {
        robot = wbi_yarp_properties.find("robot").asString().c_str();
    }
    else if (wbi_yarp_properties.check("robotName") )
    {
        yWarning() << "yarpWholeBodyModel: robot option not found, using robotName";
        robot = wbi_yarp_properties.find("robotName").asString().c_str();
    }
    else
    {
        yError() << "yarpWholeBodyModel: robot option not found";
        return false;
    }

    if(  !wbi_yarp_properties.check("urdf") && !wbi_yarp_properties.check("urdf_file") )
    {
        yError() << "yarpWholeBodyModel error: urdf not found in configuration files";
        return false;
    }

    std::string urdf_file;
    if( wbi_yarp_properties.check("urdf") )
    {
        urdf_file = wbi_yarp_properties.find("urdf").asString().c_str();
    }
    else
    {
        urdf_file = wbi_yarp_properties.find("urdf_file").asString().c_str();
    }

    yarp::os::ResourceFinder rf;
    if(  wbi_yarp_properties.check("verbose") )
    {
        rf.setVerbose();
    }

    std::string urdf_file_path = rf.findFile(urdf_file.c_str());

    // Create the list of joints added to the WBI to load a reduced
    // model with just that joints
    std::vecotr<std::string> consideredJoints;
    consideredJoints.resize(jointIdList.size());
    for(int wbi_numeric_id =0;  wbi_numeric_id < (int)jointIdList.size(); wbi_numeric_id++ )
    {
        wbi::ID joint_id;
        jointIdList.indexToID(wbi_numeric_id,joint_id);
        consideredJoints[wbi_numeric_id] = joint_id.toString();
    }

    // Load the model
    iDynTree::ModelLoader mdlLoader;
    mdlLoader.loadReducedModelFromFile(urdf_file_path,consideredJoints);



    //Loading information on the limits
    // if getLimitsFromControlBoard is set, get the limits from the real robot
    // otherwise use the limits in the urdf
    if( wbi_yarp_properties.check("getLimitsFromControlBoard") )
    {
        this->getLimitsFromControlBoard = true;
    }

    if( this->getLimitsFromControlBoard )
    {
        loadJointsControlBoardFromConfig(wbi_yarp_properties,
                                         jointIdList,
                                         controlBoardNames,
                                         controlBoardAxisList);

        dd.resize(controlBoardNames.size());
        ilim.resize(controlBoardNames.size());
    }

    // Populate the frame id list
    for(iDynTree::FrameIndex frameIdx=0;
        frameIdx < static_cast<iDynTree::FrameIndex>(m_kinDynComp.model().getNrOfFrames());
        frameIdx++ )
    {
        std::string frame_name = m_kinDynComp.model().getFrameName(frameIdx);
        frameIdList.addID(frame_name);
    }

    this->initDone = true;
    return this->initDone;
}

bool yarpWholeBodyModel::openDrivers(int bp)
{
    ilim[bp]=0; dd[bp]=0;
    if(!openPolyDriver(name+"model", robot, dd[bp], controlBoardNames[bp]))
        return false;
    bool ok = dd[bp]->view(ilim[bp]);   //if(!isRobotSimulator(robot))
    if(ok)
        return true;
    yError("Problem initializing drivers of %s", controlBoardNames[bp].c_str());
    return false;
}

bool yarpWholeBodyModel::closeDrivers()
{
    bool ok = true;
    for(int bp=0; bp < (int)controlBoardNames.size(); bp++ )
    {
        if( dd[bp] != 0 ) {
            ok = ok && dd[bp]->close();
            delete dd[bp];
            dd[bp] = 0;
        }
    }
    return ok;
}

bool yarpWholeBodyModel::close()
{
    return closeDrivers();
}

bool yarpWholeBodyModel::removeJoint(const wbi::ID &j)
{
    return false;
}

bool yarpWholeBodyModel::setYarpWbiProperties(const yarp::os::Property & yarp_wbi_properties)
{
    wbi_yarp_properties = yarp_wbi_properties;
    return true;
}

bool yarpWholeBodyModel::getYarpWbiProperties(yarp::os::Property & yarp_wbi_properties)
{
    yarp_wbi_properties = wbi_yarp_properties;
    return true;
}

int yarpWholeBodyModel::getDoFs()
{
    return dof;
}

bool yarpWholeBodyModel::addJoint(const wbi::ID &j)
{
    if( initDone )
    {
        return false;
    }

    if(!jointIdList.addID(j))
    {
        return false;
    }

    return true;
}

int yarpWholeBodyModel::addJoints(const wbi::IDList &j)
{
    int count = jointIdList.addIDList(j);
    return count;
}

void yarpWholeBodyModel::convertBasePose(const wbi::Frame &xBase, iDynTree::Transform & world_H_base)
{
    iDynTree::Position pos(xBase.p[0],xBase.p[1],xBase.p[2]);
    iDynTree::Rotation rot;

    // Both iDynTree::Rotation and wbi::Rotation are row-major
    memcpy(rot.data(),xBase.R.data,9*sizeof(double));

    world_H_base = iDynTree::Transform(rot,pos);
}

void yarpWholeBodyModel::convertBaseVelocity(const double *dxB, iDynTree::Twist & baseVel)
{
    for(int i=0; i < 6; i++)
    {
        baseVel(i) = dxB[i];
    }
}

void yarpWholeBodyModel::convertBaseAcceleration(const double *ddxB, iDynTree::Vector6 & baseAcc)
{
    for(int i=0; i < 6; i++)
    {
        baseAcc(i) = ddxB[i];
    }
}

void yarpWholeBodyModel::convertQ(const double *q_input, iDynTree::JointPosDoubleArray & q_idyntree)
{
    toEigen(q_idyntree) = Eigen::Map< const Eigen::VectorXd >(q_input,q_idyntree.size());
}

void yarpWholeBodyModel::convertDQ(const double *dq_input, iDynTree::JointDOFsDoubleArray & dq_idyntree)
{
    toEigen(dq_idyntree) = Eigen::Map< const Eigen::VectorXd >(dq_input,dq_idyntree.size());
}

void yarpWholeBodyModel::convertDDQ(const double *ddq_input, iDynTree::JointDOFsDoubleArray & ddq_idyntree)
{
    toEigen(ddq_idyntree) = Eigen::Map< const Eigen::VectorXd >(ddq_input,ddq_idyntree.size());
}

void yarpWholeBodyModel::convertGeneralizedTorques(const iDynTree::Wrench & baseWrench_idyntree,
                                                   const iDynTree::JointDOFsDoubleArray & jntTrqs_idyntree,
                                                         double * tau)
{
    Eigen::Map< Eigen::VectorXd > outGenTrqs(tau,jntTrqs_idyntree.size()+6);
    outGenTrqs.segment(0,6) = toEigen(baseWrench_idyntree);
    outGenTrqs.segment(6,jntTrqs_idyntree.size()) = toEigen(jntTrqs_idyntree);
}

void convertOutpuMatrix(const iDynTree::VectorDynSize & mat,
                               double * outMatBuffer)
{
    // Both iDynTree and WBI store matrices in row-major format, so it is quite trivial to copy them
    Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> outMat(outMatBuffer,mat.rows(),mat.cols());

    outMat = toEigen(mat);
}


void convertWorldGravity(const double *grav,
                         iDynTree::Vector3 & worldGravity)
{
    worldGravity(0) = grav[0];
    worldGravity(1) = grav[1];
    worldGravity(2) = grav[2];
}



void yarpWholeBodyModel::updateState()
{
    m_kinDynComp.setRobotState(m_modelPos.worldBasePos(),
                               m_modelPos.jointPos(),
                               m_baseVel,
                               m_jntVel,
                               m_worldGravity);
}

void yarpWholeBodyModel::posToHomMatrix(double * pos, iDynTree::Transform & frameWithoutOffset_H_frameWithOffset)
{
    iDynTree::Position pos_idyn(pos[0],pos[1],pos[2]);
    frameWithoutOffset_H_frameWithOffset = iDynTree::Transform(iDynTree::Rotation::Identity(),pos_idyn);
}

void yarpWholeBodyModel::posToAdjMatrix(double * pos,
                                        const iDynTree::Rotation & world_R_frame,
                                              iDynTree::Transform & frameWithoutOffset_world_H_frameWithOffset_world)
{
    iDynTree::Position pos_idyn_frame(pos[0],pos[1],pos[2]);
    iDynTree::Position pos_world = world_R_frame*pos_idyn_frame;

    frameWithoutOffset_world_H_frameWithOffset_world = iDynTree::Transform(iDynTree::Rotation::Identity(),pos_world);
}



bool yarpWholeBodyModel::getJointLimitFromControlBoard(double *qMin, double *qMax, int joint)
{
            int controlBoardId = controlBoardAxisList[joint].first;
            int index = controlBoardAxisList[joint].second;
            assert(ilim[controlBoardId]!=NULL);
            bool res = ilim[controlBoardId]->getLimits(index, qMin, qMax);
            if(res)
            {
                *qMin = (*qMin) * yarpWbi::Deg2Rad;   // convert from deg to rad
                *qMax = (*qMax) * yarpWbi::Deg2Rad;   // convert from deg to rad
            }
            return res;
}

bool yarpWholeBodyModel::getJointLimits(double *qMin, double *qMax, int joint)
{
    if ( (joint < 0 || joint >= (int)jointIdList.size()) && joint != -1 )
    {
        return false;
    }

    if (this->getLimitsFromControlBoard)
    {
        bool ret = true;
        for(int bp=0; bp < (int)controlBoardNames.size(); bp++ )
        {
            ret = ret && openDrivers(bp);
        }

        if( ret )
        {
            if(joint>=0)
            {
                ret = ret &&  getJointLimitFromControlBoard(qMin,qMax,joint);
            }
            else
            {
                int n = jointIdList.size();
                for(int i=0; i<n; i++)
                    ret = ret && getJointLimitFromControlBoard(qMin+i, qMax+i, i);
            }
        }

        ret = ret && closeDrivers();

        return ret;

    } else {
      // OLD IMPLEMENTATION
      all_q_min = p_model->getJointBoundMin();
      all_q_max = p_model->getJointBoundMax();

      if( joint == -1 ) {
         //Get all joint limits
          convertQ(all_q_min,qMin);
          convertQ(all_q_max,qMax);
      } else {
          *qMin = all_q_min[wbiToiDynTreeJointId[joint]];
          *qMax = all_q_max[wbiToiDynTreeJointId[joint]];
      }
      return true;
    }
}


bool yarpWholeBodyModel::computeH(double *q, const Frame &xBase, int frameId, Frame &H, double *pos)
{
    if (!this->initDone)
    {
        return false;
    }

    // Cast the frameId  to a iDynTree::FrameIndex
    iDynTree::FrameIndex frameIndex = static_cast<iDynTree::FrameIndex>(frameId);
    if ( !(m_kinDynComp.model().isValidFrameIndex(frameIndex)) && frameId != COM_LINK_ID )
    {
        return false;
    }

    convertBasePose(xBase,m_modelPos.worldBasePos());
    convertQ(q,m_modelPos.jointPos());
    this->updateState();

    iDynTree::Transform world_H_frame = iDynTree::Transform::Identity();

    if (frameId != COM_LINK_ID)
    {
        world_H_frame = m_kinDynComp.getWorldTransform(frameIndex);

        // Handle pos argument
        if (pos)
        {
            iDynTree::Transform frameWithoutOffset_H_frameWithOffset;
            posToHomMatrix(pos,frameWithoutOffset_H_frameWithOffset);
            iDynTree::Transform world_H_frameWithoutOffset = world_H_frame;
            world_H_frame = world_H_frameWithoutOffset*frameWithoutOffset_H_frameWithOffset;
        }
    }
    else
    {
        world_H_frame.setPosition(m_kinDynComp.getCenterOfMassPosition());
    }

    iDynTree::Matrix4x4 homMat = world_H_frame.asHomogeneousTransform();
    H.set4x4Matrix(homMat.data());
    return true;
}


bool yarpWholeBodyModel::computeJacobian(double *q, const Frame &xBase, int frameId, double *J, double *pos)
{
    if( !this->initDone )
    {
        return false;
    }

    // Cast the frameId  to a iDynTree::FrameIndex
    iDynTree::FrameIndex frameIndex = static_cast<iDynTree::FrameIndex>(frameId);
    if ( !(m_kinDynComp.model().isValidFrameIndex(frameIndex)) && frameId != COM_LINK_ID )
    {
        return false;
    }

    convertBasePose(xBase,m_modelPos.worldBasePos());
    convertQ(q,m_modelPos.jointPos());
    this->updateState();

    //Get Jacobian, the one of the link or the one of the COM
    bool ok = true;
    if (frameId != COM_LINK_ID)
    {
        ok = m_kinDynComp.getFrameFreeFloatingJacobian(frameIndex,m_frameJacobian);
    } else {
        ok = m_kinDynComp.getCentroidalAverageVelocityFreeFloatingJacobian(m_frameJacobian);
    }

    return ok;
}

bool yarpWholeBodyModel::computeDJdq(double *q, const Frame &xBase, double *dq, double *dxB, int frameId, double *dJdq, double *pos)
{
    if( !this->initDone )
    {
        return false;
    }

    // Cast the frameId  to a iDynTree::FrameIndex
    iDynTree::FrameIndex frameIndex = static_cast<iDynTree::FrameIndex>(frameId);
    if ( !(m_kinDynComp.model().isValidFrameIndex(frameIndex)) && frameId != COM_LINK_ID )
    {
        return false;
    }

    convertBasePose(xBase,m_modelPos.worldBasePos());
    convertQ(q,m_modelPos.jointPos());
    this->updateState();


    return false;
}

bool yarpWholeBodyModel::forwardKinematics(double *q, const Frame &xB, int linkId, double *x, double * pos)
{
    wbi::Frame output;
    bool ok = computeH(q,xB,linkId,output,pos);

    x[0] = output.p[0];
    x[1] = output.p[1];
    x[2] = output.p[2];

    // Remember: both YARP and WBI store matrices in row-major order
    Matrix dcm(3,3);
    memcpy(dcm.data(),output.R.data,9*sizeof(double));

    Vector axisangle = dcm2axis(dcm);

    x[3] = axisangle(0);
    x[4] = axisangle(1);
    x[5] = axisangle(2);
    x[6] = axisangle(3);

    return ok;
}

bool yarpWholeBodyModel::inverseDynamics(double *q, const Frame &xB,
                                         double *dq, double *dxB,
                                         double *ddq, double *ddxB,
                                         double *g, double *tau)
{
    if( !this->initDone )
    {
        return false;
    }

    //Converting local wbi positions/velocity/acceleration to iDynTree one
    convertBasePose(xB, m_modelPos.worldBasePos());
    convertQ(q, m_modelPos.jointPos());
    convertBaseVelocity(dxB, m_baseVel);
    convertDQ(dq, m_jntVel);
    convertBaseAcceleration(ddxB, m_baseAcc);
    convertDDQ(ddq, m_jntAcc);
    convertWorldGravity(g,m_worldGravity);

    // Update iDynTree state
    this->updateState();

    //Computing inverse dynamics
    m_kinDynComp.computeFreeFloatingInverseDynamics(m_baseAcc,m_jntAcc,m_genTrqs.baseWrench(),m_genTrqs.jointTorques());

    // Copy result in output buffer
    convertGeneralizedTorques(m_genTrqs.baseWrench(),m_genTrqs.jointTorques,tau);

    return true;
}

bool yarpWholeBodyModel::computeMassMatrix(double *q, const Frame &xBase, double *M)
{
    if( !this->initDone )
    {
        return false;
    }

    //Converting local wbi positions/velocity/acceleration to iDynTree one
    convertBasePose(xBase, m_modelPos.worldBasePos());
    convertQ(q, m_modelPos.jointPos());

    // Update iDynTree state
    this->updateState();

    //Computing inverse dynamics
    m_kinDynComp.getFreeFloatingMassMatrix(m_massMatrix);

    // Copy result in output buffer
    convertOutpuMatrix(m_massMatrix,M);

    return true;
}

bool yarpWholeBodyModel::computeGeneralizedBiasForces(double *q, const Frame &xBase, double *dq, double *dxB, double *g, double *h)
{
    //Converting local wbi positions/velocity/acceleration to iDynTree one
    convertBasePose(xBase, m_modelPos.worldBasePos());
    convertQ(q, m_modelPos.jointPos());
    convertBaseVelocity(dxB, m_baseVel);
    convertDQ(dq, m_jntVel);
    convertWorldGravity(g,m_worldGravity);

    // Update iDynTree state
    this->updateState();

    //Computing inverse dynamics
    m_kinDynComp.computeFreeFloatingBiasForces(m_genTrqs.baseWrench(),m_genTrqs.jointTorques());

    // Copy result in output buffer
    convertGeneralizedTorques(m_genTrqs.baseWrench(),m_genTrqs.jointTorques,tau);

    return true;
}


bool yarpWholeBodyModel::computeCentroidalMomentum(double *q, const Frame &xBase, double *dq, double *dxB, double *h)
{
    convertBasePose(xBase, m_modelPos.worldBasePos());
    convertQ(q, m_modelPos.jointPos());
    convertBaseVelocity(dxB, m_baseVel);
    convertDQ(dq, m_jntVel);

    return false;
}

const wbi::IDList & yarpWholeBodyModel::getJointList()
{
    return jointIdList;
}

const wbi::IDList & yarpWholeBodyModel::getFrameList()
{
    return frameIdList;
}



