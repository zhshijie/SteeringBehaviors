//
//  Vehicle.cpp
//  SmallEntityd
//
//  Created by 张世杰 on 15/4/16.
//
//

#include "Vehicle.h"
#include "GameData.h"
#include "EntityFunctionTemplates.h"

Vehicle::Vehicle():MovingEntity(Vec2(0,0),Vec2(0,0),Vec2(0,0),1,40,150,100),m_bSmoothingOn(true),isCellSpace(false){

    m_pSteering = new SteeringForce(this);
    
    m_pHeadingSmoother = new Smoother<Vec2>(NumSamplesForSmoothing,Vec2(0,0));
    
    schedule(schedule_selector(Vehicle::updateNeighbors), 1);
    
}
Vehicle* Vehicle::create(std::string fileName)
{
    auto ve = new Vehicle();
    ve->initWithFile(fileName);
    return ve;
}


 bool Vehicle::initWithFile(std::string &fileName)
{
    if (Sprite::initWithFile(fileName)) {
        return true;
    }
    return false;
}

void Vehicle::update(float time_elasped)
{
    //计算操作行为的合力
    Vec2 SteeringForce = m_pSteering->Calculate();
    
    log("-------SteeringForce.x : %f,SteeringForce.y : %f--------",SteeringForce.x,SteeringForce.y);

    //加速度 ＝ 力／质量
    Vec2 acceleration = SteeringForce/m_dMass;
    
    //更新速度
    m_vVeloctity += acceleration*time_elasped;
    
    //确保交通工具不超过最大速度
    
    if (m_vVeloctity.length()>m_dMaxSpeed) {
        m_vVeloctity = m_vVeloctity.getNormalized()*m_dMaxSpeed;
    }
    
    double length = m_vVeloctity.length();
    if (length>m_dMaxSpeed) {
        m_vVeloctity.x = m_vVeloctity.x*(m_dMaxSpeed/length);
        m_vVeloctity.y = m_vVeloctity.y*(m_dMaxSpeed/length);
        
    }
    //更新位置
    Vec2 pos = getPosition();
    pos+= m_vVeloctity*time_elasped;
    
    
    if (m_vVeloctity.lengthSquared() >0.00000001) {
        
        m_vHeading = m_vVeloctity.getNormalized();
        m_vSide = m_vHeading.getPerp();
        
//            log("-------m_vHeading.x : %f,m_vHeading.y:%f--------",m_vHeading.x,m_vHeading.y);
        if (isSmoothingOn())
        {
            m_vSmoothedHeading = m_pHeadingSmoother->Update(getHeading());
        }
        
        //计算旋转角度
        auto cosVal = (m_vSmoothedHeading.getNormalized()).x;
        float degree;
        float radian = 0.0;
        if (cosVal != 1) {
             radian  =  acosf(cosVal);
            if (radian < 6.24) {
                degree = CC_RADIANS_TO_DEGREES(radian);
            }
            
        }
        if ((m_vHeading.getNormalized()).y>0) {
            degree = -degree;
        }
       
//        log("------cosVal:%f-------",cosVal);
//        log("------radian:%f-------",radian);
//        log("------degree:%f-------",degree);
//        log("------time_elasped:%f-------",time_elasped);

        setRotation(degree);
        setPosition(pos);
    }
    
    EnforceNonPenetrationContraint(this,GameData::Instance()->getEntityVector());
    
    
}

void Vehicle::updateNeighbors(float time_elasped)
{
    auto cellSpace = GameData::Instance()->getCellSpace();
    cellSpace->CalculateNeighbors(getPosition(),100);
}

Vehicle::~Vehicle()
{
    delete m_pSteering;
    delete m_pHeadingSmoother;
}
