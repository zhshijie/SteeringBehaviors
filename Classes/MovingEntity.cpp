//
//  MovingEntity.cpp
//  SmallEntityd
//
//  Created by 张世杰 on 15/4/16.
//
//

#include "MovingEntity.h"

MovingEntity::MovingEntity(Vec2 Veloctitu,Vec2 Heading,Vec2 Side,double Mass,double MaxSpeed,double MaxForce,double MaxTurnRate):
BaseGameEntity(0),
m_vVeloctity(Veloctitu),
m_dMass(Mass),
m_dMaxForce(MaxForce),
m_dMaxSpeed(MaxSpeed),
m_dMaxTurnRate(MaxTurnRate),
m_vHeading(Heading),
m_vSide(Side)
{
    
}