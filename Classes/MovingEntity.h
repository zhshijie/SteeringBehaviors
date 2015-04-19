//
//  MovingEntity.h
//  SmallEntityd
//
//  Created by 张世杰 on 15/4/16.
//
//

#ifndef __SmallEntityd__MovingEntity__
#define __SmallEntityd__MovingEntity__

#include <stdio.h>
#include "BaseGameEntity.h"
#include "cocos2d.h"
USING_NS_CC;
class MovingEntity:public BaseGameEntity {
    
protected:
    //实体的速度
    CC_SYNTHESIZE(Vec2, m_vVeloctity, Veloctity) ;
    //实体的朝向向量
    CC_SYNTHESIZE(Vec2 ,m_vHeading,Heading);
    //垂直于实体朝向向量
    CC_SYNTHESIZE(Vec2, m_vSide, Side);
    //实体的位置
    
    //实体的质量
    CC_SYNTHESIZE(double,m_dMass, Mass) ;
    
    //实体的最大速度
  CC_SYNTHESIZE  (double ,m_dMaxSpeed,MaxSpeed);
    //实体产生的供自己动力的最大力
   CC_SYNTHESIZE (double, m_dMaxForce,MaxForce);
    
    //交通工具能旋转的最大速率 （弧度每秒）
    CC_SYNTHESIZE (double ,m_dMaxTurnRate,MaxTurnRate);
    
public:
    MovingEntity(Vec2 Veloctitu,Vec2 Heading,Vec2 Side,double Mass,double MaxSpeed,double MaxForce,double MaxTurnRate);
    
    };

#endif /* defined(__SmallEntityd__MovingEntity__) */
