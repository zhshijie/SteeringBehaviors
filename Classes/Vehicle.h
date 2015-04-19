//
//  Vehicle.h
//  SmallEntityd
//
//  Created by 张世杰 on 15/4/16.
//
//

#ifndef __SmallEntityd__Vehicle__
#define __SmallEntityd__Vehicle__

#include <stdio.h>
#include "MovingEntity.h"
#include "SteeringForce.h"
#include "cocos2d.h"
#include "Path.h"
#include "Wall2D.h"

#include "Smoother.h"
USING_NS_CC;

const int  NumSamplesForSmoothing =  10;
class Vehicle:public MovingEntity {
    
    
private:
    SteeringForce *m_pSteering;
    MoveTo * m_moveAction;
    RotateTo * m_rotateAction;
    Smoother<Vec2>*  m_pHeadingSmoother;

    Path* m_path;
    
    bool m_bSmoothingOn;
    Vec2 m_vSmoothedHeading;
    
    bool isCellSpace;
    
    virtual ~Vehicle();

public:
    Vehicle();
    virtual bool initWithFile(std::string &);
    static Vehicle* create(std::string fileName);
    
    
    bool        isSmoothingOn()const{return m_bSmoothingOn;}
    void        SmoothingOn(){m_bSmoothingOn = true;}
    void        SmoothingOff(){m_bSmoothingOn = false;}
    
    
    
    bool        isCellSpaceOn()const{return isCellSpace;}
    void        CellSpaceOn(){isCellSpace = true;}
    void        CellSpaceOff(){isCellSpace = false;}
    
    
    void setPath(Path* path){m_path = path;};
    Path* getPath()const {return m_path;};
    
    SteeringForce* getSteering()const{return m_pSteering;};
    
     Vec2  getSmoothedHeading()const{return m_vSmoothedHeading;}
    
    void update(float time_elasped);
    
    CC_SYNTHESIZE(Vec2, target, Target);
    CC_SYNTHESIZE(Vehicle*,m_evaderv,Evaderv);
    CC_SYNTHESIZE(Vehicle*, m_pursuer, Pursuer);
    CC_SYNTHESIZE(Vehicle*, m_interposeA, InterposeA);
    CC_SYNTHESIZE(Vehicle*, m_interposeB, InterposeB);
    CC_SYNTHESIZE(Vehicle*, hideTarget, HideTarget);
    CC_SYNTHESIZE(Vec2, m_offSet, OffsetToLeader);
    CC_SYNTHESIZE(Vehicle*, m_leader, Leader);

    void updateNeighbors(float time_elasped);
    
};
#endif /* defined(__SmallEntityd__Vehicle__) */
