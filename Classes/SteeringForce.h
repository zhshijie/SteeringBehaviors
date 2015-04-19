//
//  SteeringForce.h
//  SmallEntityd
//
//  Created by 张世杰 on 15/4/16.
//
//

#ifndef __SmallEntityd__SteeringForce__
#define __SmallEntityd__SteeringForce__

#include <stdio.h>
#include "cocos2d.h"
#include "Wall2D.h"

class Vehicle;
class BaseGameEntity;



//the radius of the constraining circle for the wander behavior
const double WanderRad    = 12;
//distance the wander circle is projected in front of the agent
const double WanderDist   = 2.0;
//the maximum amount of displacement along the circle each frame
const double WanderJitterPerSec = 80.0;

//used in path following
const double WaypointSeekDist   = 20;

const double SearchRad = 200;

USING_NS_CC;
class SteeringForce:public  Ref {
  
private:
    
    enum Decleration{slow = 9,normal = 6,fast = 3};
    enum behavior_type
    {
        none               = 0x00000,
        seek               = 0x00002,
        flee               = 0x00004,
        arrive             = 0x00008,
        wander             = 0x00010,
        cohesion           = 0x00020,
        separation         = 0x00040,
        allignment         = 0x00080,
        obstacle_avoidance = 0x00100,
        wall_avoidance     = 0x00200,
        follow_path        = 0x00400,
        pursuit            = 0x00800,
        evade              = 0x01000,
        interpose          = 0x02000,
        hide               = 0x04000,
        flock              = 0x08000,
        offset_pursuit     = 0x10000,
    };

    int           m_iFlags;

public:
    SteeringForce(Vehicle*);
    
    void setVehicle(Vehicle*v){m_pVehicle = v;};
    
    Vec2 Calculate();
    Vec2 FOrwardComponent();
    Vec2 SideComponent();
    
    
    double TurnaroundTime(const Vehicle*pAgent,Vec2 TargetPos);
    
    void SeekOn(){m_iFlags |= seek;}
    void SeekOff()  {if(On(seek))   m_iFlags ^=seek;}

    void FleeOn(){m_iFlags|= flee;};
    void FleeOff(){if(On(flee)) m_iFlags^=flee;};
    
    void ArriveOn(){m_iFlags |= arrive;};
    void ArriveOff(){if(On(arrive))m_iFlags^=arrive;};
    
    void PursuitOn(){m_iFlags |=pursuit;};
    void PursuitOff(){if(On(pursuit)) m_iFlags^=pursuit;};
    
    void EvadeOn(){m_iFlags |= evade;};
    void EvadeOff(){if(On(evade)) m_iFlags ^= evade;};
    
    void WanderOn(){m_iFlags |= wander;};
    void WanderOff(){if(On(wander))m_iFlags^=wander;};
    
    void ObstacleAvoidanceOn(){m_iFlags |= obstacle_avoidance;};
    void ObstacleAvoidanceOff(){if(On(obstacle_avoidance))m_iFlags ^=obstacle_avoidance;};
    
    void WallAvoidanceOn(){m_iFlags |= wall_avoidance;};
    void WallAvoidanceOff(){if(On(wall_avoidance)) m_iFlags ^= wall_avoidance;};
    
    void InterposeOn(){m_iFlags |= interpose;};
    void InterposeOff(){if(On(interpose)) m_iFlags^= interpose;};
    
    void HideOn(){m_iFlags|= hide;};
    void HideOff(){if(On(hide))m_iFlags ^= hide;};
    
    
    void PathFollowingOn(){m_iFlags |= follow_path;};
    void PathFollowingOff(){if(On(follow_path))m_iFlags ^= follow_path; };
    
    void OffsetPursuitOn(){m_iFlags |= offset_pursuit;};
    void OffsetPursuitOff(){if(On(offset_pursuit)) m_iFlags^=offset_pursuit;};
    
    void SeparationOn(){m_iFlags |= separation;};
    void SeparationOff(){if(On(separation)) m_iFlags ^= separation;};
    
    void AlignmentOn(){m_iFlags |= allignment;};
    void AlignmentOff(){if(On(allignment))m_iFlags ^= allignment;};
    
    void CohesionOn(){m_iFlags |= cohesion;};
    void CohesionOff(){if(On(cohesion)) m_iFlags ^= cohesion;};
    
private:
    Vec2 Seek(Vec2 target);
    Vec2 Flee(Vec2 target);
    Vec2 Arrive(Vec2 target,Decleration deceleration);
    Vec2 Pursuit(const Vehicle*evader);
    Vec2 Evade(const Vehicle *pursuer);
    Vec2 Wander();
    Vec2 ObstacleAvoidance(const std::vector<BaseGameEntity*>);
    Vec2 WallAvoidance(const std::vector<Wall2D>& walls);
    Vec2 Interpose(const Vehicle* AgentA,const Vehicle * AgentB);
    Vec2 Hide(const Vehicle* target,std::vector<Vehicle*>& obstacles);
    Vec2 PathFollow();
    Vec2 OffsetPursuit(const Vehicle* leader,const Vec2 offset);
    Vec2 Separation(const std::vector<Vehicle*>& neighbors);
    Vec2 Alignment(const std::vector<Vehicle*>&neighbors);
    Vec2 Cohesion(const std::vector<Vehicle*>&neighbors);
    
    bool AccumulateForce(Vec2 &RunningTot,Vec2 ForceToAdd);
    void CreateFeelers();
    Vec2 GetHidingPosition(const Vec2 posOb,const double radiusOb,const Vec2 posTarget);
    
    Vehicle *m_pVehicle;
    Vec2 m_vSteeringForce;
    double m_dDBoxLength;
    std::vector<Vec2> m_feelers;
    
    //wander圈的半径
    CC_SYNTHESIZE(double, m_dWanderRadius, WanderRadius);
    //wander圈凸出在智能体前面的距离
    CC_SYNTHESIZE(double, m_dWanderDistance, WanderDistance);
    //每秒加到目标的随机位移的最大值
    CC_SYNTHESIZE(double, m_dWanderJitter, WanderJitter);
    
    CC_SYNTHESIZE(Vec2, m_vWanderTaget, WanderTaget);
    
    CC_SYNTHESIZE(double, m_WaypointSeekdistSq, WaypointSeekdistSq);
    
    
    
    bool      On(behavior_type bt){return (m_iFlags & bt) == bt;}
    std::vector<Vehicle*> tagNeighbors(const Vehicle * entity, std::vector<Vehicle*> containerOfEntities,double radius);
};

#endif /* defined(__SmallEntityd__SteeringForce__) */
