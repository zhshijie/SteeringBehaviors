//
//  SteeringForce.cpp
//  SmallEntityd
//
//  Created by 张世杰 on 15/4/16.
//
//

#include "SteeringForce.h"
#include "Vehicle.h"
#include "GameData.h"
#include "Wall2D.h"
#include "CellSpacePartition.h"

SteeringForce::SteeringForce(Vehicle* ve):m_iFlags(0),m_dWanderDistance(WanderDist),m_dWanderJitter(WanderJitterPerSec),m_dWanderRadius(WanderRad),m_WaypointSeekdistSq(WaypointSeekDist*WaypointSeekDist)
{
    setVehicle(ve);

    
    
    m_vWanderTaget = Vec2(0, m_dWanderRadius);
}

Vec2 SteeringForce::Seek(cocos2d::Vec2 target)
{
    Vec2 DesiredVelocity = (target - m_pVehicle->getPosition()).getNormalized()*m_pVehicle->getMaxSpeed();
    
    return (DesiredVelocity - m_pVehicle->getVeloctity());
}


Vec2 SteeringForce::Flee(cocos2d::Vec2 target)
{
    Vec2 DesiredVelocity = ( m_pVehicle->getPosition()-target).getNormalized()*m_pVehicle->getMaxSpeed();
    return (DesiredVelocity - m_pVehicle->getVeloctity());
}

Vec2 SteeringForce:: Arrive(Vec2 target,Decleration deceleration)
{
    Vec2 ToTarget = target- m_pVehicle->getPosition();
    
    //计算到目标的位置距离
    float dist = ToTarget.length();
    
    //    log("________%f___________",dist);
    if (dist > 0.0) {
        
        //因为枚举Deceleration是整数int，所以需要这个值提供调整整减速度
        const double DecelerationTweaker = 0.3;
        
        
        //给定预定减速度，计算能到达目标位置所需的速度
        double speed = dist/((double)deceleration*DecelerationTweaker);
        
        //确保这个速度不超过最大值
        speed = MIN(speed, m_pVehicle->getMaxSpeed());
        
        
        //这边的处理和Seek一样，除了不需要标准化TOTarget向量
        //因为我们已经费力的地计算了它的长度：dist
        Vec2 desiredVelocity = ToTarget*speed/dist;
        
        
        
        return  (desiredVelocity-m_pVehicle->getVeloctity());
        
    }
    
    return Vec2(0,0);
    
}

Vec2 SteeringForce::Pursuit(const Vehicle *evader)
{
    //如果逃避者在前面，而且面对着智能体
    //那么我们可以正好靠近逃避者
    Vec2 ToEvader = evader->getPosition() - m_pVehicle->getPosition();
    
    double RelativeHeading = m_pVehicle->getHeading().dot(evader->getHeading());
    if (ToEvader.dot(m_pVehicle->getHeading())>0&&RelativeHeading<-0.95) {
        return Seek(evader->getPosition());
    }
    
    //预测逃避者的位置
    
    double LookAheadTime = ToEvader.length()/(m_pVehicle->getMaxSpeed()+evader->getVeloctity().length());
    LookAheadTime +=TurnaroundTime(m_pVehicle,evader->getPosition());
    return  Seek(evader->getPosition()+evader->getVeloctity()*LookAheadTime);
    
}

Vec2 SteeringForce::Evade(const Vehicle *pursuer)
{
    Vec2 ToPursuer = pursuer->getPosition()- m_pVehicle->getPosition();
    
    //预测的时间正比于追逐着的距离，反比雨智能体的速度和
    double LookAheadTime = ToPursuer.length()/(m_pVehicle->getMaxSpeed()+pursuer->getVeloctity().length());
    
    return Flee(pursuer->getPosition()+pursuer->getVeloctity()*LookAheadTime);
    
}


Vec2 SteeringForce::Wander()
{
    
    m_vWanderTaget+=Vec2((CCRANDOM_0_1()*2-1)*m_dWanderJitter,(CCRANDOM_0_1()*2-1)*m_dWanderJitter);
    
    m_vWanderTaget.normalize();
    m_vWanderTaget*=m_dWanderRadius;
    
    auto targetLocal = m_vWanderTaget + Vec2(m_dWanderDistance,0);
    
//    log("-------m_vWanderTaget : %f,%f---------",m_vWanderTaget.x,m_vWanderTaget.y);
    auto targetWorld =  m_pVehicle->convertToWorldSpaceAR(targetLocal);
    
    return targetWorld-m_pVehicle->getPosition();
}


Vec2 SteeringForce::ObstacleAvoidance(const std::vector<BaseGameEntity *> obstacles)
{
    m_dDBoxLength = m_pVehicle->getVeloctity().length()/m_pVehicle->getMaxSpeed()*40+40;
    
    //标记范围内的所有障碍物
    GameData::Instance()->tagObstaclesWithinViewRange(m_pVehicle, m_dDBoxLength);
    
    
    //追踪最近相交的障碍物
    BaseGameEntity *ClosestIntersectingObstacle = nullptr;
    
    //追踪到CIB的距离
    double DistToClosestIP =  MAXFLOAT;
    
    //记录CIB被转化的局部坐标
    Vec2 LocalPosOfClosestObstacle;
    
    std::vector<BaseGameEntity*>::const_iterator curOb = obstacles.begin();
    while (curOb!= obstacles.end()) {
        if ((*curOb)->isTagged()) {
            Vec2 LocalPos =  m_pVehicle->convertToNodeSpaceAR((*curOb)->getPosition());
            
            //如果举办空间位置有个负的x值，那么它肯定在智能体后面
            if (LocalPos.x >=0) {
                //如果物体到x轴的距离小于它的半径＋检查盒宽度的一半
                //那么可能相交
                double ExpandeRadius = (*curOb)->getContentSize().height/2*(*curOb)->getScale()+m_pVehicle->getContentSize().height/2;
                
                if (fabs(LocalPos.y)<ExpandeRadius) {
                    //现在做线／圆周相交厕所吗 圆周圆心是（cx，cy）;
                
                    //相交点的公式是x ＝ cx ＋(-sqrt（  r*r - cY*cY）);
                    //我们只需看x的最小值，因为那是最近的相交点
                    
                    double cX = LocalPos.x;
                    double cY = LocalPos.y;
                    //我们只需一次计算上面等式的开方
                    
                    double SqrtPart = sqrt(ExpandeRadius*ExpandeRadius-cY*cY);
                    
                    double ip = SqrtPart;
                    if (ip<=0) {
                        ip = cX+SqrtPart;
                    }
                    if (ip<DistToClosestIP) {
                        DistToClosestIP = ip;
                        
                        ClosestIntersectingObstacle = *curOb;
                        LocalPosOfClosestObstacle = LocalPos;
                    }
                }
            }
        }
        ++curOb;
    }
    
    
    Vec2 SteeringForce;
    
    if (ClosestIntersectingObstacle) {
        
        //智能体越近，操控力越强
        double muliplier =1 + (m_dDBoxLength-LocalPosOfClosestObstacle.x)/m_dDBoxLength;
        
        
        //计算侧向力
        SteeringForce.y = (ClosestIntersectingObstacle->getContentSize().height/2*ClosestIntersectingObstacle->getScale()-LocalPosOfClosestObstacle.y)*muliplier;
        
        
        if (LocalPosOfClosestObstacle.y>0&&LocalPosOfClosestObstacle.y<ClosestIntersectingObstacle->getContentSize().height/2*ClosestIntersectingObstacle->getScale()) {
            SteeringForce.y = -SteeringForce.y;
            }
        
       
        //施加一个制动力，它正比于障碍物到交通工具的距离
        const double BrakingWeight = 0.3;
        SteeringForce.x = (ClosestIntersectingObstacle->getContentSize().height/2*ClosestIntersectingObstacle->getScale()-LocalPosOfClosestObstacle.x)*BrakingWeight;
    }
    
    //最后，把操控向量换成世界坐标
//    log("********SteeringForce:%f,%f*********",( m_pVehicle->convertToWorldSpaceAR(SteeringForce)-m_pVehicle->getPosition()).x,( m_pVehicle->convertToWorldSpaceAR(SteeringForce)-m_pVehicle->getPosition()).y);
    
    return m_pVehicle->convertToWorldSpaceAR(SteeringForce)-m_pVehicle->getPosition();
    
}

Vec2 SteeringForce::WallAvoidance(const std::vector<Wall2D> &walls)
{
    //创建实体的触须
    CreateFeelers();
    
    double DistToThisIp = 0.0;
    double DistToClosestIp = MAXFLOAT;
    
    //保存walls的向量的索引
    int ClosestWall = -1;
    
    Vec2    SteeringForce,
            point,
            ClosestPoint;
    
    float s,t;
    
    //逐个检查每条触须
    for (int flr = 0; flr<m_feelers.size(); ++flr) {
        
        //跑过每堵墙，检查相交点
        for (int w = 0;w<walls.size() ; w++ ) {
            if (Vec2::isLineIntersect(walls[w].From ,walls[w].To ,m_feelers[flr],m_pVehicle->getPosition() ,&s,&t)) {
                
                if (s <1&&t<1&&s>0&&t>0) {
                    ClosestPoint = m_pVehicle->getPosition() + t*(m_feelers[flr] - m_pVehicle->getPosition());
                    DistToThisIp = (ClosestPoint).length();
                    
                    
                    if (DistToThisIp < DistToClosestIp) {
                        DistToClosestIp = DistToThisIp;
                        ClosestWall = w;
                        ClosestPoint = point;
                    }
                }
                
            }
        }//下一堵墙
        
        if (ClosestWall >=0) {
            Vec2 OverShoot = m_feelers[flr]-ClosestPoint;
            SteeringForce = walls[ClosestWall].Normal*OverShoot.length();
        }
    }//下一跳触须
    
    
    
    return SteeringForce;
}


Vec2 SteeringForce::Interpose(const Vehicle *AgentA, const Vehicle *AgentB)
{
    //首先，我们需要算出在未来时间T时，这个两个智能体的位置。
    //交通工具以最大速度到达中点所花的时间近似于T
    Vec2 MidPoint = (AgentA->getPosition() + AgentB->getPosition())/2.0;
    
    double TimeToReachMidPoint = m_pVehicle->getPosition().distance(MidPoint)/m_pVehicle->getMaxSpeed();
    
    
    //现在我们有T，我们假设智能体A和智能体B将继续径直前行
    //推断得到他们将来的位置
    
    Vec2 APos = AgentA->getPosition() + AgentA->getVeloctity()*TimeToReachMidPoint;
    Vec2 BPos = AgentB->getPosition() + AgentB->getVeloctity()*TimeToReachMidPoint;
    
    MidPoint = (APos+BPos)/2.0;
    
    return Arrive(MidPoint, fast);
}
Vec2 SteeringForce::Hide(const Vehicle *target, std::vector<Vehicle *> &obstacles)
{
    double DistToClosest = MAXFLOAT;
    
    Vec2 BestHidingSpot;
    
    std::vector<Vehicle*>::iterator curOb = obstacles.begin();
    while (curOb!=obstacles.end()) {
        
        
        if (*curOb!=target && *curOb != m_pVehicle) {
            //计算这个障碍物的hide地点
            Vec2 HidingSpot = GetHidingPosition((*curOb)->getPosition(), (*curOb)->getContentSize().width/2, target->getPosition());
            double dist = HidingSpot.distance(m_pVehicle->getPosition());
            if (dist <DistToClosest) {
                DistToClosest = dist;
                BestHidingSpot = HidingSpot;
            }
        }
       
        ++curOb;
    }
    
    if (DistToClosest == MAXFLOAT) {
        return Evade(target);
    }
    
    return Arrive(BestHidingSpot, fast);
    
}


Vec2 SteeringForce::PathFollow()
{
    
    auto m_path = m_pVehicle->getPath();
    
    log("_____path.x:%f,path:%f_______",m_path->CurrentWaypoint().x,m_path->CurrentWaypoint().y);
    if (m_path->CurrentWaypoint().distance(m_pVehicle->getPosition())<m_WaypointSeekdistSq   ) {
        m_path -> SetNextWaypoint();
    }
    
    if (!m_path->Finished()) {
        return Seek(m_path->CurrentWaypoint());
    }
    else{
        return  Arrive(m_path->CurrentWaypoint(), normal);
    }
}


Vec2 SteeringForce::OffsetPursuit(const Vehicle *leader, const cocos2d::Vec2 offset)
{
    Vec2 worldOffsetPos =  leader->convertToWorldSpaceAR(offset);
    
    Vec2 toOffset = worldOffsetPos - m_pVehicle->getPosition();
    
    //预期时间正比领队和追逐者之间的距离
    //反比两个智能体的速度和
    double lookAheadTime = toOffset.length()/(m_pVehicle->getMaxSpeed()+leader->getVeloctity().length());
    return  Arrive(worldOffsetPos+leader->getVeloctity()*lookAheadTime, fast);
}

Vec2 SteeringForce::Separation(const std::vector<Vehicle *> &neighbors)
{
    Vec2 SteeringForce;
    for (int a = 0; a<neighbors.size(); a++) {
        if (neighbors[a]!=m_pVehicle&&neighbors[a]->isTagged()) {
            Vec2 ToAgent = m_pVehicle->getPosition()-neighbors[a]->getPosition();
            SteeringForce+=ToAgent.getNormalized()/ToAgent.length();
        }
    }
    
    log("---------SteeringForce.x:%f SteeringForce.y:%f------------",SteeringForce.x,SteeringForce.y);
    return SteeringForce;
}
Vec2 SteeringForce::Alignment(const std::vector<Vehicle *> &neighbors)
{
    Vec2 AverageHeading;
    int NeighborCount = 0;
    for (int a = 0; a<neighbors.size(); a++) {
        if (neighbors[a]!= m_pVehicle&&neighbors[a]->isTagged()) {
            AverageHeading+= neighbors[a]->getHeading();
            ++NeighborCount;
        }
    }
    
    if (NeighborCount>0 ) {
        AverageHeading = AverageHeading/(double)NeighborCount;
        AverageHeading = AverageHeading-m_pVehicle->getHeading();
    }
    
    log("---------AverageHeading:%f AverageHeading:%f------------",AverageHeading.x,AverageHeading.y);

    return AverageHeading;
}

Vec2 SteeringForce::Cohesion(const std::vector<Vehicle *> &neighbors)
{
    Vec2 CenterOfMass,SteeringForce;
    
    int NeighborCount = 0;
    for (int a = 0; a<neighbors.size(); a++) {
        if (neighbors[a]!=m_pVehicle&&neighbors[a]->isTagged()) {
            CenterOfMass += neighbors[a]->getPosition();
            ++NeighborCount;
        }
    }
    
    if (NeighborCount>0) {
        CenterOfMass = CenterOfMass/ (double)NeighborCount;
        SteeringForce = Seek(CenterOfMass);
    }
    
    log("---------CohesionSteeringForce.x:%f CohesionSteeringForce.y:%f------------",SteeringForce.x,SteeringForce.y);

    
    return SteeringForce;
}


Vec2 SteeringForce::Calculate()
{
    
    m_vSteeringForce  = Vec2::ZERO;
    Vec2 force;
    
    std::vector<Vehicle*> neighbors;
    
    if (On(allignment)||On(separation)||On(cohesion)) {
        
        if (m_pVehicle->isCellSpaceOn()) {
            
            CellSpacePartition<Vehicle*>* cellSpace = GameData::Instance()->getCellSpace();
            neighbors = cellSpace->getNeighbors();
            
        }else{
        auto data = GameData::Instance()->getEntityVector();
        
        neighbors = tagNeighbors(m_pVehicle,data,SearchRad);
        }
    }
    
    if (On(wall_avoidance)) {
        auto data = GameData::Instance()->getWallls();
        force = WallAvoidance(data);
        
        if (!AccumulateForce(m_vSteeringForce, force)) {
            return  m_vSteeringForce;
        }
    }
    
    if (On(obstacle_avoidance)) {
        
        auto data = GameData::Instance()->getObstacle();
        force = ObstacleAvoidance(data);
        
        if (!AccumulateForce(m_vSteeringForce, force)) {
            return  m_vSteeringForce;
        }
    }
    
    
    
    if (On(separation)) {
        
        force = Separation(neighbors);
        if (!AccumulateForce(m_vSteeringForce, force)) {
            return  m_vSteeringForce;
        }
    }
    
    if (On(cohesion)) {
        
        force = Cohesion(neighbors);
        if (!AccumulateForce(m_vSteeringForce, force)) {
            return  m_vSteeringForce;
        }
    }
    
    if (On(allignment)) {
        
        force = Alignment(neighbors);
        if (!AccumulateForce(m_vSteeringForce, force)) {
            return  m_vSteeringForce;
        }
    }
    
    
    
    if (On(hide)) {
        CCASSERT(m_pVehicle->getHideTarget()!=nullptr, "不存在躲避目标");
        auto data = GameData::Instance()->getEntityVector();
        force = Hide(m_pVehicle->getHideTarget(),data);
        if (!AccumulateForce(m_vSteeringForce, force)) {
            return  m_vSteeringForce;
        }
    }
    
    if (On(follow_path)) {
        force = PathFollow();
        
        if (!AccumulateForce(m_vSteeringForce, force)) {
            return  m_vSteeringForce;
        }
    }
    if (On(seek)) {
        
        force = Seek(m_pVehicle->getTarget());
        
        if (!AccumulateForce(m_vSteeringForce, force)) {
            return  m_vSteeringForce;
        }
        
    }
    
    if (On(flee)) {
        force = Flee(m_pVehicle->getTarget());
        if (!AccumulateForce(m_vSteeringForce, force)) {
            return  m_vSteeringForce;
        }
    }
    
    if (On(arrive)) {
        force = Arrive(m_pVehicle->getTarget(), fast);
        if (!AccumulateForce(m_vSteeringForce, force)) {
            return  m_vSteeringForce;
        }
    }
    
    if (On(offset_pursuit)) {
        force = OffsetPursuit(m_pVehicle->getLeader(), m_pVehicle->getOffsetToLeader());
        if (!AccumulateForce(m_vSteeringForce, force)) {
            return  m_vSteeringForce;
        }
    }
    
    if (On(pursuit)) {
        force = Pursuit(m_pVehicle->getEvaderv());
        if (!AccumulateForce(m_vSteeringForce, force)) {
            return  m_vSteeringForce;
        }
    }
    if (On(evade)) {
        force = Evade(m_pVehicle->getPursuer());
        if (!AccumulateForce(m_vSteeringForce, force)) {
            return  m_vSteeringForce;
        }
    }
    
    if (On(wander)) {
        force = Wander();
        if (!AccumulateForce(m_vSteeringForce, force)) {
            return  m_vSteeringForce;
        }
    }
    
    if (On(interpose)) {
        force = Interpose(m_pVehicle->getInterposeA(), m_pVehicle->getInterposeB());
        if (!AccumulateForce(m_vSteeringForce, force)) {
            return  m_vSteeringForce;
        }
    }
    
    return m_vSteeringForce;
}


bool SteeringForce::AccumulateForce(Vec2 &RunningTot,Vec2 ForceToAdd)
{
    //计算目前为止交通工具用了多少的操控力
    double MagnitudeSoFar = RunningTot.length();
    
    //计算还剩多少的操控力
    double MagnitudeRemaining = m_pVehicle->getMaxForce() - MagnitudeSoFar;
    
    if (MagnitudeRemaining<=0) {
        return false;
    }
    
    double MagnitudeToAdd = ForceToAdd.length();
    
    if (MagnitudeToAdd<MagnitudeRemaining) {
        RunningTot+=ForceToAdd;
    }else{
        RunningTot+= ForceToAdd.getNormalized()*MagnitudeRemaining;
    }
    return true;
}

//创建探索的触须
void SteeringForce::CreateFeelers()
{
    
    int fleer_length = 30;

    m_feelers.clear();
    
    Vec2 fleer_1 = Vec2(fleer_length,fleer_length);
    Vec2 fleer_2 = Vec2(fleer_length,-fleer_length);
    Vec2 fleer_3 = Vec2(fleer_length,0);
    Vec2 fleer_4 = Vec2(0,fleer_length);
    Vec2 fleer_5 = Vec2(0,-fleer_length);

    
    fleer_1 =  m_pVehicle->convertToWorldSpaceAR(fleer_1);
    fleer_2 = m_pVehicle->convertToWorldSpaceAR(fleer_2);
    fleer_3 = m_pVehicle->convertToWorldSpaceAR(fleer_3);
    fleer_4 = m_pVehicle->convertToWorldSpaceAR(fleer_4);
    fleer_5 = m_pVehicle->convertToWorldSpaceAR(fleer_5);

    
    m_feelers.push_back(fleer_1);
    m_feelers.push_back(fleer_2);
    m_feelers.push_back(fleer_3);
    m_feelers.push_back(fleer_4);
    m_feelers.push_back(fleer_5);

}

//转弯需求的时间
double SteeringForce:: TurnaroundTime(const Vehicle*pAgent,Vec2 TargetPos)
{
    Vec2 toTarget = (TargetPos- pAgent->getPosition()).getNormalized();
    
    double dot = pAgent->getHeading().dot(toTarget);
    
    //改变值得到预期行为
    //交通工具的最大转弯率越高，这个值越大
    //如果交通工具正在朝向目标位置的相反方向，
    //那么0.5这个值意味着这个函数返回1秒的时间以便交通工具转弯
    
    const double coefficient = 0.5;
    
    //如果目标在前面，那么点积为1
    //如果目标在后面，那么点积为－1
    //减去1，除以负coefficient，得到一个正值
    //且正比于交通工具和目标的转动角度位移
    return (dot - 1.0)/-coefficient;
}


//获得Hide的地点
Vec2 SteeringForce::GetHidingPosition(const cocos2d::Vec2 posOb, const double radiusOb, const cocos2d::Vec2 posTarget)
{
    const double DistanceFromBoundary = 30.0;
    
    double DistAway = radiusOb +DistanceFromBoundary;
    
    //计算从目标到物体的朝向
    Vec2 toOb = (posOb-posTarget).getNormalized();
    
    //确定大小并加到障碍物位置
    return (toOb*DistAway)+posOb;
    
}

//标记entity的邻居
std::vector<Vehicle*> SteeringForce::tagNeighbors(const Vehicle * entity, std::vector<Vehicle*> containerOfEntities,double radius)
{
    
    std::vector<Vehicle*> neigheors;
    for (std::vector<Vehicle*>::iterator curEntity = containerOfEntities.begin(); curEntity!= containerOfEntities.end();curEntity++) {
        (*curEntity)->setTag(false);
        
        Vec2 to = (*curEntity)->getPosition()-entity->getPosition();
        
        double range  = radius +(*curEntity)->getContentSize().height/2;
        
        if (((*curEntity)!=entity)&&(to.lengthSquared()<range*range)) {
            (*curEntity)->setTag(true);
            neigheors.push_back(*curEntity);
        }
    }
    return neigheors;
}
