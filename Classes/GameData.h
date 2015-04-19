//
//  GameData.h
//  SmallEntityd
//
//  Created by 张世杰 on 15/4/17.
//
//

#ifndef __SmallEntityd__GameData__
#define __SmallEntityd__GameData__

#include <stdio.h>
#include "BaseGameEntity.h"
#include "cocos2d.h"
#include "Wall2D.h"
#include "CellSpacePartition.h"
USING_NS_CC;

class Vehicle;

class GameData:Ref
{
    
private:
    static GameData* m_gameData;
    std::vector<Vehicle*> allEntity;
    std::vector<Wall2D> allwall;
    std::vector<BaseGameEntity*>allObstacle;
    CellSpacePartition<Vehicle*> *m_cellSpace;
public:
    static GameData * Instance();
    void addEntity(Vehicle *);
    std::vector<Vehicle*> getEntityVector()const{return allEntity;};
    
    
    CellSpacePartition<Vehicle*>* getCellSpace()const{return m_cellSpace;};
    void setCellSpace(CellSpacePartition<Vehicle*>* &cellspace){m_cellSpace = cellspace;};
    
    std::vector<Wall2D> getWallls()const {return allwall;};
    void addWall(Wall2D wall);
    
    
    void addObstacle(BaseGameEntity*);
    std::vector<BaseGameEntity*> getObstacle() const {return allObstacle;};
    
    //标记在范围内的所有建筑物
    void tagObstaclesWithinViewRange(Vehicle* vehicle,double length);
};

#define MGameData GameData::Instance();
#endif /* defined(__SmallEntityd__GameData__) */
