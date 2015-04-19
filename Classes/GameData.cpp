//
//  GameData.cpp
//  SmallEntityd
//
//  Created by 张世杰 on 15/4/17.
//
//

#include "GameData.h"
#include "Vehicle.h"
#include "Wall2D.h"

GameData * GameData::m_gameData = nullptr;
GameData* GameData::Instance()
{
    if (m_gameData == nullptr) {
        m_gameData = new GameData();
    }
    return m_gameData;
}


void GameData::addEntity(Vehicle *newEntiy)
{
    allEntity.push_back(newEntiy);
}

void GameData::addObstacle(BaseGameEntity * obstacle)
{
    allObstacle.push_back(obstacle);
}


void GameData::tagObstaclesWithinViewRange(Vehicle *vehicle, double length)
{
    std::vector<BaseGameEntity*>::const_iterator ob = allObstacle.begin();
    
    while (ob!=allObstacle.end()) {
        
            double dis = vehicle->getPosition().distance((*ob)->getPosition())-(*ob)->getContentSize().width/2;
            if (dis<length) {
                (*ob)->setTag(true);
            }else{
                (*ob)->setTag(false);
            }
       
        ob++;
    }
}
void GameData::addWall(Wall2D m_wall)
{
    allwall.push_back(m_wall);
}

