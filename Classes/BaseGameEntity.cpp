//
//  BaseGameEntity.cpp
//  FiniteStateMachine
//
//  Created by 张世杰 on 15/4/13.
//
//

#include "BaseGameEntity.h"

int BaseGameEntity::M_iNextValidID = 0;

void BaseGameEntity::SetID(int Val)
{
    if (Val>=M_iNextValidID) {
        m_ID = Val;
        M_iNextValidID++;
    }
}