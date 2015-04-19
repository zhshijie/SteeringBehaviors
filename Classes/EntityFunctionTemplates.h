//
//  EntityFunctionTemplates.h
//  SmallEntityd
//
//  Created by 张世杰 on 15/4/18.
//
//

#ifndef __SmallEntityd__EntityFunctionTemplates__
#define __SmallEntityd__EntityFunctionTemplates__

#include "cocos2d.h"

USING_NS_CC;
template <class T, class conT>
void EnforceNonPenetrationContraint(T entity, const conT& others)
{
    typename conT::const_iterator it;
    
    //iterate through all entities checking for any overlap of bounding
    //radii
    for (it=others.begin(); it != others.end(); ++it)
    {
        //make sure we don't check against this entity
        if (*it == entity) continue;
        
        //calculate the distance between the positions of the entities
        Vec2 ToEntity = entity->getPosition() - (*it)->getPosition();
        
        double DistFromEachOther = ToEntity.length();
        
        //if this distance is smaller than the sum of their radii then this
        //entity must be moved away in the direction parallel to the
        //ToEntity vector
        double AmountOfOverLap = (*it)->getContentSize().height/2 + entity->getContentSize().height/2 -
        DistFromEachOther;
        
        if (AmountOfOverLap >= 0)
        {
            //move the entity a distance away equivalent to the amount of overlap.
            entity->setPosition(entity->getPosition() + (ToEntity/DistFromEachOther) *
                           AmountOfOverLap);
        }
    }//next entity
}






#endif /* defined(__SmallEntityd__EntityFunctionTemplates__) */
