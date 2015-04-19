//
//  Wall2D.h
//  SmallEntityd
//
//  Created by 张世杰 on 15/4/18.
//
//

#ifndef __SmallEntityd__Wall2D__
#define __SmallEntityd__Wall2D__

#include <stdio.h>
#include "cocos2d.h"
USING_NS_CC;

struct Wall2D{
public:
    Vec2 From;
    Vec2 To;
    Vec2 Normal;
    
};


#endif /* defined(__SmallEntityd__Wall2D__) */
