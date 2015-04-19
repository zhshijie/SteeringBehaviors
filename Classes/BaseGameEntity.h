//
//  BaseGameEntity.h
//  FiniteStateMachine
//
//  Created by 张世杰 on 15/4/13.
//
//

#ifndef __FiniteStateMachine__BaseGameEntity__
#define __FiniteStateMachine__BaseGameEntity__

#include <stdio.h>

#include "cocos2d.h"
USING_NS_CC;
class BaseGameEntity:public Sprite {
private:
    
    //每个实体都具有一个唯一的识别数字
    int m_ID;
    
    //这是下一个有效的ID，每次BaseGameEntity被实例化这个值就被更新
    static int M_iNextValidID;
    
    //在构造函数中调用这个来确认ID被正确设置。在设置ID和增量前，它校验传递给方法的值是大于还是等于下一个有效值ID。
    
    void SetID(int Val);
    
    //
    bool m_bTag;
    
public:
    BaseGameEntity(int id)
    {
        SetID(id);
    }
    virtual ~BaseGameEntity(){};
    int ID()const{return m_ID;};
    
    bool isTagged()const{return m_bTag;};
    void setTag(bool tag){m_bTag = tag;};
    
};

#endif /* defined(__FiniteStateMachine__BaseGameEntity__) */
