#ifndef INVAABBOX2D_H
#define INVAABBOX2D_H
//-----------------------------------------------------------------------------
//
//  Name:   InvertedAABBox2D.h
//
//  Author: Mat Buckland (www.ai-junkie.com)
//
//  Desc:   v simple inverted (y increases down screen) axis aligned bounding
//          box class
//-----------------------------------------------------------------------------

#include "cocos2d.h"
using namespace cocos2d;

class InvertedAABBox2D
{
private:
  
  Vec2  m_vTopLeft;
  Vec2  m_vBottomRight;

  Vec2  m_vCenter;
  
public:

  InvertedAABBox2D(Vec2 tl,
                   Vec2 br):m_vTopLeft(tl),
                                m_vBottomRight(br),
                                m_vCenter((tl+br)/2.0)
  {}

  //returns true if the bbox described by other intersects with this one
  bool isOverlappedWith(const InvertedAABBox2D& other)const
  {
    return !((other.Top() > this->Bottom()) ||
           (other.Bottom() < this->Top()) ||
           (other.Left() > this->Right()) ||
           (other.Right() < this->Left()));
  }

  
  Vec2 TopLeft()const{return m_vTopLeft;}
  Vec2 BottomRight()const{return m_vBottomRight;}

  double    Top()const{return m_vTopLeft.y;}
  double    Left()const{return m_vTopLeft.x;}
  double    Bottom()const{return m_vBottomRight.y;}
  double    Right()const{return m_vBottomRight.x;}
  Vec2 Center()const{return m_vCenter;}

};
  
#endif