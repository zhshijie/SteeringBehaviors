//
//  CellSpacePartition.h
//  SmallEntityd
//
//  Created by 张世杰 on 15/4/19.
//
//

#ifndef __SmallEntityd__CellSpacePartition__
#define __SmallEntityd__CellSpacePartition__

#include <stdio.h>


#include "InvertedAABBox2D.h"

template <class entity>
struct Cell {
    //存在这个单元的所有实体
    
    std::vector<entity> Members;
    
    //单元盒的包围盒子
    InvertedAABBox2D BBox;
    Cell(Vec2 topleft,Vec2 botright):BBox(InvertedAABBox2D(topleft,botright)){}
    
};


template <class entity>
class CellSpacePartition {
private:
    
    //空间中所有的单元格
    std::vector<Cell<entity>> m_Cells;
    
    //附近的单元格
    std::vector<entity> m_Neighbors;
    
    //迭代器
    typename std::vector<entity>::iterator m_curNeighbor;
    
    //实体所在世界空间的宽度和长度
    double m_dSpaceWidth;
    double m_dSpaceHeight;
    
    //空间要划分的单元格数目
    int m_iNumCellsX;
    int m_iNumCellsY;
    
    double m_dCellSizeX;
    double m_dCellSizeY;
    
    
    //给定一个游戏空间的位置，获得相应的单元索引
    inline int PositionToIndex(const Vec2 &pos) const;
    
public:
    CellSpacePartition(double width, // 游戏空间的宽
                       double heigth,  //游戏空间的高
                       int cellsX,   //水平的单元格数量
                       int cellsY,   //竖直的
                       int MaxEntitys); //可以添加的实体的数量
    
    std::vector<entity> getNeighbors()const{return m_Neighbors;};

    //通过分配它们到合适饿单元，把实体加到类里
    inline void AddEntity(const entity& ent);
    
    //更新实体所在的单元
    inline void UpdateEntity(const entity& ent,Vec2 OldPos);
    
    
    //计算所有目标的邻居，存在邻居向量中
    inline void CalculateNeighbors(Vec2 targetPos,double QueryRadius);
    
    
    //获得邻居集合
    
    //返回邻居向量中的第一个实体
    inline entity& begin();
    inline entity& next();
    inline bool end();
    void EmptyCells();
};


template<class entity>
CellSpacePartition<entity>::CellSpacePartition(double  width,        //width of 2D space
                                               double  height,       //height...
                                               int    cellsX,       //number of divisions horizontally
                                               int    cellsY,       //and vertically
                                               int    MaxEntitys):  //maximum number of entities to partition
m_dSpaceWidth(width),
m_dSpaceHeight(height),
m_iNumCellsX(cellsX),
m_iNumCellsY(cellsY),
m_Neighbors(MaxEntitys, entity())
{
    //calculate bounds of each cell
    m_dCellSizeX = width  / cellsX;
    m_dCellSizeY = height / cellsY;
    
    
    //create the cells
    for (int y=0; y<m_iNumCellsY; ++y)
    {
        for (int x=0; x<m_iNumCellsX; ++x)
        {
            double left  = x * m_dCellSizeX;
            double right = left + m_dCellSizeX;
            double top   = y * m_dCellSizeY;
            double bot   = top + m_dCellSizeY;
            
            m_Cells.push_back(Cell<entity>(Vec2(left, top), Vec2(right, bot)));
        }
    }
}

template<class entity>
void CellSpacePartition<entity>::CalculateNeighbors(cocos2d::Vec2 targetPos, double QueryRadius)
{
    //创建一个迭代器，设置邻近的表的开始
    typename std::vector<entity>::iterator curNbor = m_Neighbors.begin();
    
    //创建查询盒，是目标查询区域的包围盒
    InvertedAABBox2D QueryBox(targetPos - Vec2(QueryRadius,QueryRadius),targetPos+ Vec2(QueryRadius,QueryRadius));
    
    //迭代每一个单元，测试它的包围盒是否和查询盒重叠
    typename std::vector<Cell<entity>>::iterator curCell;
    for (curCell = m_Cells.begin(); curCell!=m_Cells.end(); ++curCell) {
        
        //测试这个单元是否含有成员，是否查询盒子
        if (curCell->BBox.isOverlappedWith(QueryBox)&&!curCell->Members.empty()) {
            typename std::vector<entity>::iterator it = curCell->Members.begin();
            for (it; it!=curCell->Members.end(); ++it) {
                if ((*it)->getPosition().distance(targetPos)<QueryRadius*QueryRadius) {
                    *curNbor++ = *it;
                }
            }
        }
    }
    *curNbor = 0;
}



template<class entity>
void CellSpacePartition<entity>::EmptyCells()
{
    typename std::vector<entity>::iterator it = m_Cells.begin();
    for (it; it!=m_Cells.end(); ++it) {
        (*it)->Members.clear();
    }
}



template<class entity>
inline int CellSpacePartition<entity>::PositionToIndex(const cocos2d::Vec2 &pos) const
{
    int idx = (int)(m_iNumCellsX * pos.x / m_dSpaceWidth) +
    ((int)((m_iNumCellsY) * pos.y / m_dSpaceHeight) * m_iNumCellsX);
    
    //if the entity's position is equal to vector2d(m_dSpaceWidth, m_dSpaceHeight)
    //then the index will overshoot. We need to check for this and adjust
    if (idx > m_Cells.size()-1) idx = m_Cells.size()-1;
    
    return idx;
}


template<class entity>
inline void CellSpacePartition<entity>::AddEntity(const entity& ent)
{
    assert (ent);
    
    int sz = m_Cells.size();
    int idx = PositionToIndex(ent->getPosition());
    
    m_Cells[idx].Members.push_back(ent);
}

template<class entity>
inline void CellSpacePartition<entity>::UpdateEntity(const entity&  ent,
                                                     Vec2       OldPos)
{
    //if the index for the old pos and the new pos are not equal then
    //the entity has moved to another cell.
    int OldIdx = PositionToIndex(OldPos);
    int NewIdx = PositionToIndex(ent->getPosition());
    
    if (NewIdx == OldIdx) return;
    
    //the entity has moved into another cell so delete from current cell
    //and add to new one
    m_Cells[OldIdx].Members.remove(ent);
    m_Cells[NewIdx].Members.push_back(ent);
}


#endif /* defined(__SmallEntityd__CellSpacePartition__) */
