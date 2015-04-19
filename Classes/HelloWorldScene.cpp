#include "HelloWorldScene.h"
#include "Vehicle.h"
#include "GameData.h"
#include "Wall2D.h"
#include "Path.h"
#include "CellSpacePartition.h"
USING_NS_CC;

Scene* HelloWorld::createScene()
{
    // 'scene' is an autorelease object
    auto scene = Scene::create();
    
    // 'layer' is an autorelease object
    auto layer = HelloWorld::create();

    // add layer as a child to scene
    scene->addChild(layer);

    // return the scene
    return scene;
}

// on "init" you need to initialize your instance
bool HelloWorld::init()
{
    //////////////////////////////
    // 1. super init first
    if ( !Layer::init() )
    {
        return false;
    }
    
    
    Vec2 targetPot  = Vec2(1000,600);
    
    Size visibleSize = Director::getInstance()->getVisibleSize();
    Vec2 origin = Director::getInstance()->getVisibleOrigin();

    auto target = Sprite::create("CloseSelected.png");
    target->setPosition(targetPot);
    target->setRotation(100);
    addChild(target);
    
    
    target->runAction(RepeatForever::create( RotateBy::create(0.01, 1)));

    auto g_Data = GameData::Instance();
    
    int maxEntityNum = 10000;
    auto cellSpece = new  CellSpacePartition<Vehicle*>(visibleSize.width,visibleSize.height,visibleSize.width/40,visibleSize.height/40,maxEntityNum);
    
    g_Data->setCellSpace(cellSpece);
    
    //******************SEEK*****************
//    auto s_vehicle = Vehicle::create("CloseNormal.png");
//    s_vehicle->setPosition(100,100);
//    s_vehicle->setVeloctity(Vec2(10,10));
//    s_vehicle->setTarget(targetPot);
//    addChild(s_vehicle);
//    s_vehicle->getSteering()->SeekOn();
//    s_vehicle->scheduleUpdate();
//    
    //******************FLEE*****************
//    auto s_vehicle = Vehicle::create("CloseNormal.png");
//    s_vehicle->setPosition(300,300);
//    s_vehicle->setVeloctity(Vec2(10,10));
//    s_vehicle->setTarget(targetPot);
//    addChild(s_vehicle);
//    s_vehicle->getSteering()->FleeOn();
//    s_vehicle->scheduleUpdate();
//    
//
    //******************Arrive*****************
//
//    auto s_vehicle = Vehicle::create("CloseNormal.png");
//    s_vehicle->setPosition(100,100);
//    s_vehicle->setVeloctity(Vec2(10,0));
//    s_vehicle->setTarget(targetPot);
//    addChild(s_vehicle);
//    s_vehicle->getSteering()->ArriveOn();
//    s_vehicle->scheduleUpdate();
//    
//    
    //******************Pursuit*****************

//   
//    auto evader = Vehicle::create("CloseNormal.png");
//    evader->setHeading(Vec2(0,-1));
//    evader->setPosition(100,100);
//    evader->setVeloctity(Vec2(10,0));
//    evader->setTarget(targetPot);
//    addChild(evader);
//    evader->getSteering()->ArriveOn();
//    evader->scheduleUpdate();
//    
//    auto s_vehicle = Vehicle::create("CloseNormal.png");
//    s_vehicle->setPosition(600,100);
//    s_vehicle->setHeading(Vec2(0,-1));
//    s_vehicle->setVeloctity(Vec2(10,0));
//    s_vehicle->setEvaderv(evader);
//    addChild(s_vehicle);
//    s_vehicle->getSteering()->PursuitOn();
//    s_vehicle->scheduleUpdate();
//    
//

    //******************Evade*****************
    
//    //逃避者
//    auto evader = Vehicle::create("CloseNormal.png");
//    //追逐者
//    auto pursuer = Vehicle::create("CloseNormal.png");
//
//    evader->setHeading(Vec2(0,-1));
//    evader->setPosition(400,300);
//    evader->setVeloctity(Vec2(10,0));
////    evader->setTarget(targetPot);
//    addChild(evader);
//    evader->setPursuer(pursuer);
//    evader->getSteering()->EvadeOn();
//    evader->scheduleUpdate();
//    
//    pursuer->setPosition(100,100);
//    pursuer->setHeading(Vec2(0,-1));
//    pursuer->setVeloctity(Vec2(10,0));
//    pursuer->setEvaderv(evader);
//    addChild(pursuer);
//    pursuer->getSteering()->PursuitOn();
//    pursuer->scheduleUpdate();
//    
    
    //******************Wander*****************

//    auto wander = Vehicle::create("tran.png");
//    wander->setPosition(200,200);
//    wander->setHeading(Vec2(1,0));
//    wander->setVeloctity(Vec2(10,0));
//    addChild(wander);
//    wander->getSteering()->WanderOn();
//    wander->scheduleUpdate();
//
    
    
    //******************ObstaceleAvoidance*****************
//
//    auto s_vehicle1 = Vehicle::create("CloseNormal.png");
//    s_vehicle1->setPosition(1300,400);
//    s_vehicle1->setHeading(Vec2(0,-1));
//    s_vehicle1->setVeloctity(Vec2(10,0));
////    s_vehicle1->setScale(5);
//    addChild(s_vehicle1);
//    
//    auto s_vehicle2 = Vehicle::create("CloseNormal.png");
//    s_vehicle2->setPosition(900,300);
//    s_vehicle2->setHeading(Vec2(0,-1));
//    s_vehicle2->setVeloctity(Vec2(10,0));
////    s_vehicle2->setScale(5, 5);
//    addChild(s_vehicle2);
//    
//    auto s_vehicle3 = Vehicle::create("CloseNormal.png");
//    s_vehicle3->setPosition(200,200);
//    s_vehicle3->setHeading(Vec2(0,-1));
//    s_vehicle3->setVeloctity(Vec2(10,0));
////    s_vehicle3->setScale(5);
//    addChild(s_vehicle3);
//
//    
//    auto s_vehicle4 = Vehicle::create("CloseNormal.png");
//    s_vehicle4->setPosition(500,350);
//    s_vehicle4->setHeading(Vec2(0,-1));
//    s_vehicle4->setVeloctity(Vec2(10,0));
////    s_vehicle4->setScale(5);
//    addChild(s_vehicle4);
// 
//
//    g_Data->addObstacle(s_vehicle1);
//    g_Data->addObstacle(s_vehicle2);
//    g_Data->addObstacle(s_vehicle3);
//    g_Data->addObstacle(s_vehicle4);
//    
////
//    for (int i =0; i<10; i++) {
//        auto obstacle = Vehicle::create("CloseNormal.png");
//        obstacle->setPosition(i*100+200,i*50+CCRANDOM_0_1()*100);
//        obstacle->setHeading(Vec2(0,-1));
//        obstacle->setVeloctity(Vec2(10,0));
//        addChild(obstacle);
//        g_Data->addObstacle(obstacle);
//
//    }
//    
//    auto entity = Vehicle::create("tran.png");
//    entity->setPosition(400,400);
//    entity->setHeading(Vec2(0,-1));
//    entity->setVeloctity(Vec2(10,0));
//    entity->setTarget(Vec2(1000,300));
//    entity->getSteering()->WanderOn();
//    entity->getSteering()->ObstacleAvoidanceOn();
//    entity->scheduleUpdate();
//    addChild(entity);

//******************WallAvoidahce*****************
//
//    auto entity = Vehicle::create("tran.png");
//    entity->setPosition(0,0);
//    entity->setHeading(Vec2(0,-1));
//    entity->setVeloctity(Vec2(10,0));
//    entity->setTarget(targetPot);
//    entity->getSteering()->SeekOn();
//    entity->getSteering()->WallAvoidanceOn();
//    entity->scheduleUpdate();
//    addChild(entity);
//
//
//    Vec2 from = Vec2(500,0);
//    Vec2 to = Vec2(600,500);
//
//    auto color = Color4F::GREEN;
//    auto drawLayer = DrawNode::create();
//    drawLayer->drawLine(from,to,color);
//    addChild(drawLayer);
//    
//    auto wall = new Wall2D();
//    wall->From = from;
//    wall->To = to;
//    wall->Normal = (to-from).getPerp().getNormalized();
//    GameData::Instance()->addWall(*wall);
//    
//
    
    
//******************Interpose*****************

    
//    
//    auto entity = Vehicle::create("CloseNormal.png");
//    entity->setPosition(200,200);
//    entity->setHeading(Vec2(0,-1));
//    entity->setVeloctity(Vec2(10,0));
//    entity->setTarget(targetPot);
//    entity->getSteering()->WanderOn();
//    entity->getSteering()->ObstacleAvoidanceOn();
//    entity->scheduleUpdate();
//    addChild(entity);
//    
//    auto entity1 = Vehicle::create("CloseNormal.png");
//    entity1->setPosition(400,200);
//    entity1->setHeading(Vec2(0,-1));
//    entity1->setVeloctity(Vec2(10,0));
//    entity1->setTarget(targetPot);
//    entity1->getSteering()->WanderOn();
//    entity1->getSteering()->ObstacleAvoidanceOn();
//    entity1->scheduleUpdate();
//    addChild(entity1);
//    
//    
//    
//    auto interpose = Vehicle::create("tran.png");
//    interpose->setPosition(1000,200);
//    interpose->setHeading(Vec2(0,-1));
//    interpose->setVeloctity(Vec2(10,0));
//    interpose->setTarget(targetPot);
//    interpose->setInterposeA(entity);
//    interpose->setInterposeB(entity1);
//    interpose->getSteering()->InterposeOn();
//    interpose->getSteering()->ObstacleAvoidanceOn();
//    interpose->scheduleUpdate();
//    addChild(interpose);
//    
//    g_Data->addEntity(entity1);
//    g_Data->addEntity(entity);
//    g_Data->addEntity(interpose);
//    
//
    
    
//******************Hide*****************
//    
//    auto s_vehicle1 = Vehicle::create("circle1.png");
//    s_vehicle1->setPosition(500,400);
//    s_vehicle1->setHeading(Vec2(0,-1));
//    s_vehicle1->setVeloctity(Vec2(10,0));
//    addChild(s_vehicle1);
//    
//    auto s_vehicle2 = Vehicle::create("circle2.png");
//    s_vehicle2->setPosition(900,300);
//    s_vehicle2->setHeading(Vec2(0,-1));
//    s_vehicle2->setVeloctity(Vec2(10,0));
//    addChild(s_vehicle2);
//    
//    auto s_vehicle3 = Vehicle::create("circle3.png");
//    s_vehicle3->setPosition(200,200);
//    s_vehicle3->setHeading(Vec2(0,-1));
//    s_vehicle3->setVeloctity(Vec2(10,0));
//    addChild(s_vehicle3);
//
//    
//    auto s_vehicle4 = Vehicle::create("circle1.png");
//    s_vehicle4->setPosition(500,400);
//    s_vehicle4->setHeading(Vec2(0,-1));
//    s_vehicle4->setVeloctity(Vec2(10,0));
//    addChild(s_vehicle4);
//    
//    
//    g_Data->addEntity(s_vehicle1);
//    g_Data->addEntity(s_vehicle2);
//    g_Data->addEntity(s_vehicle3);
//    g_Data->addEntity(s_vehicle4);
//    
//    
//    
//    auto wander = Vehicle::create("tran.png");
//    wander->setPosition(800,400);
//    wander->setHeading(Vec2(0,-1));
//    wander->setVeloctity(Vec2(10,0));
//    wander->getSteering()->WanderOn();
//    wander->getSteering()->ObstacleAvoidanceOn();
//    wander->scheduleUpdate();
//    addChild(wander);
//    
//
//    auto interpose = Vehicle::create("tran.png");
//    interpose->setPosition(500,200);
//    interpose->setHeading(Vec2(0,-1));
//    interpose->setVeloctity(Vec2(10,0));
//    interpose->setHideTarget(wander);
//    interpose->getSteering()->ObstacleAvoidanceOn();
//    interpose->getSteering()->HideOn();
//    interpose->scheduleUpdate();
//    addChild(interpose);
//    
//    g_Data->addEntity(wander);
//    g_Data->addEntity(interpose);
//    
//    
//******************PathFollowing*****************

//    auto path = new Path();
//    auto point1 = Vec2(100,100);
//    auto point2 = Vec2(200,100);
//    auto point3 = Vec2(290,30);
//    auto point4 = Vec2(400,200);
//    auto point5 = Vec2(100,400);
//    auto point6 = Vec2(800,400);
//    auto point7 = Vec2(300,190);
//    auto point8 = Vec2(100,290);
//    auto point9 = Vec2(10,100);
//    auto point10 = Vec2(500,300);
//   
//    std::vector<Vec2> n_path;
//    n_path.push_back(point1);
//    n_path.push_back(point2);
//    n_path.push_back(point3);
//    n_path.push_back(point4);
//    n_path.push_back(point5);
//    n_path.push_back(point6);
//    n_path.push_back(point7);
//    n_path.push_back(point8);
//    n_path.push_back(point9);
//    n_path.push_back(point10);
//    path->Set(n_path);
//    
//
//    auto followPath = Vehicle::create("tran.png");
//    followPath->setPosition(100,100);
//    followPath->setHeading(Vec2(0,-1));
//    followPath->setVeloctity(Vec2(10,0));
//    followPath->getSteering()->PathFollowingOn();
//    followPath->setPath(path);
//    followPath->scheduleUpdate();
//    addChild(followPath);
    
//******************OffsetPursuit*****************
//    
//    auto leader = Vehicle::create("tran.png");
//    leader->setPosition(500,200);
//    leader->setHeading(Vec2(0,-1));
//    leader->setVeloctity(Vec2(10,0));
//    leader->setTarget(targetPot);
//    leader->getSteering()->WanderOn();
//    leader->scheduleUpdate();
//    addChild(leader);
//    
//    
//    auto offset1 =Vec2(-50,-30);
//    auto offset2 =Vec2(-50,30);
//    auto offset3 =Vec2(-100,0);
//    auto offset4 =Vec2(-100,60);
//    auto offset5 =Vec2(-100,-60);
//    
//    
//    auto follower1 = Vehicle::create("tran.png");
//    follower1->setPosition(0,600);
//    follower1->setHeading(Vec2(0,-1));
//    follower1->setVeloctity(Vec2(10,0));
//    follower1->setOffsetToLeader(offset1);
//    follower1->setLeader(leader);
//    follower1->getSteering()->OffsetPursuitOn();
//    follower1->scheduleUpdate();
//    addChild(follower1);
//    
//    
//    auto follower2 = Vehicle::create("tran.png");
//    follower2->setPosition(0,600);
//    follower2->setHeading(Vec2(0,-1));
//    follower2->setVeloctity(Vec2(10,0));
//    follower2->setOffsetToLeader(offset2);
//    follower2->setLeader(leader);
//    follower2->getSteering()->OffsetPursuitOn();
//    follower2->scheduleUpdate();
//    addChild(follower2);
//    
//    auto follower3 = Vehicle::create("tran.png");
//    follower3->setPosition(0,600);
//    follower3->setHeading(Vec2(0,-1));
//    follower3->setVeloctity(Vec2(10,0));
//    follower3->setOffsetToLeader(offset3);
//    follower3->setLeader(leader);
//    follower3->getSteering()->OffsetPursuitOn();
//    follower3->scheduleUpdate();
//    addChild(follower3);
//    
//    auto follower4 = Vehicle::create("tran.png");
//    follower4->setPosition(0,600);
//    follower4->setHeading(Vec2(0,-1));
//    follower4->setVeloctity(Vec2(10,0));
//    follower4->setOffsetToLeader(offset4);
//    follower4->setLeader(leader);
//    follower4->getSteering()->OffsetPursuitOn();
//    follower4->scheduleUpdate();
//    addChild(follower4);
//    
//    auto follower5 = Vehicle::create("tran.png");
//    follower5->setPosition(0,600);
//    follower5->setHeading(Vec2(0,-1));
//    follower5->setVeloctity(Vec2(10,0));
//    follower5->setOffsetToLeader(offset5);
//    follower5->setLeader(leader);
//    follower5->getSteering()->OffsetPursuitOn();
//    follower5->scheduleUpdate();
//    addChild(follower5);
//    
//    
    
//******************Flocking*****************

    
//    for (int i =0; i<7; i++) {
//        auto wander = Vehicle::create("tran.png");
//        wander->setPosition(400+rand_0_1()*i*100,400+rand_0_1()*i*100);
//        wander->setHeading(Vec2(0,-1));
//        wander->setVeloctity(Vec2(10,0));
//        wander->setTarget(targetPot);
//        wander->getSteering()->WanderOn();
////        wander->getSteering()->ArriveOn();
//        wander->getSteering()->AlignmentOn();
//        wander->getSteering()->SeparationOn();
//        wander->getSteering()->CohesionOn();
//        wander->getSteering()->ObstacleAvoidanceOn();
//        wander->scheduleUpdate();
//        addChild(wander);
//        g_Data->addEntity(wander);
//        
//    }
//
    
//******************CellSpace*****************

    for (int i =0; i<7; i++) {
        auto wander = Vehicle::create("tran.png");
        wander->setPosition(400+rand_0_1()*i*100,400+rand_0_1()*i*100);
        wander->setHeading(Vec2(0,-1));
        wander->setVeloctity(Vec2(10,0));
        wander->setTarget(targetPot);
        wander->getSteering()->WanderOn();
//        wander->getSteering()->AlignmentOn();
//        wander->getSteering()->SeparationOn();
        wander->scheduleUpdate();
        addChild(wander);
        g_Data->addEntity(wander);
        cellSpece->AddEntity(wander);
    }
    
    return true;
}



