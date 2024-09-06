#include "OpenSteerServer.h"

#include "OpenSteer/Draw.h"

// seems like we need to define a few symbols here, even when we don't use them...
bool OpenSteer::enableAnnotation = false;
bool OpenSteer::drawPhaseActive = false;
OpenSteer::Color::Color( float rValue, float gValue, float bValue, float aValue ){}
void OpenSteer::drawLine( const Vec3 &startPoint, const Vec3 &endPoint, const Color &color ){}
void OpenSteer::deferredDrawLine( const Vec3 &startPoint, const Vec3 &endPoint, const Color &color ){}




OpenSteerServer * OpenSteerServer::singleton = nullptr;

void OpenSteerServer::_bind_methods()
{
    godot::ClassDB::bind_static_method("OpenSteerServer", godot::D_METHOD("get_singleton"),
            &OpenSteerServer::get_singleton);
    godot::ClassDB::bind_method(godot::D_METHOD("createParametricVehicle"), &OpenSteerServer::createParametricVehicle);
    godot::ClassDB::bind_method(godot::D_METHOD("createRectangleObstacle"), &OpenSteerServer::createRectangleObstacle);
    godot::ClassDB::bind_method(godot::D_METHOD("createSphereObstacle"), &OpenSteerServer::createSphereObstacle);
    godot::ClassDB::bind_method(godot::D_METHOD("createBoxObstacle"), &OpenSteerServer::createBoxObstacle);
    godot::ClassDB::bind_method(godot::D_METHOD("destroyParametricVehicle", "rid"), &OpenSteerServer::destroyParametricVehicle);
    godot::ClassDB::bind_method(godot::D_METHOD("destroyRectangleObstacle", "rid"), &OpenSteerServer::destroyRectangleObstacle);
    godot::ClassDB::bind_method(godot::D_METHOD("destroySphereObstacle", "rid"), &OpenSteerServer::destroySphereObstacle);
    godot::ClassDB::bind_method(godot::D_METHOD("destroyBoxObstacle", "rid"), &OpenSteerServer::destroyBoxObstacle);
    godot::ClassDB::bind_method(godot::D_METHOD("setParametricVehicleMaxSpeed", "rid", "maxSpeed"), &OpenSteerServer::setParametricVehicleMaxSpeed);
    godot::ClassDB::bind_method(godot::D_METHOD("setParametricVehicleMaxForce", "rid", "maxForce"), &OpenSteerServer::setParametricVehicleMaxForce);
    godot::ClassDB::bind_method(godot::D_METHOD("setParametricVehiclePosition", "vehicleRID", "position"), &OpenSteerServer::setParametricVehiclePosition);
    godot::ClassDB::bind_method(godot::D_METHOD("setParametricVehicleForward", "vehicleRID", "forwardDir"), &OpenSteerServer::setParametricVehicleForward);
    godot::ClassDB::bind_method(godot::D_METHOD("setParametricVehicleUp", "vehicleRID", "upDir"), &OpenSteerServer::setParametricVehicleUp);
    godot::ClassDB::bind_method(godot::D_METHOD("setParametricVehicleSide", "vehicleRID", "sideDir"), &OpenSteerServer::setParametricVehicleSide);
    godot::ClassDB::bind_method(godot::D_METHOD("setParametricVehicleSpeed", "vehicleRID", "speed"), &OpenSteerServer::setParametricVehicleSpeed);
    godot::ClassDB::bind_method(godot::D_METHOD("setParametricVehicleRadius", "vehicleRID", "radius"), &OpenSteerServer::setParametricVehicleRadius);
    godot::ClassDB::bind_method(godot::D_METHOD("setParametricVehicleAvoidObstacles", "vehicleRID", "avoidObstacles"), &OpenSteerServer::setParametricVehicleAvoidObstacles);
    godot::ClassDB::bind_method(godot::D_METHOD("setParametricVehicleWanderWeight", "vehicleRID", "wanderWeight"), &OpenSteerServer::setParametricVehicleWanderWeight);
    godot::ClassDB::bind_method(godot::D_METHOD("setParametricVehicleFleeWeight", "vehicleRID", "fleeWeight"), &OpenSteerServer::setParametricVehicleFleeWeight);
    godot::ClassDB::bind_method(godot::D_METHOD("setParametricVehicleSeekWeight", "vehicleRID", "seekWeight"), &OpenSteerServer::setParametricVehicleSeekWeight);
    godot::ClassDB::bind_method(godot::D_METHOD("setParametricVehicleArriveWeight", "vehicleRID", "arriveWeight"), &OpenSteerServer::setParametricVehicleArriveWeight);
    godot::ClassDB::bind_method(godot::D_METHOD("setParametricVehicleArriveTarget", "vehicleRID", "arriveTarget"), &OpenSteerServer::setParametricVehicleArriveTarget);
    godot::ClassDB::bind_method(godot::D_METHOD("setParametricVehicleSeekTarget", "vehicleRID", "seekTarget"), &OpenSteerServer::setParametricVehicleSeekTarget);
    godot::ClassDB::bind_method(godot::D_METHOD("setParametricVehicleSeparationWeight", "vehicleRID", "separationWeight"), &OpenSteerServer::setParametricVehicleSeparationWeight);
    godot::ClassDB::bind_method(godot::D_METHOD("setParametricVehicleSeparationRadius", "vehicleRID", "separationRadius"), &OpenSteerServer::setParametricVehicleSeparationRadius);
    godot::ClassDB::bind_method(godot::D_METHOD("setParametricVehicleCohesionWeight", "vehicleRID", "cohesionWeight"), &OpenSteerServer::setParametricVehicleCohesionWeight);
    godot::ClassDB::bind_method(godot::D_METHOD("setParametricVehicleCohesionRadius", "vehicleRID", "cohesionRadius"), &OpenSteerServer::setParametricVehicleCohesionRadius);
    godot::ClassDB::bind_method(godot::D_METHOD("setParametricVehicleAlignmentWeight", "vehicleRID", "alignmentWeight"), &OpenSteerServer::setParametricVehicleAlignmentWeight);
    godot::ClassDB::bind_method(godot::D_METHOD("setParametricVehicleAlignmentRadius", "vehicleRID", "alignmentRadius"), &OpenSteerServer::setParametricVehicleAlignmentRadius);
    godot::ClassDB::bind_method(godot::D_METHOD("getParametricVehiclePosition", "vehicleRID"), &OpenSteerServer::getParametricVehiclePosition);
    godot::ClassDB::bind_method(godot::D_METHOD("getParametricVehicleForward", "vehicleRID"), &OpenSteerServer::getParametricVehicleForward);
    godot::ClassDB::bind_method(godot::D_METHOD("getParametricVehicleSide", "vehicleRID"), &OpenSteerServer::getParametricVehicleSide);
    godot::ClassDB::bind_method(godot::D_METHOD("getParametricVehicleUp", "vehicleRID"), &OpenSteerServer::getParametricVehicleUp);

    godot::ClassDB::bind_method(godot::D_METHOD("setRectangleObstacleDimensions", "rectangleObstacleRID", "dimensions"), &OpenSteerServer::setRectangleObstacleDimensions);
    godot::ClassDB::bind_method(godot::D_METHOD("setRectangleObstaclePosition", "rectangleObstacleRID", "position"), &OpenSteerServer::setRectangleObstaclePosition);
    godot::ClassDB::bind_method(godot::D_METHOD("setRectangleObstacleForward", "rectangleObstacleRID", "forwardDir"), &OpenSteerServer::setRectangleObstacleForward);
    godot::ClassDB::bind_method(godot::D_METHOD("setRectangleObstacleSide", "rectangleObstacleRID", "sideDir"), &OpenSteerServer::setRectangleObstacleSide);
    godot::ClassDB::bind_method(godot::D_METHOD("setRectangleObstacleUp", "rectangleObstacleRID", "upDir"), &OpenSteerServer::setRectangleObstacleUp);

    godot::ClassDB::bind_method(godot::D_METHOD("setSphereObstaclePosition", "sphereObstacleRID", "position"), &OpenSteerServer::setSphereObstaclePosition);
    godot::ClassDB::bind_method(godot::D_METHOD("setSphereObstacleRadius", "sphereObstacleRID", "radius"), &OpenSteerServer::setSphereObstacleRadius);

    godot::ClassDB::bind_method(godot::D_METHOD("setBoxObstacleDimensions", "boxObstacleRID", "dimensions"), &OpenSteerServer::setBoxObstacleDimensions);
    godot::ClassDB::bind_method(godot::D_METHOD("setBoxObstaclePosition", "boxObstacleRID", "position"), &OpenSteerServer::setBoxObstaclePosition);
    godot::ClassDB::bind_method(godot::D_METHOD("setBoxObstacleForward", "boxObstacleRID", "forwardDir"), &OpenSteerServer::setBoxObstacleForward);
    godot::ClassDB::bind_method(godot::D_METHOD("setBoxObstacleSide", "boxObstacleRID", "sideDir"), &OpenSteerServer::setBoxObstacleSide);
    godot::ClassDB::bind_method(godot::D_METHOD("setBoxObstacleUp", "boxObstacleRID", "upDir"), &OpenSteerServer::setBoxObstacleUp);

    godot::ClassDB::bind_method(godot::D_METHOD("update", "delta"), &OpenSteerServer::update);
}

OpenSteerServer::OpenSteerServer()
{
    singleton = this;
    _proximityDatabase = new ProximityDatabase(
        OpenSteer::Vec3::zero,
        OpenSteer::Vec3(200,200,200),
        OpenSteer::Vec3(20, 20, 20));
}

OpenSteerServer *OpenSteerServer::get_singleton()
{
    return singleton;
}

void OpenSteerServer::update( float delta )
{
    static OpenSteer::AVGroup neighbors;
    const float separationAngle  = -0.707f;
    const float alignmentAngle  = 0.7f;
    const float cohesionAngle  = -0.15f;
    
    static godot::List<godot::RID> ownedVehicles;
    ownedVehicles.clear();
    _parametricVehicles.get_owned_list( &ownedVehicles );
    for(const auto & ownedVehicleRID : ownedVehicles)
    {
        auto vehicle = _parametricVehicles.get_or_null( ownedVehicleRID );
        if(vehicle->AvoidObstacles)
        {
            OpenSteer::Vec3 obstacleAvoidance = vehicle->steerToAvoidObstacles( 6, _allObstaclesGroup );
            if(obstacleAvoidance != OpenSteer::Vec3::zero)
            {
                vehicle->applySteeringForce( obstacleAvoidance, delta );
                continue;
            }
        }
        OpenSteer::Vec3 vehicleSteeringForce;
        if(vehicle->WanderWeight > 0)
            vehicleSteeringForce += vehicle->WanderWeight * vehicle->steerForWander( delta );
        if(vehicle->SeekWeight > 0)
            vehicleSteeringForce += vehicle->SeekWeight * vehicle->steerForSeek( vehicle->SeekTarget );
        if(vehicle->FleeWeight > 0)
            vehicleSteeringForce += vehicle->FleeWeight * vehicle->steerForFlee( vehicle->FleeTarget );

        if(vehicle->AlignmentWeight > 0 || vehicle->SeparationWeight > 0 || vehicle->CohesionWeight > 0)
        {
            // flocking is active, we need our neighbors!
            float max_radius = std::max( vehicle->AlignmentRadius, std::max(vehicle->SeparationRadius, vehicle->CohesionRadius ));
            neighbors.clear();
            vehicle->proximityToken->findNeighbors( vehicle->position(  ),  max_radius, neighbors);
            if(vehicle->AlignmentWeight > 0)
                vehicleSteeringForce += vehicle->AlignmentWeight * vehicle->steerForAlignment(
                    vehicle->AlignmentRadius, alignmentAngle, neighbors);
            if(vehicle->SeparationWeight > 0)
                vehicleSteeringForce += vehicle->SeparationWeight * vehicle->steerForSeparation(
                    vehicle->SeparationRadius, separationAngle, neighbors);
            if(vehicle->CohesionWeight > 0)
                vehicleSteeringForce += vehicle->CohesionWeight * vehicle->steerForCohesion(
                    vehicle->CohesionRadius, cohesionAngle, neighbors);
        }
        
        vehicle->applySteeringForce( vehicleSteeringForce, delta );
        vehicle->proximityToken->updateForNewPosition( vehicle->position(  ) );
    }
}