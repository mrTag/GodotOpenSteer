#ifndef OPENSTEERSERVER_H
#define OPENSTEERSERVER_H

#include "godot_cpp/classes/object.hpp"
#include "godot_cpp/classes/thread.hpp"
#include "godot_cpp/templates/rid_owner.hpp"

#include "OpenSteer/Obstacle.h"
#include "OpenSteer/Proximity.h"
#include "OpenSteer/SimpleVehicle.h"


class OpenSteerServer : public godot::Object {
        GDCLASS(OpenSteerServer, Object)
protected:
    static void _bind_methods();
private:
    typedef OpenSteer::AbstractTokenForProximityDatabase<OpenSteer::AbstractVehicle*> ProximityToken;
    typedef OpenSteer::LQProximityDatabase<OpenSteer::AbstractVehicle*> ProximityDatabase;
    static OpenSteerServer *singleton;

    class ParametricVehicle : public OpenSteer::SimpleVehicle
    {
    public:
        ~ParametricVehicle() override
        {
            delete proximityToken;
        }
        ParametricVehicle() = default;

        ProximityToken* proximityToken = nullptr;
        bool AvoidObstacles{};
        float WanderWeight{};
        float FleeWeight{};
        OpenSteer::Vec3 FleeTarget;
        float SeekWeight{};
        OpenSteer::Vec3 SeekTarget;
        float ArriveWeight{};
        OpenSteer::Vec3 ArriveTarget;

        float SeparationRadius = 5;
        float SeparationWeight{};
        float AlignmentRadius = 7.5f;
        float AlignmentWeight{};
        float CohesionRadius = 9;
        float CohesionWeight{};


        // we'll update the vehicles in a central update method. but we have to override
        // the update method here, because SimpleVehicle is otherwise abstract...
        void update( const float currentTime, const float elapsedTime ) override {}
    };

    godot::RID_Owner<OpenSteer::RectangleObstacle> _rectangleObstacles;
    godot::RID_Owner<OpenSteer::SphereObstacle> _sphereObstacles;
    godot::RID_Owner<OpenSteer::BoxObstacle> _boxObstacles;
    godot::RID_Owner<ParametricVehicle> _parametricVehicles;

    ProximityDatabase* _proximityDatabase;
    OpenSteer::ObstacleGroup _allObstaclesGroup;
    OpenSteer::AVGroup _allVehiclesGroup;
public:
    OpenSteerServer();
    static OpenSteerServer *get_singleton();

    void update(float delta);

    godot::RID createParametricVehicle()
    {
        godot::RID rid = _parametricVehicles.make_rid();
        auto vehicle = _parametricVehicles.get_or_null( rid );
        vehicle->proximityToken = _proximityDatabase->allocateToken( vehicle );
        _allVehiclesGroup.push_back(vehicle);
        return rid;
    }
    void destroyParametricVehicle( const godot::RID &rid )
    {
        auto vehicle = _parametricVehicles.get_or_null( rid );
        if (vehicle == nullptr)
            return;
        auto vehicleInAllVehiclesIter = std::find(_allVehiclesGroup.begin(), _allVehiclesGroup.end(), vehicle);
        if (vehicleInAllVehiclesIter != _allVehiclesGroup.end())
            _allVehiclesGroup.erase(vehicleInAllVehiclesIter);
        _parametricVehicles.free( rid );
    }
    godot::RID createRectangleObstacle()
    {
        godot::RID rid = _rectangleObstacles.make_rid();
        auto obstacle = _rectangleObstacles.get_or_null(rid);
        _allObstaclesGroup.push_back(obstacle);
        return rid;
    }
    void destroyRectangleObstacle( const godot::RID &rid )
    {
        auto obstacle = _rectangleObstacles.get_or_null(rid);
        if (obstacle == nullptr)
            return;
        auto obstacleIter = std::find(_allObstaclesGroup.begin(), _allObstaclesGroup.end(), obstacle);
        if (obstacleIter != _allObstaclesGroup.end())
            _allObstaclesGroup.erase(obstacleIter);
        _rectangleObstacles.free( rid );
    }
    godot::RID createSphereObstacle()
    {
        godot::RID rid = _sphereObstacles.make_rid();
        auto obstacle = _sphereObstacles.get_or_null(rid);
        _allObstaclesGroup.push_back(obstacle);
        return rid;
    }
    void destroySphereObstacle( const godot::RID &rid )
    {
        auto obstacle = _sphereObstacles.get_or_null(rid);
        if (obstacle == nullptr)
            return;
        auto obstacleIter = std::find(_allObstaclesGroup.begin(), _allObstaclesGroup.end(), obstacle);
        if (obstacleIter != _allObstaclesGroup.end())
            _allObstaclesGroup.erase(obstacleIter);
        _sphereObstacles.free( rid );
    }
    godot::RID createBoxObstacle()
    {
        godot::RID rid = _boxObstacles.make_rid();
        auto obstacle = _boxObstacles.get_or_null(rid);
        _allObstaclesGroup.push_back(obstacle);
        return rid;
    }
    void destroyBoxObstacle( const godot::RID &rid )
    {
        auto obstacle = _boxObstacles.get_or_null(rid);
        if (obstacle == nullptr)
            return;
        auto obstacleIter = std::find(_allObstaclesGroup.begin(), _allObstaclesGroup.end(), obstacle);
        if (obstacleIter != _allObstaclesGroup.end())
            _allObstaclesGroup.erase(obstacleIter);
        _boxObstacles.free( rid );
    }

    void setParametricVehicleMaxSpeed( const godot::RID &rid, float maxSpeed )
    {
        if ( auto vehicle = _parametricVehicles.get_or_null( rid ) )
            vehicle->setMaxSpeed(maxSpeed);
    }
    void setParametricVehicleMaxForce( const godot::RID &rid, float maxForce )
    {
        if ( auto vehicle = _parametricVehicles.get_or_null( rid ) )
            vehicle->setMaxForce(maxForce);
    }
    void setParametricVehiclePosition( const godot::RID &vehicleRID, const godot::Vector3 &position )
    {
        if ( auto vehicle = _parametricVehicles.get_or_null( vehicleRID ) )
            vehicle->setPosition({position.x, position.y, position.z});
    }
    void setParametricVehicleForward( const godot::RID &vehicleRID, const godot::Vector3 &forwardDir )
    {
        if ( auto vehicle = _parametricVehicles.get_or_null( vehicleRID ) )
            vehicle->setForward( {forwardDir.x, forwardDir.y, forwardDir.z} );
    }
    void setParametricVehicleUp( const godot::RID &vehicleRID, const godot::Vector3 &upDir )
    {
        if ( auto vehicle = _parametricVehicles.get_or_null( vehicleRID ) )
            vehicle->setUp( {upDir.x, upDir.y, upDir.z} );
    }
    void setParametricVehicleSide( const godot::RID &vehicleRID, const godot::Vector3 &sideDir )
    {
        if ( auto vehicle = _parametricVehicles.get_or_null( vehicleRID ) )
            vehicle->setSide( {sideDir.x, sideDir.y, sideDir.z} );
    }
    void setParametricVehicleSpeed( const godot::RID &vehicleRID, float speed )
    {
        if ( auto vehicle = _parametricVehicles.get_or_null( vehicleRID ) )
            vehicle->setSpeed(speed);
    }
    void setParametricVehicleRadius( const godot::RID &vehicleRID, float radius )
    {
        if ( auto vehicle = _parametricVehicles.get_or_null( vehicleRID ) )
            vehicle->setRadius(radius);
    }
    void setParametricVehicleAvoidObstacles( const godot::RID &vehicleRID, bool avoidObstacles )
    {
        if ( auto vehicle = _parametricVehicles.get_or_null( vehicleRID ) )
            vehicle->AvoidObstacles = avoidObstacles;
    }
    void setParametricVehicleWanderWeight( const godot::RID &vehicleRID, float wanderWeight )
    {
        if ( auto vehicle = _parametricVehicles.get_or_null( vehicleRID ) )
            vehicle->WanderWeight = wanderWeight;
    }
    void setParametricVehicleFleeWeight( const godot::RID &vehicleRID, float fleeWeight )
    {
        if ( auto vehicle = _parametricVehicles.get_or_null( vehicleRID ) )
            vehicle->FleeWeight = fleeWeight;
    }
    void setParametricVehicleSeekWeight( const godot::RID &vehicleRID, float seekWeight )
    {
        if ( auto vehicle = _parametricVehicles.get_or_null( vehicleRID ) )
            vehicle->SeekWeight = seekWeight;
    }
    void setParametricVehicleArriveWeight( const godot::RID& vehicleRID, float arriveWeight )
    {
        if ( auto vehicle = _parametricVehicles.get_or_null( vehicleRID ) )
            vehicle->ArriveWeight = arriveWeight;
    }
    void setParametricVehicleArriveTarget( const godot::RID &vehicleRID, const godot::Vector3& arriveTarget )
    {
        if ( auto vehicle = _parametricVehicles.get_or_null( vehicleRID ) )
            vehicle->ArriveTarget = {arriveTarget.x, arriveTarget.y, arriveTarget.z};
    }
    void setParametricVehicleSeekTarget( const godot::RID &vehicleRID, const godot::Vector3& seekTarget )
    {
        if ( auto vehicle = _parametricVehicles.get_or_null( vehicleRID ) )
            vehicle->SeekTarget = {seekTarget.x, seekTarget.y, seekTarget.z};
    }
    void setParametricVehicleSeparationWeight( const godot::RID& vehicleRID, float separationWeight )
    {
        if ( auto vehicle = _parametricVehicles.get_or_null( vehicleRID ) )
            vehicle->SeparationWeight = separationWeight;
    }
    void setParametricVehicleSeparationRadius( const godot::RID& vehicleRID, float separationRadius )
    {
        if ( auto vehicle = _parametricVehicles.get_or_null( vehicleRID ) )
            vehicle->SeparationRadius = separationRadius;
    }
    void setParametricVehicleCohesionWeight( const godot::RID& vehicleRID, float cohesionWeight )
    {
        if ( auto vehicle = _parametricVehicles.get_or_null( vehicleRID ) )
            vehicle->CohesionWeight = cohesionWeight;
    }
    void setParametricVehicleCohesionRadius( const godot::RID& vehicleRID, float cohesionRadius )
    {
        if ( auto vehicle = _parametricVehicles.get_or_null( vehicleRID ) )
            vehicle->CohesionRadius = cohesionRadius;
    }
    void setParametricVehicleAlignmentWeight( const godot::RID& vehicleRID, float alignmentWeight )
    {
        if ( auto vehicle = _parametricVehicles.get_or_null( vehicleRID ) )
            vehicle->AlignmentWeight = alignmentWeight;
    }
    void setParametricVehicleAlignmentRadius( const godot::RID& vehicleRID, float alignmentRadius )
    {
        if ( auto vehicle = _parametricVehicles.get_or_null( vehicleRID ) )
            vehicle->AlignmentRadius = alignmentRadius;
    }
    godot::Vector3 getParametricVehiclePosition( const godot::RID &vehicleRID )
    {
        if ( auto vehicle = _parametricVehicles.get_or_null( vehicleRID ) )
        {
            auto pos = vehicle->position();
            return {pos.x, pos.y, pos.z};
        }
        return {};
    }
    godot::Vector3 getParametricVehicleForward( const godot::RID &vehicleRID )
    {
        if ( auto vehicle = _parametricVehicles.get_or_null( vehicleRID ) )
        {
            auto forward = vehicle->forward();
            return {forward.x, forward.y, forward.z};
        }
        return {};
    }
    godot::Vector3 getParametricVehicleSide( const godot::RID &vehicleRID )
    {
        if ( auto vehicle = _parametricVehicles.get_or_null( vehicleRID ) )
        {
            auto side = vehicle->side();
            return {side.x, side.y, side.z};
        }
        return {};
    }
    godot::Vector3 getParametricVehicleUp( const godot::RID &vehicleRID )
    {
        if ( auto vehicle = _parametricVehicles.get_or_null( vehicleRID ) )
        {
            auto up = vehicle->up();
            return {up.x, up.y, up.z};
        }
        return {};
    }

    void setRectangleObstacleDimensions( const godot::RID &rectangleObstacleRID, godot::Vector2 dimensions )
    {
        if ( auto rectangleObstacle = _rectangleObstacles.get_or_null( rectangleObstacleRID ) )
        {
            rectangleObstacle->width = dimensions.x;
            rectangleObstacle->height = dimensions.y;
        }
    }
    void setRectangleObstaclePosition( const godot::RID &rectangleObstacleRID, godot::Vector3 position )
    {
        if ( auto rectangleObstacle = _rectangleObstacles.get_or_null( rectangleObstacleRID ) )
            rectangleObstacle->setPosition(position.x, position.y, position.z);
    }
    void setRectangleObstacleForward( const godot::RID &rectangleObstacleRID, godot::Vector3 forwardDir )
    {
        if ( auto rectangleObstacle = _rectangleObstacles.get_or_null( rectangleObstacleRID ) )
            rectangleObstacle->setForward(forwardDir.x, forwardDir.y, forwardDir.z);
    }
    void setRectangleObstacleSide( const godot::RID &rectangleObstacleRID, godot::Vector3 sideDir )
    {
        if ( auto rectangleObstacle = _rectangleObstacles.get_or_null( rectangleObstacleRID ) )
            rectangleObstacle->setSide(sideDir.x, sideDir.y, sideDir.z);
    }
    void setRectangleObstacleUp( const godot::RID &rectangleObstacleRID, godot::Vector3 upDir )
    {
        if ( auto rectangleObstacle = _rectangleObstacles.get_or_null( rectangleObstacleRID ) )
            rectangleObstacle->setUp(upDir.x, upDir.y, upDir.z);
    }

    void setSphereObstaclePosition( const godot::RID &sphereObstacleRID, godot::Vector3 position )
    {
        if ( auto sphereObstacle = _sphereObstacles.get_or_null( sphereObstacleRID ))
            sphereObstacle->center = { position.x, position.y, position.z };
    }
    void setSphereObstacleRadius( const godot::RID &sphereObstacleRID, float radius )
    {
        if ( auto sphereObstacle = _sphereObstacles.get_or_null( sphereObstacleRID ))
            sphereObstacle->radius = radius;
    }

    void setBoxObstacleDimensions( const godot::RID &boxObstacleRID, godot::Vector3 dimensions )
    {
        if ( auto boxObstacle = _boxObstacles.get_or_null( boxObstacleRID ) )
        {
            boxObstacle->width = dimensions.x;
            boxObstacle->height = dimensions.y;
            boxObstacle->depth = dimensions.z;
        }
    }
    void setBoxObstaclePosition( const godot::RID &boxObstacleRID, godot::Vector3 position )
    {
        if ( auto boxObstacle = _boxObstacles.get_or_null( boxObstacleRID ) )
            boxObstacle->setPosition(position.x, position.y, position.z);
    }
    void setBoxObstacleForward( const godot::RID &boxObstacleRID, godot::Vector3 forwardDir )
    {
        if ( auto boxObstacle = _boxObstacles.get_or_null( boxObstacleRID ) )
            boxObstacle->setForward(forwardDir.x, forwardDir.y, forwardDir.z);
    }
    void setBoxObstacleSide( const godot::RID &boxObstacleRID, godot::Vector3 sideDir )
    {
        if ( auto boxObstacle = _boxObstacles.get_or_null( boxObstacleRID ) )
            boxObstacle->setSide(sideDir.x, sideDir.y, sideDir.z);
    }
    void setBoxObstacleUp( const godot::RID &boxObstacleRID, godot::Vector3 upDir )
    {
        if ( auto boxObstacle = _boxObstacles.get_or_null( boxObstacleRID ) )
            boxObstacle->setUp(upDir.x, upDir.y, upDir.z);
    }
};



#endif //OPENSTEERSERVER_H
