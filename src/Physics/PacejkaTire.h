#pragma once
#include <btBulletDynamicsCommon.h>

struct TireCoeffs {
    float B, C, D, E; // Magic Formula coefficients
};

class PacejkaTire {
public:
    // Standard Road Tire Coefficients
    TireCoeffs latCoeffs = {10.0f, 1.3f, 1.0f, 0.97f}; // Lateral (Cornering)
    TireCoeffs longCoeffs = {10.0f, 1.65f, 1.0f, 0.97f}; // Longitudinal (Accel/Brake)

    float CalculateForce(float slip, float load, bool lateral) {
        TireCoeffs& k = lateral ? latCoeffs : longCoeffs;
        float Fz_scale = load / 4000.0f; // Normalize load (approx)
        
        // Magic Formula
        return (k.D * Fz_scale * 4000.f) * sin(k.C * atan(k.B * slip - k.E * (k.B * slip - atan(k.B * slip))));
    }

    void UpdateWheel(btRaycastVehicle* vehicle, int wheelIdx, float deltaTime) {
        btWheelInfo& wheel = vehicle->getWheelInfo(wheelIdx);
        if (!wheel.m_raycastInfo.m_isInContact) return;

        // Get local velocity at wheel contact point
        btTransform wheelTrans = wheel.m_worldTransform;
        btVector3 wheelVel = vehicle->getRigidBody()->getVelocityInLocalPoint(wheel.m_chassisConnectionPointCS);
        
        // Calculate Slip Angle (Alpha)
        // Lateral velocity / Longitudinal velocity
        float v_long = wheelVel.dot(wheelTrans.getBasis().getColumn(0)); // Forward axis
        float v_lat = wheelVel.dot(wheelTrans.getBasis().getColumn(2));  // Right axis
        float slipAngle = atan2(v_lat, abs(v_long) + 0.1f); // Avoid div/0

        // Calculate Longitudinal Slip (Kappa)
        // (AngularVel * Radius - GroundVel) / max(GroundVel, 1.0)
        float wheelVelAngular = wheel.m_deltaRotation / deltaTime; // Approx angular vel
        float slipRatio = (wheelVelAngular * wheel.m_wheelsRadius - v_long) / (abs(v_long) + 1.0f);

        // Calculate Forces
        float f_lat = CalculateForce(slipAngle, wheel.m_wheelsSuspensionForce, true);
        float f_long = CalculateForce(slipRatio, wheel.m_wheelsSuspensionForce, false);

        // Apply to Chassis (Need to transform to world space)
        btVector3 forceLocal = btVector3(0, 0, f_lat); // Assuming Z is right in local, check Bullet coords
        // Note: You must properly rotate this force vector by the wheel's steering angle + chassis rotation
        
        // Apply Force: In a real implementation, apply this force to the chassis at the contact point
        // vehicle->getRigidBody()->applyForce(worldForce, contactPoint);
        
        // IMPORTANT: Disable Bullet's native friction to avoid double-dipping
        wheel.m_frictionSlip = 0.0f; 
    }
};
