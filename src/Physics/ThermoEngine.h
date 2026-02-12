#pragma once
#include <cmath>
#include <string>
#include <unordered_map>

struct EngineSpecs {
    float displacement; // Liters
    float maxHP;        // Horsepower
    float maxRPM;       // RPM limit
    float compression;  // Compression Ratio
    float volEffPeak;   // Volumetric Efficiency (0.0 - 1.0)
};

class ThermoEngine {
public:
    EngineSpecs specs;
    
    // Database of known NFS cars to inject real specs
    static EngineSpecs GetSpecsForCar(const std::string& carID) {
        static const std::unordered_map<std::string, EngineSpecs> db = {
            {"corvette", {5.665f, 345.f, 6000.f, 10.1f, 0.92f}}, // C5 LS1
            {"diablo",   {5.707f, 492.f, 7500.f, 10.0f, 0.94f}}, // Lamborghini V12
            {"911",      {3.600f, 282.f, 6800.f, 11.3f, 0.88f}}, // Porsche 993
            {"ferrari",  {3.495f, 375.f, 8250.f, 11.1f, 0.98f}}, // F355 V8
            {"mercedes", {6.898f, 389.f, 6000.f, 10.0f, 0.85f}}  // CLK GTR (approx)
        };
        
        // Return specs if found, otherwise generic V8 fallback
        if (db.count(carID)) return db.at(carID);
        return {5.0f, 300.f, 6000.f, 9.5f, 0.85f};
    }

    void Init(const std::string& carID) {
        specs = GetSpecsForCar(carID);
    }

    // Returns available Torque (Nm) based on current environment
    float CalculateTorque(float currentRPM, float throttle, float ambientPressPa, float ambientTempC) {
        if (currentRPM <= 0.f) return 0.f;

        // 1. Air Density (Ideal Gas Law)
        float T_kelvin = ambientTempC + 273.15f;
        float airDensity = ambientPressPa / (287.05f * T_kelvin);

        // 2. Volumetric Efficiency Curve (Simplified Bell Curve around 70% max RPM)
        float peakRPM = specs.maxRPM * 0.7f;
        float sigma = specs.maxRPM * 0.25f;
        float ve = specs.volEffPeak * exp(-0.5f * pow((currentRPM - peakRPM) / sigma, 2));

        // 3. Mass Air Flow (kg/s)
        // (RPM/60) * (Displacement/2) * Density * VE
        float displacementM3 = specs.displacement / 1000.0f;
        float m_air = (currentRPM / 60.0f) * (displacementM3 / 2.0f) * airDensity * ve;

        // 4. Energy Release
        // Gasoline LHV = 44 MJ/kg, AFR = 12.5:1 (Power), Thermal Eff ~ 30%
        float m_fuel = m_air / 12.5f;
        float powerWatts = m_fuel * 44000000.0f * 0.30f * throttle;

        // 5. Torque = Power / Omega
        float omega = (currentRPM * 2.0f * 3.14159f) / 60.0f;
        return (omega > 1.0f) ? (powerWatts / omega) : 0.0f;
    }
};
