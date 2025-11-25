#include "../include/engine_sim_plugin.h"
#include "../include/simulator.h"
#include "../include/engine.h"
#include "../include/transmission.h"
#include "../include/vehicle.h"
#include "../include/synthesizer.h"
#include "../include/audio_buffer.h"
#include "../include/units.h"
#include "../scripting/include/compiler.h"

#include <string>
#include <cstring>
#include <memory>

// ============================================================================
// INTERNAL WRAPPER CLASS
// ============================================================================

class EngineSimWrapper {
public:
    Simulator simulator;
    AudioBuffer audioBuffer;
    Engine* engine = nullptr;
    Vehicle* vehicle = nullptr;
    Transmission* transmission = nullptr;
    std::string lastError;
    bool initialized = false;
    bool audioThreadStarted = false;
    
    // Stato precedente per gestire i toggle
    bool prevDynoToggle = false;
    bool prevDynoHoldToggle = false;
    bool prevIgnitionToggle = false;
    int prevGearShift = 0;
    
    EngineSimWrapper() = default;
    ~EngineSimWrapper() {
        if (initialized) {
            if (audioThreadStarted) {
                simulator.endAudioRenderingThread();
                audioThreadStarted = false;
            }
            simulator.destroy();
            audioBuffer.destroy();
            initialized = false;
        }
    }
};

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static EngineSimWrapper* GetWrapper(EngineSimHandle handle) {
    return static_cast<EngineSimWrapper*>(handle);
}

static std::string g_lastError;

static void SetError(const std::string& error) {
    g_lastError = error;
}

// ============================================================================
// API IMPLEMENTATION
// ============================================================================

ENGINE_SIM_API EngineSimHandle EngineSimCreate(const char* configPath) {
    try {
        auto* wrapper = new EngineSimWrapper();
        SetError("");
        
        // Initialize audio buffer for headless operation
        wrapper->audioBuffer.initialize(44100, 44100);
        wrapper->audioBuffer.m_writePointer = (int)(44100 * 0.1);
        
        // Load engine from script if provided
        if (configPath != nullptr && configPath[0] != '\0') {
            if (!EngineSimLoadEngine(wrapper, configPath)) {
                delete wrapper;
                return nullptr;
            }
        }
        
        wrapper->initialized = true;
        return static_cast<EngineSimHandle>(wrapper);
    }
    catch (const std::exception& e) {
        SetError(std::string("Failed to create engine sim: ") + e.what());
        return nullptr;
    }
}

ENGINE_SIM_API void EngineSimDestroy(EngineSimHandle handle) {
    if (!handle) return;
    
    try {
        auto* wrapper = GetWrapper(handle);
        delete wrapper;  // Destructor handles cleanup
    }
    catch (const std::exception& e) {
        SetError(std::string("Failed to destroy engine sim: ") + e.what());
    }
}

ENGINE_SIM_API bool EngineSimInitializeAudio(EngineSimHandle handle, int sampleRate) {
    if (!handle) {
        SetError("Invalid handle");
        return false;
    }
    
    try {
        // Audio thread started after loadSimulation in EngineSimLoadEngine
        return true;
    }
    catch (const std::exception& e) {
        SetError(std::string("Failed to initialize audio: ") + e.what());
        return false;
    }
}

ENGINE_SIM_API void EngineSimUpdate(
    EngineSimHandle handle, 
    float deltaTime, 
    const EngineSimControlInput* input)
{
    
    if (!handle || !input) return;
    
    try {
        
        auto* wrapper = GetWrapper(handle);
        if (!wrapper->initialized) return;
        
        
        Simulator* sim = &wrapper->simulator;
        if (!sim) return;
        
        
        Engine* engine = sim->getEngine();
        if (!engine) return;
        
        
        Transmission* trans = sim->getTransmission();
        
        
        // ---- THROTTLE ----
        engine->setSpeedControl(input->throttle);
        
        
        
        // ---- STARTER MOTOR ----
        sim->m_starterMotor.m_enabled = input->starterMotor;
        
        
        
        // ---- IGNITION (toggle) ----
        IgnitionModule* ignitionModule = engine->getIgnitionModule();
        if (ignitionModule != nullptr) {
            bool shouldToggle = input->ignition && !wrapper->prevIgnitionToggle;
            if (shouldToggle) {
                ignitionModule->m_enabled = !ignitionModule->m_enabled;
            }
        }
        wrapper->prevIgnitionToggle = input->ignition;
        
        
        // ---- TRANSMISSION ----
        if (trans) {
            
            // Gear shift
            if (input->gearShift > 0 && wrapper->prevGearShift <= 0) {
                trans->changeGear(trans->getGear() + 1);
            }
            else if (input->gearShift < 0 && wrapper->prevGearShift >= 0) {
                trans->changeGear(trans->getGear() - 1);
            }
            wrapper->prevGearShift = input->gearShift;
            
            
            // Clutch
            trans->setClutchPressure(input->clutchPressure);
            
        }
        
        
        // ---- DYNAMOMETER ----
        if (input->dynoToggle && !wrapper->prevDynoToggle) {
            sim->m_dyno.m_enabled = !sim->m_dyno.m_enabled;
        }
        wrapper->prevDynoToggle = input->dynoToggle;
        
        if (input->dynoHoldToggle && !wrapper->prevDynoHoldToggle) {
            sim->m_dyno.m_hold = !sim->m_dyno.m_hold;
        }
        wrapper->prevDynoHoldToggle = input->dynoHoldToggle;
        
        if (input->dynoSpeed > 0.0f && sim->m_dyno.m_hold) {
            sim->m_dyno.m_rotationSpeed = input->dynoSpeed * (2.0 * 3.14159265359 / 60.0); // RPM to rad/s
        }
        
        
        // ---- AUDIO PARAMETERS ----
        // NOTE: Audio parameters should NOT be modified during simulation updates
        // because of race conditions with the audio rendering thread.
        // Use EngineSimSetAudioParameters() instead if you need to change them.
        
        // ---- SIMULATION CONTROLS ----
        if (input->simulationFrequency > 0.0f) {
            sim->setSimulationFrequency(static_cast<int>(input->simulationFrequency));
        }
        
        
        if (input->simulationFrequency > 0.0f) {
            sim->setSimulationFrequency(static_cast<int>(input->simulationFrequency));
        }
        
        if (input->simulationSpeed > 0.0f) {
            sim->setSimulationSpeed(input->simulationSpeed);
        }
        
        // ---- UPDATE SIMULATION ----
        sim->setSimulationSpeed(1.0 / input->simulationSpeed);
        sim->startFrame(deltaTime);
        
        
        while (sim->simulateStep()) {
            // Simula step by step
        }
        
        
        sim->endFrame();
        
        
    }
    catch (const std::exception& e) {
        SetError(std::string("Failed to update: ") + e.what());
    }
}

ENGINE_SIM_API bool EngineSimGetState(
    EngineSimHandle handle, 
    EngineSimState* outState)
{
    if (!handle || !outState) {
        SetError("Invalid handle or output state");
        return false;
    }
    
    try {
        auto* wrapper = GetWrapper(handle);
        if (!wrapper->initialized) return false;
        
        Simulator* sim = &wrapper->simulator;
        if (!sim) return false;
        
        Engine* engine = sim->getEngine();
        if (!engine) return false;
        
        Transmission* trans = sim->getTransmission();
        Vehicle* vehicle = sim->getVehicle();
        
        // Clear structure
        memset(outState, 0, sizeof(EngineSimState));
        
        // Engine metrics
        outState->rpm = static_cast<float>(engine->getRpm());
        outState->speed = static_cast<float>(engine->getSpeed());
        outState->manifoldPressure = static_cast<float>(engine->getManifoldPressure());
        outState->intakeFlowRate = static_cast<float>(engine->getIntakeFlowRate());
        outState->intakeAfr = static_cast<float>(engine->getIntakeAfr());
        outState->exhaustO2 = static_cast<float>(engine->getExhaustO2());
        
        // Power metrics
        outState->torque = static_cast<float>(sim->getFilteredDynoTorque());
        outState->power = static_cast<float>(sim->getDynoPower());
        outState->horsePower = outState->power * 1.34102f; // kW to HP
        
        // Vehicle metrics
        if (trans) {
            outState->currentGear = trans->getGear();
            outState->clutchPosition = 1.0f; // TODO: get actual clutch position
        }
        
        // Engine state
        outState->isRunning = (outState->rpm > 100.0f);
        IgnitionModule* ignitionModule = engine->getIgnitionModule();
        outState->ignitionEnabled = (ignitionModule != nullptr) ? ignitionModule->m_enabled : false;
        outState->starterActive = sim->m_starterMotor.m_enabled;
        outState->dynoEnabled = sim->m_dyno.m_enabled;
        outState->dynoHold = sim->m_dyno.m_hold;
        
        // Fuel consumption
        outState->fuelMassConsumed = static_cast<float>(engine->getTotalFuelMassConsumed());
        outState->fuelVolumeConsumed = static_cast<float>(engine->getTotalVolumeFuelConsumed());
        
        // Temperature & displacement
        outState->displacement = static_cast<float>(engine->getDisplacement());
        
        return true;
    }
    catch (const std::exception& e) {
        SetError(std::string("Failed to get state: ") + e.what());
        return false;
    }
}

ENGINE_SIM_API int EngineSimReadAudio(
    EngineSimHandle handle, 
    int16_t* buffer, 
    int maxSamples)
{
    if (!handle || !buffer || maxSamples <= 0) {
        SetError("Invalid parameters");
        return 0;
    }
    
    try {
        auto* wrapper = GetWrapper(handle);
        if (!wrapper->initialized) return 0;
        
        return wrapper->simulator.readAudioOutput(maxSamples, buffer);
    }
    catch (const std::exception& e) {
        SetError(std::string("Failed to read audio: ") + e.what());
        return 0;
    }
}

ENGINE_SIM_API int EngineSimGetAudioLatency(EngineSimHandle handle) {
    if (!handle) return 0;
    
    try {
        auto* wrapper = GetWrapper(handle);
        if (!wrapper->initialized) return 0;
        
        return static_cast<int>(wrapper->simulator.getSynthesizerInputLatency());
    }
    catch (const std::exception& e) {
        SetError(std::string("Failed to get audio latency: ") + e.what());
        return 0;
    }
}

ENGINE_SIM_API void EngineSimResetControls(EngineSimControlInput* input) {
    if (!input) return;
    
    memset(input, 0, sizeof(EngineSimControlInput));
    input->throttle = 0.0f;
    input->clutchPressure = 1.0f; // Default: engaged
    input->volume = 1.0f;
    input->convolution = 1.0f;
    input->simulationSpeed = 1.0f;
}

ENGINE_SIM_API const char* EngineSimGetVersion(void) {
    return "0.1.11a";  // Match the build version
}

ENGINE_SIM_API const char* EngineSimGetLastError(void) {
    return g_lastError.empty() ? nullptr : g_lastError.c_str();
}

ENGINE_SIM_API bool EngineSimLoadEngine(
    EngineSimHandle handle, 
    const char* configPath)
{
    if (!handle || !configPath) {
        SetError("Invalid parameters");
        return false;
    }
    
    try {
        auto* wrapper = GetWrapper(handle);
        
#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
        es_script::Compiler compiler;
        compiler.initialize();
        const bool compiled = compiler.compile(configPath);
        
        if (compiled) {
            const es_script::Compiler::Output output = compiler.execute();
            
            wrapper->engine = output.engine;
            wrapper->vehicle = output.vehicle;
            wrapper->transmission = output.transmission;
            
            // Clean up old simulation if it exists
            wrapper->simulator.releaseSimulation();
            
            if (output.engine == nullptr) {
                compiler.destroy();
                SetError("Engine compilation produced null engine");
                return false;
            }
            
            // Create default vehicle if not provided
            if (wrapper->vehicle == nullptr) {
                Vehicle::Parameters vehParams;
                vehParams.mass = units::mass(1597, units::kg);
                vehParams.diffRatio = 3.42;
                vehParams.tireRadius = units::distance(10, units::inch);
                vehParams.dragCoefficient = 0.25;
                vehParams.crossSectionArea = units::distance(6.0, units::foot) * units::distance(6.0, units::foot);
                vehParams.rollingResistance = 2000.0;
                wrapper->vehicle = new Vehicle;
                wrapper->vehicle->initialize(vehParams);
            }
            
            // Create default transmission if not provided
            if (wrapper->transmission == nullptr) {
                const double gearRatios[] = { 2.97, 2.07, 1.43, 1.00, 0.84, 0.56 };
                Transmission::Parameters tParams;
                tParams.GearCount = 6;
                tParams.GearRatios = gearRatios;
                tParams.MaxClutchTorque = units::torque(1000.0, units::ft_lb);
                wrapper->transmission = new Transmission;
                wrapper->transmission->initialize(tParams);
            }
            
            // Calculate displacement
            output.engine->calculateDisplacement();
            
            // Initialize simulator
            try {
                wrapper->simulator.setFluidSimulationSteps(8);
                wrapper->simulator.setSimulationFrequency(output.engine->getSimulationFrequency());
                
                Simulator::Parameters simulatorParams;
                simulatorParams.SystemType = Simulator::SystemType::NsvOptimized;
                wrapper->simulator.initialize(simulatorParams);
                wrapper->simulator.loadSimulation(wrapper->engine, wrapper->vehicle, wrapper->transmission);
                
                // CRITICAL: Initialize impulse responses BEFORE starting audio thread
                // to prevent heap-buffer-overflow in ConvolutionFilter.
                // ConvolutionFilter expects valid impulse response data even in headless mode.
                for (int i = 0; i < output.engine->getExhaustSystemCount(); ++i) {
                    constexpr int dummySize = 1024;
                    static int16_t dummyIR[dummySize] = {0};
                    wrapper->simulator.getSynthesizer()->initializeImpulseResponse(
                        dummyIR, dummySize, 0.0, i
                    );
                }
                
                // Disable convolution for headless operation
                Synthesizer::AudioParameters audioParams = wrapper->simulator.getSynthesizer()->getAudioParameters();
                audioParams.Convolution = 0.0;
                wrapper->simulator.getSynthesizer()->setAudioParameters(audioParams);
                
                // Start audio rendering thread after all initialization is complete
                wrapper->simulator.startAudioRenderingThread();
                wrapper->audioThreadStarted = true;
            }
            catch (const std::exception& e) {
                compiler.destroy();
                SetError(std::string("Simulator initialization failed: ") + e.what());
                return false;
            }
            
            compiler.destroy();
            return true;
        }
        else {
            compiler.destroy();
            SetError("Failed to compile engine script");
            return false;
        }
#else
        SetError("Engine scripting not enabled (ATG_ENGINE_SIM_PIRANHA_ENABLED not defined)");
        return false;
#endif
    }
    catch (const std::exception& e) {
        SetError(std::string("Failed to load engine: ") + e.what());
        return false;
    }
}

ENGINE_SIM_API void EngineSimSetAudioParameters(
    EngineSimHandle handle,
    float volume,
    float convolution,
    float highFreqGain)
{
    if (!handle) return;
    
    try {
        auto* wrapper = GetWrapper(handle);
        if (!wrapper->initialized) return;
        
        Synthesizer::AudioParameters audioParams = wrapper->simulator.getSynthesizer()->getAudioParameters();
        audioParams.Volume = volume;
        audioParams.Convolution = convolution;
        audioParams.dF_F_mix = highFreqGain;
        wrapper->simulator.getSynthesizer()->setAudioParameters(audioParams);
    }
    catch (const std::exception& e) {
        SetError(std::string("Failed to set audio parameters: ") + e.what());
    }
}
