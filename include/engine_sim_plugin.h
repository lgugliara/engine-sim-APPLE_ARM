#ifndef ATG_ENGINE_SIM_PLUGIN_H
#define ATG_ENGINE_SIM_PLUGIN_H

#include <stdint.h>
#include <stdbool.h>

// Export macro per cross-platform compatibility
#ifdef _WIN32
    #ifdef ENGINE_SIM_EXPORT
        #define ENGINE_SIM_API __declspec(dllexport)
    #else
        #define ENGINE_SIM_API __declspec(dllimport)
    #endif
#else
    #define ENGINE_SIM_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// HANDLE TYPES
// ============================================================================

typedef void* EngineSimHandle;

// ============================================================================
// CONTROL INPUT STRUCTURE
// ============================================================================

typedef struct {
    // Throttle controls
    float throttle;              // 0.0 to 1.0 (sostituisce Q,W,E,R)
    
    // Engine controls
    bool starterMotor;           // true = starter on (S key)
    bool ignition;               // true = ignition on/off toggle (A key)
    
    // Transmission controls
    int gearShift;               // -1 = down, 0 = no change, 1 = up (Up/Down arrows)
    float clutchPressure;        // 0.0 = depressed, 1.0 = engaged (Shift/T/U)
    
    // Dyno controls
    bool dynoToggle;             // true = toggle dyno (D key)
    bool dynoHoldToggle;         // true = toggle hold (H key)
    float dynoSpeed;             // Target dyno speed in RPM (G + wheel)
    
    // Audio mixer controls
    float volume;                // 0.0 to 1.0 (Z + wheel)
    float convolution;           // 0.0 to 1.0 (X + wheel)
    float highFreqGain;          // dF_F_mix (C + wheel)
    float lowFreqNoise;          // Air noise (V + wheel)
    float highFreqNoise;         // Input sample noise (B + wheel)
    
    // Simulation controls
    float simulationFrequency;   // Hz (N + wheel)
    float simulationSpeed;       // Speed multiplier (1,2,3,4,5 keys)
    
    // View controls
    int viewLayer;               // -1 = decrease, 0 = no change, 1 = increase (M/,)
    int screenMode;              // 0,1,2 for different screen layouts (Tab)
    
} EngineSimControlInput;

// ============================================================================
// ENGINE STATE OUTPUT STRUCTURE
// ============================================================================

typedef struct {
    // Engine metrics
    float rpm;                   // Current RPM
    float speed;                 // Engine speed (rad/s)
    float manifoldPressure;      // kPa
    float intakeFlowRate;        // kg/s
    float intakeAfr;             // Air-fuel ratio
    float exhaustO2;             // O2 percentage
    
    // Power metrics
    float torque;                // Nm
    float power;                 // kW
    float horsePower;            // HP
    
    // Vehicle metrics
    float vehicleSpeed;          // m/s
    int currentGear;             // Current gear (-1 = neutral)
    float clutchPosition;        // 0.0 to 1.0
    
    // Engine state
    bool isRunning;
    bool ignitionEnabled;
    bool starterActive;
    bool dynoEnabled;
    bool dynoHold;
    
    // Fuel consumption
    float fuelMassConsumed;      // kg
    float fuelVolumeConsumed;    // liters
    
    // Temperature
    float displacement;          // liters
    
} EngineSimState;

// ============================================================================
// AUDIO OUTPUT STRUCTURE
// ============================================================================

typedef struct {
    int16_t* samples;            // Audio samples buffer
    int sampleCount;             // Number of samples
    int sampleRate;              // Sample rate (44100 Hz)
    int channels;                // Number of channels (1 = mono)
} EngineSimAudioOutput;

// ============================================================================
// INITIALIZATION & LIFECYCLE
// ============================================================================

/**
 * Inizializza l'engine simulator
 * @param configPath Percorso al file di configurazione del motore (.mr)
 * @return Handle all'istanza, NULL se errore
 */
ENGINE_SIM_API EngineSimHandle EngineSimCreate(const char* configPath);

/**
 * Distrugge l'istanza dell'engine simulator
 * @param handle Handle all'istanza
 */
ENGINE_SIM_API void EngineSimDestroy(EngineSimHandle handle);

/**
 * Inizializza il sistema audio
 * @param handle Handle all'istanza
 * @param sampleRate Sample rate (es. 44100)
 * @return true se successo
 */
ENGINE_SIM_API bool EngineSimInitializeAudio(EngineSimHandle handle, int sampleRate);

// ============================================================================
// SIMULATION UPDATE
// ============================================================================

/**
 * Aggiorna la simulazione per un frame
 * @param handle Handle all'istanza
 * @param deltaTime Tempo trascorso in secondi
 * @param input Struttura con i controlli input
 */
ENGINE_SIM_API void EngineSimUpdate(
    EngineSimHandle handle, 
    float deltaTime, 
    const EngineSimControlInput* input
);

/**
 * Ottiene lo stato corrente del motore
 * @param handle Handle all'istanza
 * @param outState Struttura dove scrivere lo stato
 * @return true se successo
 */
ENGINE_SIM_API bool EngineSimGetState(
    EngineSimHandle handle, 
    EngineSimState* outState
);

// ============================================================================
// AUDIO OUTPUT
// ============================================================================

/**
 * Legge i campioni audio generati dalla simulazione
 * @param handle Handle all'istanza
 * @param buffer Buffer dove scrivere i samples (allocato dal chiamante)
 * @param maxSamples Numero massimo di samples da leggere
 * @return Numero di samples effettivamente letti
 */
ENGINE_SIM_API int EngineSimReadAudio(
    EngineSimHandle handle, 
    int16_t* buffer, 
    int maxSamples
);

/**
 * Ottiene la latenza audio corrente
 * @param handle Handle all'istanza
 * @return Latency in samples
 */
ENGINE_SIM_API int EngineSimGetAudioLatency(EngineSimHandle handle);

// ============================================================================
// CONTROL UTILITIES
// ============================================================================

/**
 * Resetta tutti i controlli a valori di default
 * @param input Struttura da resettare
 */
ENGINE_SIM_API void EngineSimResetControls(EngineSimControlInput* input);

/**
 * Ottiene la versione della libreria
 * @return Stringa con la versione
 */
ENGINE_SIM_API const char* EngineSimGetVersion(void);

/**
 * Ottiene l'ultimo messaggio di errore
 * @return Stringa con l'errore, o NULL
 */
ENGINE_SIM_API const char* EngineSimGetLastError(void);

// ============================================================================
// ADVANCED CONTROLS
// ============================================================================

/**
 * Carica un nuovo motore runtime
 * @param handle Handle all'istanza
 * @param configPath Percorso al file .mr
 * @return true se successo
 */
ENGINE_SIM_API bool EngineSimLoadEngine(
    EngineSimHandle handle, 
    const char* configPath
);

/**
 * Imposta parametri audio avanzati
 * @param handle Handle all'istanza
 * @param volume Volume (0.0-1.0)
 * @param convolution Convolution level (0.0-1.0)
 * @param highFreqGain High frequency gain (0.0-1.0)
 */
ENGINE_SIM_API void EngineSimSetAudioParameters(
    EngineSimHandle handle,
    float volume,
    float convolution,
    float highFreqGain
);

#ifdef __cplusplus
}
#endif

#endif /* ATG_ENGINE_SIM_PLUGIN_H */
