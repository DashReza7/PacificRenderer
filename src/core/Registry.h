// Core module for handling plugins in a registry design pattern

#pragma once

#include "core/BSDF.h"
#include <functional>
#include <unordered_map>
#include <string>

class Integrator;
class Scene;
class Emitter;
class RFilter;


class BSDFRegistry {
public:
    // Function signature for BSDF creators
    using BSDFCreator = std::function<BSDF*(const std::unordered_map<std::string, std::string>&)>;
    
    // Register a BSDF type with its creator function
    static void registerBSDF(const std::string& type, BSDFCreator creator);
    
    // Create a BSDF by type name
    static BSDF* createBSDF(const std::string& type, const std::unordered_map<std::string, std::string>& properties);
    
    // List all registered types
    static std::vector<std::string> getRegisteredTypes();

private:
    static std::unordered_map<std::string, BSDFCreator>& getCreators() {
        static std::unordered_map<std::string, BSDFCreator> creators;
        return creators;
    }
};

class IntegratorRegistry {
public:
    // Function signature for integrator creators
    using IntegratorCreator = std::function<Integrator*(const std::unordered_map<std::string, std::string>&)>;
    
    // Register an integrator type with its creator function
    static void registerIntegrator(const std::string& type, IntegratorCreator creator);

    // Create an integrator by type name
    static Integrator* createIntegrator(const std::string& type, const std::unordered_map<std::string, std::string>& properties);

    // List all registered types
    static std::vector<std::string> getRegisteredTypes();

private:
    static std::unordered_map<std::string, IntegratorCreator>& getCreators() {
        static std::unordered_map<std::string, IntegratorCreator> creators;
        return creators;
    }
};

class EmitterRegistry {
public:
    // Function signature for Emitter creators
    using EmitterCreator = std::function<Emitter*(const std::unordered_map<std::string, std::string>&)>;
    
    // Register a Emitter type with its creator function
    static void registerEmitter(const std::string& type, EmitterCreator creator);
    
    // Create a Emitter by type name
    static Emitter* createEmitter(const std::string& type, const std::unordered_map<std::string, std::string>& properties);
    
    // List all registered types
    static std::vector<std::string> getRegisteredTypes();

private:
    static std::unordered_map<std::string, EmitterCreator>& getCreators() {
        static std::unordered_map<std::string, EmitterCreator> creators;
        return creators;
    }
};

class RFilterRegistry {
public:
    // Function signature for RFilter creators
    using RFilterCreator = std::function<RFilter*(const std::unordered_map<std::string, std::string>&)>;
    
    // Register a RFilter type with its creator function
    static void registerRFilter(const std::string& type, RFilterCreator creator);

    // Create a RFilter by type name
    static RFilter* createRFilter(const std::string& type, const std::unordered_map<std::string, std::string>& properties);

    // List all registered types
    static std::vector<std::string> getRegisteredTypes();

private:
    static std::unordered_map<std::string, RFilterCreator>& getCreators() {
        static std::unordered_map<std::string, RFilterCreator> creators;
        return creators;
    }
};

