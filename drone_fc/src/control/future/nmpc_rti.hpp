#pragma once

#include "../../math/quaternion.hpp"
#include "../../math/vector3.hpp"
#include <cstddef>
#include <cstdint>

namespace FC { namespace Control { namespace Future {

using Math::Quaternionf;
using Math::Vector3f;

/* Interface scaffold for a future nonlinear MPC real-time iteration loop.
 * It deliberately has no built-in optimizer: until a bounded, target-tested
 * QP backend is supplied, feedback() returns kBackendUnavailable and zeros
 * the output. It is not connected to the flight-mode or motor path. */
class NmpcRti {
public:
    static constexpr std::size_t kHorizon = 20;

    struct State {
        Vector3f position_ned_m;
        Vector3f velocity_ned_mps;
        Quaternionf body_to_ned;
        Vector3f body_rate_rps;
    };

    struct Reference {
        Vector3f position_ned_m;
        Vector3f velocity_ned_mps;
        Vector3f acceleration_ned_mps2;
        float yaw_rad = 0.0f;
    };

    struct Model {
        float mass_kg = 0.0f;
        Vector3f inertia_kgm2;
        float arm_length_m = 0.0f;
        float thrust_coefficient = 0.0f;
        float yaw_moment_coefficient = 0.0f;
        float minimum_rotor_thrust_n = 0.0f;
        float maximum_rotor_thrust_n = 0.0f;
    };

    struct Input {
        float collective_thrust_n = 0.0f;
        Vector3f body_torque_nm;
    };

    enum class Status : uint8_t {
        kReady,
        kInvalidInput,
        kBackendUnavailable,
        kBackendFailure
    };

    /* Backend split follows RTI's preparation/feedback phases. A target
     * backend owns its fixed-size linearization and QP workspace. */
    struct Backend {
        void* context = nullptr;
        bool (*prepare)(void*, const Model&, const State&,
                        const Reference*, std::size_t, float) = nullptr;
        bool (*feedback)(void*, const State&, Input&) = nullptr;
    };

    NmpcRti() : backend_{} {}
    explicit NmpcRti(const Backend& backend) : backend_(backend) {}

    Status prepare(const Model& model, const State& state,
                   const Reference (&references)[kHorizon + 1], float dt_s) {
        if (!validModel(model) || !validState(state) || !finite(dt_s) ||
            dt_s <= 0.0f || dt_s > 0.2f || !backend_.prepare || !backend_.feedback)
            return (!backend_.prepare || !backend_.feedback)
                ? Status::kBackendUnavailable : Status::kInvalidInput;
        for (std::size_t i = 0; i <= kHorizon; ++i)
            if (!validReference(references[i])) return Status::kInvalidInput;
        return backend_.prepare(backend_.context, model, state, references,
                                kHorizon + 1, dt_s)
            ? Status::kReady : Status::kBackendFailure;
    }

    Status feedback(const State& state, Input& input) const {
        input = Input{};
        if (!validState(state)) return Status::kInvalidInput;
        if (!backend_.feedback) return Status::kBackendUnavailable;
        return backend_.feedback(backend_.context, state, input)
            ? Status::kReady : Status::kBackendFailure;
    }

private:
    Backend backend_;

    static bool finite(float x) {
        return x == x && x <= 3.402823466e+38F && x >= -3.402823466e+38F;
    }
    static bool validVector(const Vector3f& v) {
        return finite(v.x) && finite(v.y) && finite(v.z);
    }
    static bool validState(const State& s) {
        return validVector(s.position_ned_m) && validVector(s.velocity_ned_mps) &&
               validVector(s.body_rate_rps) && finite(s.body_to_ned.w) &&
               finite(s.body_to_ned.x) && finite(s.body_to_ned.y) &&
               finite(s.body_to_ned.z) && s.body_to_ned.normSq() > 1.0e-8f;
    }
    static bool validReference(const Reference& r) {
        return validVector(r.position_ned_m) && validVector(r.velocity_ned_mps) &&
               validVector(r.acceleration_ned_mps2) && finite(r.yaw_rad);
    }
    static bool validModel(const Model& m) {
        return finite(m.mass_kg) && m.mass_kg > 0.0f &&
               validVector(m.inertia_kgm2) && m.inertia_kgm2.x > 0.0f &&
               m.inertia_kgm2.y > 0.0f && m.inertia_kgm2.z > 0.0f &&
               finite(m.arm_length_m) && m.arm_length_m > 0.0f &&
               finite(m.thrust_coefficient) && m.thrust_coefficient > 0.0f &&
               finite(m.yaw_moment_coefficient) && m.yaw_moment_coefficient > 0.0f &&
               finite(m.minimum_rotor_thrust_n) &&
               finite(m.maximum_rotor_thrust_n) &&
               m.minimum_rotor_thrust_n >= 0.0f &&
               m.maximum_rotor_thrust_n > m.minimum_rotor_thrust_n;
    }
};

}}} // namespace FC::Control::Future
