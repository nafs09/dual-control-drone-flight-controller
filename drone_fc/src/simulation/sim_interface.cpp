#include "sim_interface.hpp"

namespace FC { namespace Simulation {

SimInterface::Mode SimInterface::mode = SimInterface::Mode::HARDWARE;
SimInterface::MotorOutputSink SimInterface::outputSink_ = nullptr;
void* SimInterface::outputContext_ = nullptr;

void SimInterface::setMotorOutputSink(MotorOutputSink sink, void* context) {
    outputSink_ = sink;
    outputContext_ = context;
}

}} // namespace FC::Simulation
