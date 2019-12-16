
#ifndef REINFORCEMENT_LEARNING_STEERING_RL_UTIL_H_
#define REINFORCEMENT_LEARNING_STEERING_RL_UTIL_H_

#include <reinforcement_learning_steering/RLSimulation.h>

namespace reinforcement_learning_steering
{
namespace util
{

struct RlEpisodeComperator
{
    bool operator()(const RLSimulation &lhs, const RLSimulation &rhs) const
    {
        return lhs.episode_iteration < rhs.episode_iteration;
    }
};
} // namespace util

} // namespace reinforcement_learning_steering

#endif