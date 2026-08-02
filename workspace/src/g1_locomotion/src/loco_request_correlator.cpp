/**
 * @file loco_request_correlator.cpp
 * @brief Async request/response correlator for the LocoClient wire contract.
 */
#include "g1_locomotion/loco_request_correlator.hpp"

#include <utility>
#include <vector>

#include "g1_locomotion/loco_api_ids.hpp"

namespace g1_locomotion
{

LocoRequestCorrelator::LocoRequestCorrelator(const Config& config)
  : config_(config)
  , next_id_(std::chrono::steady_clock::now().time_since_epoch().count())
{}

std::optional<unitree_api::msg::Request> LocoRequestCorrelator::send(
    std::int64_t api_id, const std::string& parameter, std::chrono::steady_clock::time_point now,
    ResponseCallback on_done)
{
    if (pending_.size() >= config_.max_pending)
    {
        return std::nullopt;
    }

    const std::int64_t id = next_id_++;
    const auto deadline   = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                    std::chrono::duration<double>(config_.request_timeout_s));
    pending_.emplace(id, PendingEntry{ std::move(on_done), deadline });

    unitree_api::msg::Request request;
    request.header.identity.id     = id;
    request.header.identity.api_id = api_id;
    request.parameter              = parameter;
    return request;
}

void LocoRequestCorrelator::onResponse(const unitree_api::msg::Response& msg)
{
    const auto it = pending_.find(msg.header.identity.id);
    if (it == pending_.end())
    {
        ++dropped_response_count_;
        return;
    }
    const auto callback = std::move(it->second.on_done);
    pending_.erase(it);
    callback(msg.header.status.code, msg.data);
}

void LocoRequestCorrelator::sweep(std::chrono::steady_clock::time_point now)
{
    // Two-phase: collect expired callbacks first, then invoke. A callback
    // calling send() during iteration could rehash the map and invalidate iterators.
    std::vector<ResponseCallback> expired;
    for (auto it = pending_.begin(); it != pending_.end();)
    {
        if (now < it->second.deadline)
        {
            ++it;
            continue;
        }
        expired.push_back(std::move(it->second.on_done));
        it = pending_.erase(it);
    }
    for (const auto& callback : expired)
    {
        callback(kCodeTaskTimeout, "");
    }
}

void LocoRequestCorrelator::supersede(std::int64_t id) { pending_.erase(id); }

}  // namespace g1_locomotion
