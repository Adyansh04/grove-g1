#ifndef G1_LOCOMOTION__LOCO_REQUEST_CORRELATOR_HPP_
#define G1_LOCOMOTION__LOCO_REQUEST_CORRELATOR_HPP_

/**
 * @file loco_request_correlator.hpp
 * @brief Pure, ROS-free async request/response correlator for the LocoClient wire contract.
 */

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>

#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"

namespace g1_locomotion
{

/**
 * @brief Async LocoClient request/response correlator.
 *
 * Bookkeeps pending requests by ID and handles response callbacks and timeouts non-blockingly.
 */
class LocoRequestCorrelator
{
public:
    /**
     * @brief Tunables -- see the package README's param table for defaults/provenance.
     */
    struct Config
    {
        /// Vendor's own BaseClient blocks 5.0 s; this is that same value, non-blocking.
        double request_timeout_s{ 5.0 };
        /// Belt-and-braces bound on the pending map's size (see send()).
        std::size_t max_pending{ 16 };
    };

    /// Invoked exactly once per send() call that reaches a terminal outcome: a matched response
    /// (error_code from header.status.code, data from the response body) or a sweep()-driven
    /// timeout (kCodeTaskTimeout, empty data). Never invoked for a superseded id.
    using ResponseCallback = std::function<void(std::int32_t error_code, const std::string& data)>;

    explicit LocoRequestCorrelator(const Config& config);

    /**
     * @brief Builds a Request with a fresh identity.id, tracks `on_done` against it, and returns
     * it ready to publish. LocoClient never sets `lease`/`policy` -- left at their zero default.
     *
     * Id generation: seeded once from steady_clock ns at construction, then post-incremented per
     * call -- unique even for two sends within the same clock tick, unlike the vendor's
     * BaseClient, which recomputes wall-clock uptime on every single call.
     *
     * @param api_id     Destination LocoClient API id.
     * @param parameter  JSON request payload (see loco_payloads.hpp); empty for a GET.
     * @param now        Current time, used to compute this entry's sweep() deadline.
     * @param on_done    Invoked later from onResponse() or sweep() -- see ResponseCallback.
     * @return The Request to publish, or nullopt if max_pending is already reached -- nothing is
     *   tracked in that case, and the caller sends nothing this call.
     */
    [[nodiscard]] std::optional<unitree_api::msg::Request> send(
        std::int64_t api_id, const std::string& parameter,
        std::chrono::steady_clock::time_point now, ResponseCallback on_done);

    /**
     * @brief Matches `msg.header.identity.id` against the pending map and invokes+erases it.
     *
     * An id with no pending entry -- already swept by a timeout, already superseded, or simply
     * never sent by this process -- is dropped and counted, never treated as an error: a
     * response can legitimately arrive after sweep() already timed its entry out.
     * @param msg  The received /api/sport/response.
     */
    void onResponse(const unitree_api::msg::Response& msg);

    /**
     * @brief Expires every pending entry older than request_timeout_s as of `now`, invoking its
     * callback with (kCodeTaskTimeout, "").
     *
     * Every expired callback is invoked only after every expired entry has already been erased
     * (two-phase, unlike onResponse()) -- so it is safe for a callback to call send() and insert
     * a new entry into this same instance, even though that can rehash the pending map.
     * @param now  Current time.
     */
    void sweep(std::chrono::steady_clock::time_point now);

    /**
     * @brief Drops a pending entry without invoking its callback -- e.g. a fresher command
     * superseding one still in flight, where only the newest outcome matters. A no-op if `id`
     * isn't pending.
     * @param id  Id previously returned via send()'s Request.
     */
    void supersede(std::int64_t id);

    /**
     * @brief Drops every pending entry with no callback invocation -- e.g. the owning node
     * tearing down, where nothing left standing (no sweep timer, no response subscription) could
     * ever service a response or a timeout anyway, so leaving entries pending would just strand
     * their captured callbacks. Same no-callback contract as supersede(), for all entries at once.
     */
    void clear() noexcept { pending_.clear(); }

    /// @return The number of requests currently awaiting a response or sweep() timeout.
    std::size_t pendingCount() const noexcept { return pending_.size(); }

    /// @return How many onResponse() calls matched no pending entry (unknown or already-resolved
    ///   ids) -- exposed so tests and the node's debug logging can observe it directly.
    std::size_t droppedResponseCount() const noexcept { return dropped_response_count_; }

private:
    struct PendingEntry
    {
        ResponseCallback                      on_done;
        std::chrono::steady_clock::time_point deadline;
    };

    Config                                         config_;
    std::int64_t                                   next_id_;
    std::unordered_map<std::int64_t, PendingEntry> pending_;
    std::size_t                                    dropped_response_count_{ 0 };
};

}  // namespace g1_locomotion

#endif  // G1_LOCOMOTION__LOCO_REQUEST_CORRELATOR_HPP_
