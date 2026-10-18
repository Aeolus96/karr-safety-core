#include "karr/micro_ros_bridge.h"

#include <Arduino.h>
#include <SPI.h> // SPI: shared by the W5500 Ethernet controller

// --- micro-ROS (Jazzy distribution, Ethernet transport) ---
#include <micro_ros_platformio.h>
#include <rcl/init_options.h> // explicit ROS domain via init options
#include <rmw_microros/time_sync.h> // one-shot agent time sync on connect
#include <rmw_microros/timing.h>    // explicit entity create/destroy budgets

#include "karr/rc_receiver.h"

namespace karr
{


namespace
{

// --- Network (static IP; agent must match your LAN) ---
const IPAddress BOARD_STATIC_IP(192, 168, 0, 24);
const IPAddress LOCAL_GATEWAY(192, 168, 0, 1);
const IPAddress LOCAL_SUBNET_MASK(255, 255, 255, 0);
const IPAddress MICRO_ROS_AGENT_IP(192, 168, 0, 4);
constexpr uint16_t MICRO_ROS_AGENT_PORT = 8888;
constexpr char BOARD_HOSTNAME[] = "estop_relay_board";
// ROS domain, applied explicitly at support init through rcl init options
// (rclc_support_init_with_options + rcl_init_options_set_domain_id).
constexpr size_t ROS_DOMAIN_ID = 0;

// --- W5500 Ethernet SPI pins (also set as build flags in platformio.ini,
//     which the transport reads from macros; repeated for SPI.begin()) ---
constexpr uint8_t ETHERNET_SPI_CLOCK_PIN = 15;
constexpr uint8_t ETHERNET_SPI_MISO_PIN = 14;
constexpr uint8_t ETHERNET_SPI_MOSI_PIN = 13;
constexpr uint8_t ETHERNET_CHIP_SELECT_PIN = 16;

// --- Topics (short chains; names carry the meaning) ---
constexpr char RECEIVER_JOY_TOPIC[] = "/rc/joy";          // OUT sensor_msgs/Joy
constexpr char RECEIVER_LINK_TOPIC[] = "/rc/link";        // OUT std_msgs/Bool
constexpr char BOARD_HEARTBEAT_TOPIC[] = "/relay_board/heartbeat"; // OUT UInt32 (s)
constexpr char BOARD_RELAY_STATE_TOPIC[] = "/relay_board/relay_state"; // OUT UInt8MultiArray
constexpr char BOARD_INPUTS_TOPIC[] = "/relay_board/inputs";     // OUT UInt8
constexpr char BOARD_ESTOP_TOPIC[] = "/relay_board/estop";       // OUT Bool (engaged)

// --- Cadences / agent ping budgets ---
constexpr uint32_t JOY_PUBLISH_PERIOD_MS = 20;        // 50 Hz
constexpr uint32_t HEALTH_PUBLISH_PERIOD_MS = 1000;   // 1 Hz
constexpr uint32_t AGENT_PING_PERIOD_CONNECTED_MS = 1000;
constexpr uint32_t AGENT_PING_PERIOD_WAITING_MS = 500;
constexpr uint32_t AGENT_PING_TIMEOUT_MS = 100;       // worst-case block
constexpr uint8_t AGENT_PING_ATTEMPTS = 1;
constexpr uint64_t TIME_SYNC_TIMEOUT_MS = 100; // one-shot sync budget (ms)
// Explicit micro-ROS session budgets (bounded blocking, per design rules).
constexpr int ENTITY_CREATION_SESSION_TIMEOUT_MS = 100; // agent handshake per entity
constexpr int ENTITY_DESTROY_SESSION_TIMEOUT_MS = 200; // teardown handshake
constexpr int RELIABLE_PUBLISHER_SESSION_TIMEOUT_MS = 100; // relay_state ack budget
constexpr uint32_t RELAY_STATE_RETRY_PERIOD_MS = 500;    // failed publish retry gap
constexpr uint32_t CLEANUP_RETRY_PERIOD_MS = 500;        // teardown retry cadence

} // namespace

// ---------------------------------------------------------------------------
// MicroRosBridge implementation
// ---------------------------------------------------------------------------

MicroRosBridge::MicroRosBridge()
{
    allocator_ = rcl_get_default_allocator();
    resetEntityHandles(); // single zero-init source — safe pre-publication:
                          // nothing is live yet, so no flag check is needed
}

void MicroRosBridge::begin()
{
    SPI.begin(ETHERNET_SPI_CLOCK_PIN, ETHERNET_SPI_MISO_PIN,
              ETHERNET_SPI_MOSI_PIN, ETHERNET_CHIP_SELECT_PIN);

    // CRITICAL DECISION: set_microros_ethernet_transports() performs
    // ETH.begin() itself using the ETH_PHY_* build flags from platformio.ini,
    // configures the static IP, and points the XRCE-DDS session at the
    // agent. The SPI bus is pre-initialized with the same pins so the
    // transport and any later ETH status queries share one bus setup.
    set_microros_ethernet_transports(
        BOARD_STATIC_IP, LOCAL_GATEWAY, LOCAL_SUBNET_MASK,
        MICRO_ROS_AGENT_IP, MICRO_ROS_AGENT_PORT, BOARD_HOSTNAME);
    Serial.printf("[ETH] static IP %s, agent %s:%u\n",
                  BOARD_STATIC_IP.toString().c_str(),
                  MICRO_ROS_AGENT_IP.toString().c_str(),
                  MICRO_ROS_AGENT_PORT);

    // Attach static storage to every sequence-bearing message exactly once.
    // From here on, publishing only ever updates `size` — micro-ROS must
    // not reallocate sequence buffers during steady-state operation.
    joy_message_.axes.data = joy_axes_storage_;
    joy_message_.axes.capacity = RC_CHANNEL_COUNT;
    joy_message_.axes.size = 0;
    joy_message_.buttons.data = joy_buttons_storage_;
    joy_message_.buttons.capacity = RC_CHANNEL_COUNT;
    joy_message_.buttons.size = 0;

    relay_state_message_.data.data = relay_state_storage_;
    relay_state_message_.data.capacity = RELAY_COUNT;
    relay_state_message_.data.size = 0;
}

bool MicroRosBridge::isAgentConnected() const
{
    return agent_state_ == AgentState::AgentConnected;
}

const char *MicroRosBridge::agentStateText() const
{
    switch (agent_state_)
    {
    case AgentState::WaitingForAgent:
        return "WAITING";
    case AgentState::AgentAvailable:
        return "AVAILABLE";
    case AgentState::AgentConnected:
        return "CONNECTED";
    case AgentState::CleanupPending:
        return "CLEANUP";
    }
    return "?";
}

bool MicroRosBridge::publishTelemetryMessage(rcl_publisher_t *publisher, const void *message)
{
    // CRITICAL DECISION: publish results are propagated, not discarded.
    // Publication failures are counted. Best-effort telemetry avoids
    // acknowledgement waits. Transport execution time and reliable
    // relay-state publication remain part of the loop timing budget.
    if (rcl_publish(publisher, message, nullptr) != RCL_RET_OK)
    {
        ++publish_failure_count_;
        return false;
    }
    return true;
}
bool MicroRosBridge::initializeEntities()
{
    allocator_ = rcl_get_default_allocator();

    // Domain comes from ROS_DOMAIN_ID through rcl init options (the only
    // domain path: rclc_support_init takes no domain parameter). Same
    // sequence as the micro_ros_platformio ethernet_pubsub example. init
    // options are ALWAYS finalized before return — rclc_support_init copies
    // what it needs, so holding them open across entity creation would leak.
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&init_options, allocator_) != RCL_RET_OK)
    {
        Serial.println("[ROS] init options init failed");
        return false;
    }
    // Domain configuration is explicit. Init options are local resources —
    // report finalization failure without losing track of initialized
    // support: support_ stays valid even if the fini below fails.
    rcl_ret_t support_result = RCL_RET_ERROR;
    if (rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID) != RCL_RET_OK)
    {
        Serial.println("[ROS] domain id config failed");
    }
    else
    {
        support_result = rclc_support_init_with_options(&support_, 0, nullptr, &init_options, &allocator_);
        if (support_result != RCL_RET_OK)
        {
            Serial.println("[ROS] support init failed");
        }
    }
    if (rcl_init_options_fini(&init_options) != RCL_RET_OK)
    {
        // Local options leak only — support, when initialized, is unaffected.
        Serial.println("[ROS] WARNING: init options fini failed (support state kept)");
    }
    if (support_result != RCL_RET_OK)
    {
        return false;
    }
    entities_.support = true;

    // Explicit creation budget while the context is valid: bounds how long
    // each entity-creation handshake can block when the agent is slow.
    // REQUIRED timing configuration is part of successful initialization —
    // do not report the requested budget as active if its setup failed.
    rmw_ret_t creation_timeout_result =
        rmw_uros_set_context_entity_creation_session_timeout(
            rcl_context_get_rmw_context(&support_.context),
            ENTITY_CREATION_SESSION_TIMEOUT_MS);
    if (creation_timeout_result != RMW_RET_OK)
    {
        Serial.printf("[ROS] REQUIRED entity-creation timeout config failed: %d\n",
                      creation_timeout_result);
        return false;
    }

    if (rclc_node_init_default(&node_, "estop_relay_controller", "", &support_) != RCL_RET_OK)
    {
        Serial.println("[ROS] node init failed");
        return false;
    }
    entities_.node = true;

    // Telemetry publishers use BEST-EFFORT QoS: stale data is worthless and
    // a reliable publisher with a full history can block in rcl_publish.
    if (rclc_publisher_init_best_effort(&joy_publisher_, &node_,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
                                        RECEIVER_JOY_TOPIC) != RCL_RET_OK)
    {
        Serial.println("[ROS] joy publisher init failed");
        return false;
    }
    entities_.joy_publisher = true;

    if (rclc_publisher_init_best_effort(&link_publisher_, &node_,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                                        RECEIVER_LINK_TOPIC) != RCL_RET_OK)
    {
        Serial.println("[ROS] link publisher init failed");
        return false;
    }
    entities_.link_publisher = true;

    if (rclc_publisher_init_best_effort(&heartbeat_publisher_, &node_,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
                                        BOARD_HEARTBEAT_TOPIC) != RCL_RET_OK)
    {
        Serial.println("[ROS] heartbeat publisher init failed");
        return false;
    }
    entities_.heartbeat_publisher = true;

    // Relay state uses DEFAULT (reliable) QoS: consumers may care about
    // every transition of a safety-relevant output. The publisher's
    // acknowledgement budget is REQUIRED timing configuration — without it
    // a reliable publish could block unboundedly, so init fails if it
    // cannot be configured.
    if (rclc_publisher_init_default(&relay_state_publisher_, &node_,
                                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
                                    BOARD_RELAY_STATE_TOPIC) != RCL_RET_OK)
    {
        Serial.println("[ROS] relay state publisher init failed");
        return false;
    }
    entities_.relay_state_publisher = true;

    rmw_ret_t publisher_timeout_result = rmw_uros_set_publisher_session_timeout(
        rcl_publisher_get_rmw_handle(&relay_state_publisher_),
        RELIABLE_PUBLISHER_SESSION_TIMEOUT_MS);
    if (publisher_timeout_result != RMW_RET_OK)
    {
        Serial.printf("[ROS] REQUIRED relay-state publisher timeout config failed: %d\n",
                      publisher_timeout_result);
        return false;
    }

    if (rclc_publisher_init_best_effort(&inputs_publisher_, &node_,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
                                        BOARD_INPUTS_TOPIC) != RCL_RET_OK)
    {
        Serial.println("[ROS] inputs publisher init failed");
        return false;
    }
    entities_.inputs_publisher = true;

    if (rclc_publisher_init_best_effort(&estop_publisher_, &node_,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                                        BOARD_ESTOP_TOPIC) != RCL_RET_OK)
    {
        Serial.println("[ROS] estop publisher init failed");
        return false;
    }
    entities_.estop_publisher = true;

    Serial.println("[ROS] entities initialized — node 'estop_relay_controller' up");
    return true;
}

// ---------------------------------------------------------------------------
// resetEntityHandles — explicit re-zeroing of every entity handle. Called
// ONLY when no flag is set (nothing live) or immediately after a terminal
// support-fini abandon (see destroyEntities): overwriting potentially-live
// handles would leak agent resources without crashing, which is NOT
// recovery — the flags decide when this is safe.
// ---------------------------------------------------------------------------
void MicroRosBridge::resetEntityHandles()
{
    // rclc_support_t has no public zero-init accessor; value-initialize it.
    support_ = rclc_support_t{};
    node_ = rcl_get_zero_initialized_node();
    joy_publisher_ = rcl_get_zero_initialized_publisher();
    link_publisher_ = rcl_get_zero_initialized_publisher();
    heartbeat_publisher_ = rcl_get_zero_initialized_publisher();
    relay_state_publisher_ = rcl_get_zero_initialized_publisher();
    inputs_publisher_ = rcl_get_zero_initialized_publisher();
    estop_publisher_ = rcl_get_zero_initialized_publisher();
}

// ---------------------------------------------------------------------------
// finalizePublisher — one child publisher's fini/clear/log step, guarded by
// its flag (no-op when already released). See the destroyEntities failure
// policy below for why the flag is kept on failure.
// ---------------------------------------------------------------------------
void MicroRosBridge::finalizePublisher(rcl_publisher_t &publisher, bool &created_flag, const char *name)
{
    if (!created_flag)
    {
        return;
    }
    rcl_ret_t fini_result = rcl_publisher_fini(&publisher, &node_);
    if (fini_result == RCL_RET_OK)
    {
        created_flag = false;
    }
    else
    {
        Serial.printf("[ROS] %s publisher fini failed: %d\n", name, fini_result);
    }
}
// ---------------------------------------------------------------------------
// destroyEntities — safe teardown, possibly spanning multiple calls.
//
// CRITICAL ORDERING: destroy CHILDREN BEFORE THEIR OWNERS (publishers ->
// node -> support). rcl_publisher_fini()/rcl_node_fini() require their
// parent handles to still be valid, and the context must stay alive until
// all entity cleanup completes. The destroy-timeout is configured FIRST,
// while the context is still valid.
//
// Failure policy (NO attempt counter — stage-specific, per the finalizer
// semantics verified in the installed micro_ros_platformio source):
//  - publisher/node fini: success clears the flag, failure KEEPS it and the
//    parent is preserved; the next call retries. rcl_publisher_fini /
//    rcl_node_fini free and NULL their impl EVEN ON FAILURE, and a repeat
//    call on the NULLed handle returns RCL_RET_OK — so a failed child
//    normally self-heals in exactly one extra cycle (never a wedge; the
//    NULLed impl is re-init-safe, which is also the standard micro-ROS
//    reconnection pattern).
//  - support fini: SINGLE attempt, then terminal. rclc_support_fini (as
//    shipped in this micro_ros_platformio revision) runs its three
//    sub-finalizers unconditionally in one call, and a repeat after a
//    failure returns an error for the already-shut-down context
//    (rcl_shutdown re-entry returns ALREADY_SHUTDOWN/INVALID) — so
//    retrying cannot clear the error. On failure the handles are
//    abandoned immediately: the state machine still proceeds, and the log
//    states plainly that this is NOT a clean teardown (the agent-side
//    session may linger until the agent restarts; any local allocation
//    the partial cleanup did not free stays until reboot — forced
//    abandonment is honesty, not recovery). Repeated abandonments are
//    counted in support_abandonment_count_ and stay visible in the serial
//    log. Abandonment is NOT indefinitely leak-free recovery: no automatic
//    reboot — a board reboot must not become the default answer to a
//    ROS-only failure. If hardware testing ever shows the abandonment
//    count climbing, the planned escalation is SUSPENDING ROS
//    reconnection in a visible degraded state while local input sampling
//    and non-ROS features continue. That decision needs hardware
//    failure-rate data first.
// Returns false while any entity remains live so the caller (the
// CleanupPending state) retries — never blindly overwrite live resources
// and call that recovery.
// ---------------------------------------------------------------------------
bool MicroRosBridge::destroyEntities()
{
    if (!entities_.anyResources())
    {
        // Nothing live (e.g. init failed at the very first step). Ensure a
        // clean slate for the next init attempt — safe because no flag is set.
        resetEntityHandles();
        time_sync_pending_ = false;
        time_synced_ = false;
        return true;
    }

    // Configure the destroy budget FIRST, while the context is still valid.
    if (entities_.support)
    {
        rmw_ret_t timing_result = rmw_uros_set_context_entity_destroy_session_timeout(
            rcl_context_get_rmw_context(&support_.context),
            ENTITY_DESTROY_SESSION_TIMEOUT_MS);
        if (timing_result != RMW_RET_OK)
        {
            // A mis-configured budget undermines the teardown-time contract —
            // say so rather than discarding it as [[maybe_unused]].
            Serial.printf("[ROS] destroy timeout config failed: %d\n", timing_result);
        }
    }

    // Teardown failures are recorded, not silently ignored: each fini that
    // succeeds clears its flag; each failure keeps it (retry next call).
    // Explicit per-publisher calls (NOT a loop/framework): the child-before-
    // parent ordering is a safety boundary and must stay visible here.
    // Each call is a no-op unless its flag is set (guard inside the helper).
    finalizePublisher(joy_publisher_, entities_.joy_publisher, "joy");
    finalizePublisher(link_publisher_, entities_.link_publisher, "link");
    finalizePublisher(heartbeat_publisher_, entities_.heartbeat_publisher, "heartbeat");
    finalizePublisher(relay_state_publisher_, entities_.relay_state_publisher, "relay state");
    finalizePublisher(inputs_publisher_, entities_.inputs_publisher, "inputs");
    finalizePublisher(estop_publisher_, entities_.estop_publisher, "estop");

    // A failed child cleanup prevents owner cleanup: preserve the node
    // until EVERY publisher has been released. Destroying the node while a
    // publisher flag is still set would make the next cleanup call run
    // rcl_publisher_fini() against an already-finalized node.
    if (entities_.anyPublishers())
    {
        Serial.println("[ROS] teardown incomplete — node preserved, retry next cycle");
        return false;
    }

    // Only now attempt node cleanup.
    if (entities_.node)
    {
        rcl_ret_t fini_result = rcl_node_fini(&node_);
        if (fini_result == RCL_RET_OK) entities_.node = false;
        else Serial.printf("[ROS] node fini failed: %d\n", fini_result);
    }

    // Do not finalize support while the node remains.
    if (entities_.node)
    {
        Serial.println("[ROS] teardown incomplete — support preserved, retry next cycle");
        return false;
    }

    if (entities_.support)
    {
        rcl_ret_t fini_result = rclc_support_fini(&support_);
        if (fini_result == RCL_RET_OK)
        {
            entities_.support = false;
        }
        else
        {
            // TERMINAL (see the failure policy above): no retry — for this
            // library revision re-entry cannot clear the error. Abandon the
            // handles for a clean-slate re-init and say exactly what this
            // is NOT: NOT successful cleanup.
            Serial.printf("[ROS] support fini failed (%d) — abandoning handles "
                          "(agent-side entities may linger until the agent restarts; "
                          "any unfreed local allocation stays until reboot)\n",
                          fini_result);
            resetEntityHandles();
            entities_ = EntityFlags{};
            time_sync_pending_ = false;
            time_synced_ = false;
            ++support_abandonment_count_;
            Serial.printf("[ROS] entities abandoned after failed support fini "
                          "(total abandonments: %lu)\n",
                          (unsigned long)support_abandonment_count_);
            return true;
        }
    }

    resetEntityHandles(); // clean slate for the next init
    time_sync_pending_ = false;
    time_synced_ = false;
    Serial.println("[ROS] entities destroyed");
    return true;
}
void MicroRosBridge::service(uint32_t now_ms)
{
    // CRITICAL DECISION (operation ordering):
    //  1. CLEANUP dispatch runs FIRST, on its own schedule, with NO
    //     discovery ping — teardown does not need agent discovery.
    //  2. Detecting agent loss NEVER performs teardown in the same call:
    //     the call returns right after marking CleanupPending so the
    //     feature layer re-evaluates its dependencies FIRST (with
    //     FORCE_SAFE_ON_AGENT_LOSS=true that commands every current
    //     RC-driven feature to 0x00) BEFORE any bounded destroy work
    //     (200 ms budget, several entities) runs.
    //  3. A failed entity creation also defers teardown to a later call
    //     (CleanupPending) — creation failure and destruction never stack
    //     in one invocation.
    //  4. The one-shot time sync runs only on a LATER call than the one
    //     that created the entities, and only while still connected —
    //     never on a destroyed context.
    //
    // Lifecycle map (loop() keeps sampling RC and evaluating features in
    // EVERY state — this state machine only owns the ROS side):
    //
    //   WaitingForAgent --ping ok--> AgentAvailable --created--> AgentConnected
    //         ^                          |                          |
    //         |                          | creation failed          | ping fails:
    //         |                          v                          v  LOSS DETECTED
    //         +---- destroyEntities ---+-> CleanupPending <---------+ ONLY
    //              (500 ms cadence,        (teardown deferred one call:
    //               no discovery ping)      features re-evaluated first)
    if (agent_state_ == AgentState::CleanupPending)
    {
        if (now_ms - last_cleanup_attempt_ms_ >= CLEANUP_RETRY_PERIOD_MS)
        {
            last_cleanup_attempt_ms_ = now_ms;
            if (destroyEntities())
            {
                agent_state_ = AgentState::WaitingForAgent;
                Serial.println("[ROS] waiting for micro-ROS agent...");
                previous_agent_state_ = agent_state_;
            }
            // else: stay CleanupPending and retry on the next schedule tick
        }
        return; // no discovery ping while cleaning up
    }

    uint32_t ping_period = isAgentConnected() ? AGENT_PING_PERIOD_CONNECTED_MS
                                              : AGENT_PING_PERIOD_WAITING_MS;
    if (now_ms - last_ping_ms_ < ping_period)
    {
        return; // not due yet
    }
    last_ping_ms_ = now_ms;

    // Entry state: prevents entity creation and time sync from stacking
    // in the same call (sync must happen on a LATER call than creation).
    const bool connected_at_entry = isAgentConnected();

    bool agent_reachable =
        (rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_ATTEMPTS) == RMW_RET_OK);

    switch (agent_state_)
    {
    case AgentState::WaitingForAgent:
        if (agent_reachable)
        {
            agent_state_ = AgentState::AgentAvailable;
        }
        break;

    case AgentState::AgentAvailable:
        if (agent_reachable && initializeEntities())
        {
            agent_state_ = AgentState::AgentConnected;
        }
        else
        {
            // Creation failed: defer teardown to the cleanup dispatch —
            // never stack failed creation and destruction in one call.
            agent_state_ = AgentState::CleanupPending;
        }
        break;

    case AgentState::AgentConnected:
        if (!agent_reachable)
        {
            // LOSS DETECTED ONLY — teardown is deferred so ROS-dependent
            // features go inactive FIRST (the loop re-evaluates feature
            // permissions immediately after this call returns; with
            // FORCE_SAFE_ON_AGENT_LOSS=true that is every current feature).
            agent_state_ = AgentState::CleanupPending;
        }
        break;

    case AgentState::CleanupPending:
        // Handled by the dispatch above; unreachable here.
        break;
    }

    // Transition handling
    if (agent_state_ != previous_agent_state_)
    {
        switch (agent_state_)
        {
        case AgentState::WaitingForAgent:
            Serial.println("[ROS] waiting for micro-ROS agent...");
            break;
        case AgentState::AgentConnected:
            Serial.println("[ROS] agent connected");
            time_sync_pending_ = true;            // executed on a LATER call
            relay_state_publish_forced_ = true;   // force a fresh snapshot
            break;
        case AgentState::CleanupPending:
            Serial.println("[ROS] AGENT LOST / init failed — teardown pending; ROS-dependent features to 0x00 (FORCE_SAFE_ON_AGENT_LOSS)");
            break;
        default:
            break;
        }
        previous_agent_state_ = agent_state_;
    }

    // One-shot time sync: BOTH entry and current state must be connected.
    // Entry state prevents entity creation and sync from stacking in one
    // call; current state prevents syncing after loss detection/teardown.
    if (connected_at_entry && isAgentConnected() && time_sync_pending_)
    {
        time_sync_pending_ = false;
        if (rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS) == RMW_RET_OK)
        {
            time_synced_ = true;
            Serial.println("[ROS] device clock synced with agent");
        }
        else
        {
            time_synced_ = false;
            Serial.println("[ROS] time sync unavailable (OK — Joy stamps stay zero)");
        }
    }
}

void MicroRosBridge::publishReceiverTelemetry(const ControlSnapshot &snapshot, uint32_t now_ms)
{
    if (!isAgentConnected())
    {
        return;
    }
    if (now_ms - last_joy_publish_ms_ < JOY_PUBLISH_PERIOD_MS)
    {
        return;
    }
    last_joy_publish_ms_ = now_ms;

    // Receiver health from the coherent snapshot: SignalActive means fresh
    // frames and no failsafe signature. While unhealthy we still publish
    // — but zeroed — so consumers can tell "board alive, RC data neutral"
    // apart from "board dead" using the heartbeat + /rc/link pair.
    bool receiver_healthy = (snapshot.receiver_state ==
                             static_cast<uint8_t>(ReceiverOperatingState::SignalActive));
    uint8_t channel_count = RC_CHANNEL_COUNT;
    for (uint8_t index = 0; index < channel_count; ++index)
    {
        uint16_t raw_value = snapshot.channels[index];
        const ReceiverChannelConfiguration &configuration = RC_CHANNEL_CONFIGURATION[index];
        joy_axes_storage_[index] =
            receiver_healthy ? mapChannelValueToJoyAxis(raw_value, configuration) : 0.0f;
        joy_buttons_storage_[index] =
            (receiver_healthy && channelIsActivated(raw_value, configuration)) ? 1 : 0;
    }
    joy_message_.axes.size = channel_count;
    joy_message_.buttons.size = channel_count;

    // Timestamp: zero means "unsynchronized" — populated ONLY after a
    // successful agent time sync (time_synced_) with a positive epoch.
    // Safety timeouts never use this clock.
    joy_message_.header.stamp = {};
    if (time_synced_)
    {
        int64_t epoch_nanoseconds = rmw_uros_epoch_nanos();
        if (epoch_nanoseconds > 0)
        {
            joy_message_.header.stamp.sec = static_cast<int32_t>(epoch_nanoseconds / 1000000000LL);
            joy_message_.header.stamp.nanosec =
                static_cast<uint32_t>(epoch_nanoseconds % 1000000000LL);
        }
    }

    publishTelemetryMessage(&joy_publisher_, &joy_message_);
}

void MicroRosBridge::publishBoardTelemetry(const ControlSnapshot &snapshot, uint32_t now_ms)
{
    if (!isAgentConnected())
    {
        return;
    }

    // Relay state (reliable QoS) has ONE authoritative eligibility decision
    // — the periodic health tick must NOT bypass the retry gap:
    //   fresh-connection snapshot   -> attempt immediately
    //   new target vs last ATTEMPTED -> attempt immediately
    //   same target as a failed attempt -> enforce RELAY_STATE_RETRY_PERIOD_MS
    //   periodic refresh            -> only when no pending failure
    // NOTE: latest-state SNAPSHOTS, not a guaranteed event history — a
    // failed transition can be superseded by newer state even with
    // reliable QoS.
    uint8_t relay_states = snapshot.last_written_mask;
    bool periodic_due = (now_ms - last_health_publish_ms_) >= HEALTH_PUBLISH_PERIOD_MS;
    bool relay_new_target =
        !relay_attempt_exists_ || (relay_states != last_attempted_relay_state_);
    bool relay_retry_allowed = !relay_state_publish_pending_ ||
        (now_ms - last_relay_state_publish_attempt_ms_) >= RELAY_STATE_RETRY_PERIOD_MS;

    bool relay_should_attempt;
    if (relay_state_publish_forced_ || relay_new_target)
    {
        relay_should_attempt = true; // fresh snapshot / new target: immediate
    }
    else if (relay_state_publish_pending_)
    {
        relay_should_attempt = relay_retry_allowed; // same failed target: gap
    }
    else
    {
        relay_should_attempt = periodic_due; // periodic refresh
    }

    if (!periodic_due && !relay_should_attempt)
    {
        return;
    }

    if (periodic_due)
    {
        last_health_publish_ms_ = now_ms;

        heartbeat_message_.data = now_ms / 1000; // uptime seconds: a stalled
        publishTelemetryMessage(&heartbeat_publisher_, &heartbeat_message_); // counter = wedged device

        // SEMANTICS: /rc/link = frame-state liveness (fresh frames, no
        // CONFIRMED failsafe signature) — NOT permission to operate; the
        // decoded CH6 stop and persistent invalid input can still stop
        // locally.
        link_message_.data = (snapshot.receiver_state ==
                              static_cast<uint8_t>(ReceiverOperatingState::SignalActive));
        publishTelemetryMessage(&link_publisher_, &link_message_);

        inputs_message_.data = snapshot.digital_inputs;
        publishTelemetryMessage(&inputs_publisher_, &inputs_message_);

        // SEMANTICS: /relay_board/estop = authoritative receiver-local stop,
        // computed once in control and copied here — never rebuilt from raw
        // fields. Receiver scope only; board/agent/config reasons are
        // reported separately (LED, status line, reason-code logs).
        estop_message_.data = snapshot.receiver_local_stop;
        publishTelemetryMessage(&estop_publisher_, &estop_message_);
    }

    if (relay_should_attempt)
    {
        for (uint8_t index = 0; index < RELAY_COUNT; ++index)
        {
            relay_state_storage_[index] = (relay_states >> index) & 0x01;
        }
        relay_state_message_.data.size = RELAY_COUNT;
        last_attempted_relay_state_ = relay_states;
        relay_attempt_exists_ = true;
        relay_state_publish_forced_ = false;
        last_relay_state_publish_attempt_ms_ = now_ms;
        if (publishTelemetryMessage(&relay_state_publisher_, &relay_state_message_))
        {
            relay_state_publish_pending_ = false;
        }
        else
        {
            relay_state_publish_pending_ = true; // retried at the retry cadence
        }
    }
}

} // namespace karr
