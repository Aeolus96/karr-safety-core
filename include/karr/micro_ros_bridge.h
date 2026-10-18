#pragma once

// ============================================================================
// MicroRosBridge — agent connection management + telemetry publishers.
// PUBLISH-ONLY by design: there is deliberately no subscription and no
// executor, so no external ROS topic can ever actuate the relays. Consumes
// coherent ControlSnapshot copies — never reads live RcReceiver /
// RelayBoard instances. All micro-ROS calls stay in the ROS execution
// context (Arduino loop on core 1).
// ============================================================================

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/joy.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/u_int8_multi_array.h>

#include "karr/control_snapshot.h"
#include "karr/rc_config.h"
#include "karr/relay_board.h"

namespace karr
{

class MicroRosBridge
{
public:
    // Sets up the SPI bus and the micro-ROS ethernet transport (which also
    // configures the W5500 with the static IP). Attaches static message
    // storage. Entity creation happens later, in service().
    void begin();

    // Explicit handle zero-initialization. Static globals are zeroed by
    // the C runtime, but the library is designed around explicit
    // zero-initialized handles (rcl_get_zero_initialized_*) — doing it in
    // the constructor keeps the class correct even if it is ever
    // instantiated differently (stack, array, re-created at runtime).
    MicroRosBridge();

    // Throttled agent ping + connection state machine + entity lifecycle.
    void service(uint32_t now_ms);

    bool isAgentConnected() const;

    // Human-readable agent state for the serial status line
    // (WAITING / AVAILABLE / CONNECTED / CLEANUP).
    const char *agentStateText() const;

    // Telemetry publishers (all no-ops while the agent is not connected).
    // Consume a coherent ControlSnapshot filled by the control task — no
    // live cross-core reads of RcReceiver / RelayBoard.
    void publishReceiverTelemetry(const ControlSnapshot &snapshot, uint32_t now_ms);
    void publishBoardTelemetry(const ControlSnapshot &snapshot, uint32_t now_ms);

private:
    bool initializeEntities();
    bool destroyEntities();
    void resetEntityHandles();
    // Finalizes one child publisher if its flag is set (no-op otherwise):
    // clears the flag on success, keeps it (preserving the parent) on
    // failure. Collapses the 6x fini/clear/log pattern; the call order in
    // destroyEntities stays explicit so the child-before-parent teardown
    // boundary is never hidden in a loop.
    void finalizePublisher(rcl_publisher_t &publisher, bool &created_flag, const char *name);
    bool publishTelemetryMessage(rcl_publisher_t *publisher, const void *message);

    // micro-ROS core
    rcl_allocator_t allocator_;
    rclc_support_t support_;
    rcl_node_t node_;

    // publishers (no subscriber — see header)
    rcl_publisher_t joy_publisher_;
    rcl_publisher_t link_publisher_;
    rcl_publisher_t heartbeat_publisher_;
    rcl_publisher_t relay_state_publisher_;
    rcl_publisher_t inputs_publisher_;
    rcl_publisher_t estop_publisher_;

    // lifecycle flags: the agent can drop at ANY time, so teardown must
    // only touch entities that were actually created.
    struct EntityFlags
    {
        bool support = false;
        bool node = false;
        bool joy_publisher = false;
        bool link_publisher = false;
        bool heartbeat_publisher = false;
        bool relay_state_publisher = false;
        bool inputs_publisher = false;
        bool estop_publisher = false;

        // Any child publisher still created? Used as the teardown gate: the
        // node is finalized only once every child has been released.
        bool anyPublishers() const
        {
            return joy_publisher || link_publisher || heartbeat_publisher ||
                   relay_state_publisher || inputs_publisher || estop_publisher;
        }

        // Any resource at all? Used as the "nothing to do" fast path.
        bool anyResources() const
        {
            return support || node || anyPublishers();
        }
    };
    EntityFlags entities_;

    // agent connection state machine
    enum class AgentState
    {
        WaitingForAgent, // pinging until the agent answers
        AgentAvailable,  // answered — next cycle creates entities
        AgentConnected,  // entities live, publishers active
        CleanupPending,  // agent lost OR entity init failed — entities must
                         // be torn down (possibly over several cycles)
                         // before the next connect attempt
    };
    AgentState agent_state_ = AgentState::WaitingForAgent;
    AgentState previous_agent_state_ = AgentState::WaitingForAgent;
    uint32_t last_ping_ms_ = 0;
    uint32_t last_cleanup_attempt_ms_ = 0; // teardown dispatch schedule
    // Persistent support-abandonment count: repeated abandonments (each a
    // possible local-allocation leak) stay visible across reconnect cycles.
    // Monotonic — never reset — so a reconnect-and-leak cycle cannot hide.
    uint32_t support_abandonment_count_ = 0;
    bool time_sync_pending_ = false; // set on connect; executed on a later
                                     // ping, never in the entity-creation call
    bool time_synced_ = false;       // set only after a SUCCESSFUL sync;
                                     // cleared on loss/teardown — Joy stamps
                                     // are populated only while this is true

    // publish cadence / change tracking
    uint32_t last_joy_publish_ms_ = 0;
    uint32_t last_health_publish_ms_ = 0;
    // /relay_board/relay_state (reliable QoS) publication tracking:
    //  - last_attempted_relay_state_ + relay_attempt_exists_ : the target of
    //    the last attempt (SUCCESS NOT REQUIRED — a failed attempt still sets
    //    the baseline, so only a NEW target bypasses the retry gap)
    //  - relay_state_publish_pending_ : last attempt FAILED (retry gap applies)
    //  - relay_state_publish_forced_ : fresh-connection snapshot request
    // ONE eligibility decision consumes these (see publishBoardTelemetry).
    uint8_t last_attempted_relay_state_ = SAFE_RELAY_STATE;
    bool relay_attempt_exists_ = false;
    bool relay_state_publish_pending_ = false;
    bool relay_state_publish_forced_ = false;
    uint32_t last_relay_state_publish_attempt_ms_ = 0;
    uint32_t publish_failure_count_ = 0;

    // messages + static storage for every sequence-bearing message.
    // In-class value-initialization ({}): nested fields such as
    // frame_id/layout are zeroed independently of storage duration, so the
    // class is also correct if ever instantiated on the stack.
    sensor_msgs__msg__Joy joy_message_ = {};
    std_msgs__msg__Bool link_message_ = {};
    std_msgs__msg__UInt32 heartbeat_message_ = {};
    std_msgs__msg__UInt8MultiArray relay_state_message_ = {};
    std_msgs__msg__UInt8 inputs_message_ = {};
    std_msgs__msg__Bool estop_message_ = {};
    float joy_axes_storage_[RC_CHANNEL_COUNT] = {};
    int32_t joy_buttons_storage_[RC_CHANNEL_COUNT] = {};
    uint8_t relay_state_storage_[RELAY_COUNT] = {};
};

} // namespace karr
