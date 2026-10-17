#pragma once

namespace auto_battlebot {
/** Where a policy runs. INLINE calls it on the control thread every policy_period_ticks. */
enum class PolicyExecution { INLINE };
}  // namespace auto_battlebot
