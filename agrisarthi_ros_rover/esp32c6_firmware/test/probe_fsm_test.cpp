#include "../probe_fsm.h"
#include <cassert>
#include <cstring>

int main() {
  ProbeStateMachine probe;
  assert(probe.state() == ProbeState::IDLE);
  assert(!probe.motorAllowed());

  ProbeCommandResult retract = probe.command("cmd-r1", "PROBE_RETRACT", 100, 1000, 100, 3000);
  assert(retract.accepted && retract.beginRetract);
  assert(probe.state() == ProbeState::RETRACTING && !probe.motorAllowed());
  assert(probe.tick(1200, 300, false, false, false));
  assert(probe.state() == ProbeState::RETRACTED && probe.motorAllowed() && probe.positionAssumed());

  ProbeCommandResult insert = probe.command("cmd-i1", "PROBE_INSERT", 1300, 1000, 100, 3000);
  assert(insert.accepted && insert.beginInsert && !probe.motorAllowed());
  ProbeCommandResult duplicate = probe.command("cmd-i1", "PROBE_INSERT", 1400, 1000, 100, 3000);
  assert(duplicate.accepted && duplicate.duplicate && !duplicate.beginInsert);
  assert(probe.tick(2400, 300, false, false, false));
  assert(probe.state() == ProbeState::INSERTED && !probe.motorAllowed());
  duplicate = probe.command("cmd-i1", "PROBE_INSERT", 2500, 1000, 100, 3000);
  assert(duplicate.accepted && duplicate.duplicate && probe.state() == ProbeState::INSERTED);
  bool cachedAccepted = false;
  assert(probe.cachedResult("cmd-i1", "PROBE_INSERT", cachedAccepted) && cachedAccepted);
  assert(!probe.cachedResult("unknown", "PROBE_INSERT", cachedAccepted));

  ProbeCommandResult conflict = probe.command("cmd-i1", "PROBE_RETRACT", 2501, 1000, 100, 3000);
  assert(conflict.duplicate && !conflict.accepted);
  ProbeCommandResult status = probe.command("cmd-status", "PROBE_STATUS", 2502, 1000, 100, 3000);
  assert(status.accepted && !status.beginInsert && !status.beginRetract);

  ProbeCommandResult safeRetract = probe.command("cmd-r2", "PROBE_RETRACT", 2600, 1000, 100, 3000);
  assert(safeRetract.accepted && safeRetract.beginRetract);
  assert(probe.tick(3701, 300, false, false, false));
  assert(probe.state() == ProbeState::RETRACTED && probe.motorAllowed());

  probe.command("cmd-i2", "PROBE_INSERT", 4000, 1000, 100, 500);
  assert(probe.tick(4501, 300, false, false, false));
  assert(probe.state() == ProbeState::ERROR && !probe.motorAllowed());
  assert(std::strcmp(probe.error(), "SERVO_TIMEOUT") == 0);
  return 0;
}
