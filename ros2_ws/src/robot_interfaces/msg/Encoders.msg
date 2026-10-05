# Live quadrature counts. Signed and hardware-decoded: a wheel rolled backwards
# down a slope counts DOWN. Direction is measured, not inferred from the
# commanded move.

int32 left
int32 right
int32 sync_error            # left - right; a value pinned to one side across
                            # every move means the motor<->encoder pairing is
                            # crossed, not that the gain needs tuning.
