---
description: Execute one profile-gated native Go2 action with exclusive motor ownership and recovery.
---

# Named Action

Use `execute` with one name returned by `list`, retain the `run_id`, and poll
`status` until terminal. The skill validates the active environment, delegates
admission to the Native controller, waits through action and recovery, then
releases the exact controller hold. `cancel` is asynchronous and addresses only
the matching run.
