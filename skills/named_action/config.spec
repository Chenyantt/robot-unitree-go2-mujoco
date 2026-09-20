config:
  bridge_url:
    type: string
    default: http://127.0.0.1:8766
    description: Local simulator bridge HTTP endpoint.
  poll_interval_s:
    type: float
    default: 0.05
    description: Runtime state polling interval while a named action owns control.
  release_timeout_s:
    type: float
    default: 10.0
    description: Maximum wait for competing Twist publishers to become quiet before release.
  admission_timeout_s:
    type: float
    default: 5.0
    description: Maximum wait for base and leg motion to settle before action admission.
