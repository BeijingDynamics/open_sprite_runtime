"""Distinguish a completed, rejected zero-gain run from a runtime fault."""

import json


def completed_rejected_shadow(path, returncode, duration):
    # Only defer a final qualification rejection. Never defer an early fault,
    # enabled-motor report, missing evidence, or a different executable mode.
    if returncode != 2:
        return False
    try:
        report = json.loads(path.read_text())
        return (
            report['mode'] == 'native_xbox_500hz_zero_gain_can_shadow'
            and report['passed'] is False
            and duration <= report['elapsed_s'] <= duration + 1
            and report['state_ticks'] == duration * 500
            and report['nonzero_gain_or_torque_tx_attempts'] == 0
            and report['automatic_enable_attempts'] == 0
            and report['automatic_mode_switch_attempts'] == 0
        )
    except (OSError, ValueError, KeyError, TypeError):
        return False


def can_diagnostics(path):
    try:
        report = json.loads(path.read_text())
    except (OSError, ValueError):
        return {'can_report_available': False}
    reasons = []
    if report.get('transport_slot_overruns', 0):
        reasons.append('transport_slot_overruns')
    if report.get('deadline_misses', 0):
        reasons.append('state_tick_deadline_misses')
    if report.get('lateness_p99_ms', 0) > 0.5 or report.get('lateness_max_ms', 0) > 2:
        reasons.append('state_tick_lateness')
    for key in ('coverage_passed', 'policy_ipc_passed'):
        if report.get(key) is False:
            reasons.append(key)
    if report.get('passed') is False and not reasons:
        reasons.append('unclassified_native_rejection_inspect_can_report')
    return {
        'can_report_available': True,
        'can_qualification_reasons': reasons,
        'can_timing': {key: value for key, value in report.items()
                       if key.startswith(('transport_', 'lateness_')) or key == 'deadline_misses'},
    }
