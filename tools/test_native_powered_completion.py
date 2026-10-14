"""Pure report validation tests; no hardware."""
import copy
from check_native_powered_completion import validate_completion

names = [f'motor_{i}' for i in range(31)]
good = dict(mode='native_suspended_protected_policy_actuation', passed=True, errors=[],
            duration_s=40, deadline_misses=0, automatic_mode_switch_attempts=0,
            automatic_zero_reset_attempts=0, motors={n: {'final_disabled': True} for n in names})
validate_completion(good, names, 0, 0)
cases = []
for key, value in [('mode','shadow'), ('passed',False), ('errors',['fault']),
                   ('duration_s',8), ('deadline_misses',1), ('automatic_zero_reset_attempts',1)]:
    bad = copy.deepcopy(good)
    bad[key] = value
    cases.append((bad, 0, 0))
bad = copy.deepcopy(good)
del bad['motors'][names[0]]
cases.append((bad, 0, 0))
bad = copy.deepcopy(good)
bad['motors'][names[0]]['final_disabled'] = False
cases.append((bad, 0, 0))
cases.extend([(good, 1, 0), (good, 0, 2)])
for bad, actor, controller in cases:
    try:
        validate_completion(bad, names, actor, controller)
    except ValueError:
        continue
    raise AssertionError('invalid completion accepted')
print(f'PASS {len(cases)+1} report gates; no hardware')
