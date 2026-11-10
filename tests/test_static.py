"""Source-level regression checks: does not replace SDK compilation or RF tests."""
from pathlib import Path
import re
ROOT = Path(__file__).resolve().parents[1]
v4 = (ROOT / 'src/main.c').read_text()
v3 = (ROOT / 'reference/Hybrid_V3_main.c').read_text()

def extract_function(src, name):
    match = re.search(r'\bstatic\s+[^\n]*\b' + re.escape(name) + r'\s*\([^)]*\)\s*\{', src)
    assert match, f'Missing {name}'
    start = match.end()-1
    depth=0
    for idx in range(start,len(src)):
        if src[idx]=='{': depth+=1
        if src[idx]=='}':
            depth-=1
            if depth==0: return src[match.start():idx+1]
    raise AssertionError(f'Unbalanced {name}')

# The timing state machine and player-pair rules must remain byte-for-byte unchanged.
for name in ('get_person_id', 'should_process_wristband', 'filter_median',
             'filter_add', 'classify_rssi', 'select_nearest_legacy_target',
             'update_proximity', 'update_vibration', 'fatal_error_pattern'):
    assert extract_function(v4,name)==extract_function(v3,name), f'Regression in {name}'

for text in (
    '#define LEGACY_INTERVAL_UNITS       80',
    '#define PERIODIC_INTERVAL_UNITS     40',
    '#define SCAN_WINDOW_UNITS          128',
    '#define RSSI_UPDATE_INTERVAL_MS    200',
    '#define EMA_ALPHA                 0.35f',
    'DM_ROLE_REFLECTOR',
    'DM_ROLE_INITIATOR',
    'dm_request_add(&req)',
    '#ifdef CONFIG_DM_MODULE',
    'bt_le_per_adv_sync_create',
    'dm_data_ready',
    'dm_try_initiate_from_scan_response',
    'bt_le_ext_adv_set_data(legacy_adv',
    'ENABLE_PERIODIC',
    'DM_TEST_SEED + dm_addr_share(&info->addr->a)',
    'parsed->dm_seed + my_scanner_address_share',
):
    assert text in v4, f'Missing required element: {text}'
assert (ROOT / 'dm.conf').exists()
assert (ROOT / 'boards/xiao_ble_nrf52840.overlay').exists()
assert (ROOT / 'scripts/build_player.ps1').exists()
print('PASS: preserved V3 logic and required V4 structure (static checks only).')
