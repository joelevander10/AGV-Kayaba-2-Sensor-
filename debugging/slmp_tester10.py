import pymcprotocol
import time

# 1. Initialize and Connect
plc = pymcprotocol.Type3E(plctype="Q")
plc.setaccessopt(commtype="binary")
plc.connect(ip="192.168.4.10", port=1280)

print("Logic started. Press Ctrl+C to stop.")

try:
    step = 0
    while True:
        # --- M100-M104 LOGIC (Every 0.5s) ---
        # Toggle: step 0 = ON, step 1 = OFF, step 2 = ON...
        m_state = 1 if (step % 2 == 0) else 0
        plc.batchwrite_bitunits(headdevice="M100", values=[m_state] * 5)

        # --- D10, D11, D12 LOGIC (Evaluated every 0.5s) ---
        # We define values based on a 4-second total cycle (8 steps of 0.5s)
        # to accommodate the 2s requirement for D11.
        
        cycle_pos = step % 8  # Cycles every 4 seconds
        
        # D10: 100 (1s), 200 (1s), back to 100
        # Steps 0-1 (1s): 100 | Steps 2-3 (1s): 200
        d10_val = 200 if (2 <= cycle_pos <= 3) else 100

        # D11: 200 (1s), 100 (2s), back to 200
        # Steps 0-1 (1s): 200 | Steps 2-5 (2s): 100 | Steps 6-7: 200
        if 2 <= cycle_pos <= 5:
            d11_val = 100
        else:
            d11_val = 200

        # D12: 300 (1s), 499 (1s)
        # Steps 0-1: 300 | Steps 2-3: 499 | Steps 4-5: 300 | Steps 6-7: 499
        d12_val = 499 if (cycle_pos in [2, 3, 6, 7]) else 300

        # Write D values
        plc.batchwrite_wordunits(headdevice="D10", values=[d10_val, d11_val, d12_val])

        # Increment step and wait 0.5 seconds
        step += 1
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nStopping and cleaning up...")
    # Optional: Turn off M devices on exit
    plc.batchwrite_bitunits(headdevice="M100", values=[0, 0, 0, 0, 0])
    plc.close()