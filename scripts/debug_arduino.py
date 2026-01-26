#!/usr/bin/env python3
import serial
import time
import sys

def debug_arduino():
    port = '/dev/ttyUSB1'
    baud = 115200
    
    print(f"Attempting to connect to {port} at {baud}...")
    
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2) # Wait for reset
        
        print("Connected! Streaming output...")
        
        def read_serial():
            while True:
                if ser.in_waiting:
                    try:
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            print(f"[ARDUINO] {line}")
                    except:
                        pass
                time.sleep(0.01)

        # Function to read current pos
        def get_pos():
            ser.write(b"READ_POS\n")
            time.sleep(0.1)
            # Read from the stream thread? No, stream thread consumes it.
            # We need to pause stream thread or just rely on stream output?
            # Actually, read_serial consumes everything. get_pos will never see it.
            # We should disable read_serial or make it smarter.
            # For simplicity, let's just rely on the stream thread printing it.
            # We can't return it easily.
            # Let's just send READ_POS and let the stream thread print it.
            return "Check Log Above"

        # Start reader thread
        import threading
        t = threading.Thread(target=read_serial)
        t.daemon = True
        t.start()

        time.sleep(3) # Wait for boot
        
        import argparse
        parser = argparse.ArgumentParser()
        parser.add_argument('--joint', type=int, help='Joint index to test (0-4)')
        args = parser.parse_args()

        joints_to_test = []
        all_joints = [
            (0, "Base"),
            (1, "Shoulder"),
            (2, "Elbow"),
            (3, "Wrist 1"),
            (4, "Wrist 2")
        ]

        if args.joint is not None:
            if 0 <= args.joint <= 4:
                joints_to_test.append(all_joints[args.joint])
            else:
                print(f"Invalid joint index {args.joint}. Must be 0-4.")
                return
        else:
            joints_to_test = all_joints
        
        print("\n--- DIAGNOSTIC: Individual Joint Wiggle ---")
        
        for j_idx, j_name in joints_to_test:
            print(f"\nTesting {j_name} (J{j_idx})...")
            
            # Get initial pos
            initial_line = get_pos()
            print(f"  Start: {initial_line}")
            
            # Move +0.4 rad (larger movement)
            # Construct J command: all 0 except target
            vals = [0.0] * 6
            vals[j_idx] = 0.4
            cmd_str = ",".join([f"{v:.2f}" for v in vals])
            cmd = f"J{cmd_str}\n"
            
            print(f"  Sending: {cmd.strip()}")
            ser.write(cmd.encode())
            time.sleep(2.0)
            
            mid_line = get_pos()
            print(f"  Mid:   {mid_line}")
            
            # Move back to 0
            vals[j_idx] = 0.0
            cmd_str = ",".join([f"{v:.2f}" for v in vals])
            cmd = f"J{cmd_str}\n"
            
            print(f"  Resetting: {cmd.strip()}")
            ser.write(cmd.encode())
            time.sleep(2.0)
            
            end_line = get_pos()
            print(f"  End:   {end_line}")

        print("\nDone. Closing.")
        ser.close()
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    debug_arduino()
