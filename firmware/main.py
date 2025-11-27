import bobot
import os
import time
import sys

# Try to run user program "program.py" in a separate thread if it exists
try:
    import _thread
    
    def run_program():
        # Wait for bobot to initialize (BLE/UART)
        print("[BOOT] Waiting for system initialization...")
        for _ in range(50): # Wait up to 5s
            if getattr(bobot, 'uart', None) is not None:
                break
            time.sleep(0.1)
        
        time.sleep(0.5) # Extra small delay to ensure stability
        
        # Check if program.py exists
        try:
            os.stat('program.py')
        except OSError:
            print("[BOOT] No program.py found")
            return

        try:
            print("[BOOT] Found program.py, executing...")
            
            # Read code and execute it (better than import for scripts)
            with open('program.py', 'r') as f:
                code = f.read()
            
            # Create isolated globals but ensure it can import system modules
            # bobot module is already in sys.modules so 'import bobot' inside program works correctly
            prog_globals = {'__name__': '__main__'}
            exec(code, prog_globals)
            
        except Exception as e:
            print("[BOOT] Error in program.py:", e)
            # Write error to file for debugging
            try:
                with open('program_error.log', 'w') as f:
                    f.write(str(e))
            except:
                pass
            
    _thread.start_new_thread(run_program, ())

except ImportError:
    print("[BOOT] _thread not available, running only bobot firmware")

# Start the firmware main loop (blocks)
if __name__ == '__main__':
    print("[BOOT] Starting bobot firmware...")
    bobot.main()
