import sys
import time
import subprocess
import psutil
import matplotlib.pyplot as plt
import os

def monitor_memory(cmd, log_file):
    print(f"Starting process: {' '.join(cmd)}")
    process = subprocess.Popen(cmd)
    
    try:
        ps_proc = psutil.Process(process.pid)
    except psutil.NoSuchProcess:
        print("Process failed to start.")
        return

    times = []
    mem_mbs = []
    start_time = time.time()

    print("Monitoring memory... (Press Ctrl+C to stop and plot early)")
    
    try:
        while True:
            # Check if process has terminated
            if process.poll() is not None:
                break
                
            try:
                # Get RSS memory in MB
                mem_info = ps_proc.memory_info()
                rss_mb = mem_info.rss / (1024 * 1024)
                
                times.append(time.time() - start_time)
                mem_mbs.append(rss_mb)
                
            except psutil.NoSuchProcess:
                break
                
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\nMonitoring stopped by user.")
        process.terminate()
        process.wait()

    print(f"Process finished. Max Memory: {max(mem_mbs):.2f} MB")
    
    # Plotting
    plt.figure(figsize=(10, 6))
    plt.plot(times, mem_mbs, label='ORB-SLAM3 (Sparsification ON)', linewidth=2, color='b')
    plt.title('Memory Usage (RSS) over Time')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Memory Usage (MB)')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    
    plt.savefig('memory_usage.png')
    print(f"Memory usage plot saved to 'memory_usage.png'")
    
    # Save raw data
    with open('memory_log.txt', 'w') as f:
        for t, m in zip(times, mem_mbs):
            f.write(f"{t:.2f},{m:.2f}\n")

if __name__ == "__main__":
    # The default command to run
    cmd = [
        "./Examples/Stereo-Inertial/stereo_inertial_euroc",
        "./Vocabulary/orbvoc.dbow3",
        "./Examples/Stereo-Inertial/EuRoC.yaml",
        "/home/ywl/dataset/vicon_room1/V1_01_easy",
        "./Examples/Stereo-Inertial/EuRoC_TimeStamps/V101.txt"
    ]
    
    if len(sys.argv) > 1:
        cmd = sys.argv[1:]
        
    monitor_memory(cmd, 'memory_usage.png')
