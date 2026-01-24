import serial
import time
import numpy as np
import matplotlib.pyplot as plt

def parse_line(line):
    """Parse a line of IMU data and return a dictionary of values."""
    try:
        fields = line.strip().split('|')
        data = {}
        for field in fields:
            if ':' in field:
                key, value = field.split(':')
                data[key.strip()] = float(value.strip())
        return data
    except:
        return None

if __name__ == "__main__":
    # Open serial port
    ser = serial.Serial('/dev/ttyUSB0', 921600, timeout=2)
    
    acc_deg_list = []
    kalman_list = []

    num_of_samples = 1000
    sample_rate_ms = 10
    
    print(f"Collecting {num_of_samples} samples...")
    print("-" * 60)
    
    # Read and parse lines from the serial port
    for i in range(num_of_samples):
        line = ser.readline()
        if not line:
            continue
            
        decoded = line.decode('utf-8', errors='ignore').strip()
        data = parse_line(decoded)
        
        if data and 'acc_deg' in data and 'gyro_deg' in data:
            acc_deg_list.append(data['acc_deg'])
            kalman_list.append(data['gyro_deg'])
            
            if i % 100 == 0:
                print(f"Sample {i}: acc_deg={data['acc_deg']:.2f}° kalman={data['gyro_deg']:.2f}°")

    ser.close()
    
    # Ensure we have data
    if len(acc_deg_list) == 0:
        print("No data collected!")
        exit(1)
    
    # Convert to numpy arrays
    acc_deg = np.array(acc_deg_list)
    kalman = np.array(kalman_list)
    
    # Calculate statistics
    acc_std = np.std(acc_deg)
    acc_mean = np.mean(acc_deg)
    acc_range = np.max(acc_deg) - np.min(acc_deg)
    
    kalman_std = np.std(kalman)
    kalman_mean = np.mean(kalman)
    kalman_range = np.max(kalman) - np.min(kalman)
    
    noise_reduction = (1 - kalman_std / acc_std) * 100
    range_reduction = (1 - kalman_range / acc_range) * 100
    
    # Print statistics
    print("\n" + "=" * 60)
    print("KALMAN FILTER PERFORMANCE ANALYSIS")
    print("=" * 60)
    print(f"\nSamples collected: {len(acc_deg)}")
    print(f"Sample rate: {sample_rate_ms} ms")
    print(f"Total time: {len(acc_deg) * sample_rate_ms / 1000:.1f} seconds")
    
    print("\n--- Accelerometer (Raw) ---")
    print(f"  Mean:     {acc_mean:+.3f}°")
    print(f"  Std Dev:  {acc_std:.3f}°")
    print(f"  Min:      {np.min(acc_deg):+.3f}°")
    print(f"  Max:      {np.max(acc_deg):+.3f}°")
    print(f"  Range:    {acc_range:.3f}°")
    
    print("\n--- Kalman Filter (Filtered) ---")
    print(f"  Mean:     {kalman_mean:+.3f}°")
    print(f"  Std Dev:  {kalman_std:.3f}°")
    print(f"  Min:      {np.min(kalman):+.3f}°")
    print(f"  Max:      {np.max(kalman):+.3f}°")
    print(f"  Range:    {kalman_range:.3f}°")
    
    print("\n--- Improvement ---")
    print(f"  Noise reduction (std dev): {noise_reduction:.1f}%")
    print(f"  Range reduction:           {range_reduction:.1f}%")
    print("=" * 60)
    
    # Create time axis
    t = np.linspace(0, len(acc_deg) * sample_rate_ms / 1000, len(acc_deg))
    
    # Create figure with subplots
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Kalman Filter Performance Analysis', fontsize=14, fontweight='bold')
    
    # Plot 1: Time series comparison
    ax1 = axes[0, 0]
    ax1.plot(t, acc_deg, 'b-', alpha=0.7, linewidth=0.8, label=f'Accelerometer (σ={acc_std:.3f}°)')
    ax1.plot(t, kalman, 'r-', alpha=0.9, linewidth=1.2, label=f'Kalman Filter (σ={kalman_std:.3f}°)')
    ax1.axhline(y=acc_mean, color='b', linestyle='--', alpha=0.5)
    ax1.axhline(y=kalman_mean, color='r', linestyle='--', alpha=0.5)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Angle (°)')
    ax1.set_title('Angle Measurement Over Time')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Zoomed view (first 2 seconds)
    ax2 = axes[0, 1]
    zoom_samples = min(200, len(acc_deg))
    t_zoom = t[:zoom_samples]
    ax2.plot(t_zoom, acc_deg[:zoom_samples], 'b-', alpha=0.7, linewidth=1, label='Accelerometer')
    ax2.plot(t_zoom, kalman[:zoom_samples], 'r-', alpha=0.9, linewidth=1.5, label='Kalman Filter')
    ax2.fill_between(t_zoom, acc_deg[:zoom_samples], kalman[:zoom_samples], alpha=0.2, color='green')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Angle (°)')
    ax2.set_title('Zoomed View (First 2 seconds)')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Histogram comparison
    ax3 = axes[1, 0]
    bins = 50
    ax3.hist(acc_deg, bins=bins, alpha=0.6, color='blue', label=f'Accelerometer\nσ={acc_std:.3f}°', density=True)
    ax3.hist(kalman, bins=bins, alpha=0.6, color='red', label=f'Kalman Filter\nσ={kalman_std:.3f}°', density=True)
    ax3.axvline(x=acc_mean, color='blue', linestyle='--', linewidth=2)
    ax3.axvline(x=kalman_mean, color='red', linestyle='--', linewidth=2)
    ax3.set_xlabel('Angle (°)')
    ax3.set_ylabel('Density')
    ax3.set_title('Distribution Comparison')
    ax3.legend(loc='upper right')
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Statistics bar chart
    ax4 = axes[1, 1]
    metrics = ['Std Dev\n(°)', 'Range\n(°)']
    acc_values = [acc_std, acc_range]
    kalman_values = [kalman_std, kalman_range]
    
    x_pos = np.arange(len(metrics))
    width = 0.35
    
    bars1 = ax4.bar(x_pos - width/2, acc_values, width, label='Accelerometer', color='blue', alpha=0.7)
    bars2 = ax4.bar(x_pos + width/2, kalman_values, width, label='Kalman Filter', color='red', alpha=0.7)
    
    # Add value labels on bars
    for bar, val in zip(bars1, acc_values):
        ax4.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01, f'{val:.3f}', 
                 ha='center', va='bottom', fontsize=10)
    for bar, val in zip(bars2, kalman_values):
        ax4.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01, f'{val:.3f}', 
                 ha='center', va='bottom', fontsize=10)
    
    ax4.set_ylabel('Value')
    ax4.set_title(f'Noise Metrics (Reduction: {noise_reduction:.1f}%)')
    ax4.set_xticks(x_pos)
    ax4.set_xticklabels(metrics)
    ax4.legend(loc='upper right')
    ax4.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    output_path = '../img/kalman_filter_analysis.png'
    plt.savefig(output_path, dpi=150)
    print(f"\nPlot saved to: {output_path}")
    plt.show()




