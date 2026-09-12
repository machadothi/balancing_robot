#!/usr/bin/env python3
"""
Filter Comparison Analysis Tool
Compares Kalman and Complementary filters for IMU angle estimation
"""

import serial
import time
import numpy as np
import matplotlib.pyplot as plt

def parse_line(line):
    """Parse a line of IMU data and return a dictionary of values.
    
    Expected format:
    ax: X.XX | ay: X.XX | az: X.XX | gx: X.XX | gy: X.XX | gz: X.XX | acc_deg: X.XX | kalman: X.XX | comp: X.XX
    """
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

def calculate_stats(data, name):
    """Calculate statistics for a data array."""
    return {
        'name': name,
        'mean': np.mean(data),
        'std': np.std(data),
        'min': np.min(data),
        'max': np.max(data),
        'range': np.max(data) - np.min(data)
    }

def print_stats(stats):
    """Print statistics in a formatted way."""
    print(f"\n--- {stats['name']} ---")
    print(f"  Mean:     {stats['mean']:+.3f}°")
    print(f"  Std Dev:  {stats['std']:.3f}°")
    print(f"  Min:      {stats['min']:+.3f}°")
    print(f"  Max:      {stats['max']:+.3f}°")
    print(f"  Range:    {stats['range']:.3f}°")

if __name__ == "__main__":
    # Configuration
    SERIAL_PORT = '/dev/ttyUSB0'
    BAUD_RATE = 921600
    NUM_SAMPLES = 1000
    SAMPLE_RATE_MS = 10
    
    # Open serial port
    print(f"Opening {SERIAL_PORT} at {BAUD_RATE} baud...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
    
    # Data storage
    acc_deg_list = []
    kalman_list = []
    comp_list = []

    print(f"Collecting {NUM_SAMPLES} samples...")
    print("-" * 70)
    
    # Read and parse lines from the serial port
    collected = 0
    attempts = 0
    max_attempts = NUM_SAMPLES * 3  # Allow for some bad reads
    
    while collected < NUM_SAMPLES and attempts < max_attempts:
        line = ser.readline()
        attempts += 1
        
        if not line:
            continue
            
        decoded = line.decode('utf-8', errors='ignore').strip()
        data = parse_line(decoded)
        
        if data and 'acc_deg' in data and 'kalman' in data and 'comp' in data:
            acc_deg_list.append(data['acc_deg'])
            kalman_list.append(data['kalman'])
            comp_list.append(data['comp'])
            collected += 1
            
            if collected % 100 == 0:
                print(f"Sample {collected}: acc={data['acc_deg']:+.2f}° | "
                      f"kalman={data['kalman']:+.2f}° | comp={data['comp']:+.2f}°")

    ser.close()
    
    # Ensure we have data
    if len(acc_deg_list) == 0:
        print("No data collected!")
        exit(1)
    
    # Convert to numpy arrays
    acc_deg = np.array(acc_deg_list)
    kalman = np.array(kalman_list)
    comp = np.array(comp_list)
    
    # Calculate statistics
    acc_stats = calculate_stats(acc_deg, "Accelerometer (Raw)")
    kalman_stats = calculate_stats(kalman, "Kalman Filter")
    comp_stats = calculate_stats(comp, "Complementary Filter")
    
    # Calculate improvements
    kalman_noise_reduction = (1 - kalman_stats['std'] / acc_stats['std']) * 100
    comp_noise_reduction = (1 - comp_stats['std'] / acc_stats['std']) * 100
    kalman_range_reduction = (1 - kalman_stats['range'] / acc_stats['range']) * 100
    comp_range_reduction = (1 - comp_stats['range'] / acc_stats['range']) * 100
    
    # Print statistics
    print("\n" + "=" * 70)
    print("FILTER COMPARISON ANALYSIS")
    print("=" * 70)
    print(f"\nSamples collected: {len(acc_deg)}")
    print(f"Sample rate: {SAMPLE_RATE_MS} ms")
    print(f"Total time: {len(acc_deg) * SAMPLE_RATE_MS / 1000:.1f} seconds")
    
    print_stats(acc_stats)
    print_stats(kalman_stats)
    print_stats(comp_stats)
    
    print("\n--- Improvement Summary ---")
    print(f"                      Kalman    Complementary")
    print(f"  Noise reduction:    {kalman_noise_reduction:+.1f}%     {comp_noise_reduction:+.1f}%")
    print(f"  Range reduction:    {kalman_range_reduction:+.1f}%     {comp_range_reduction:+.1f}%")
    print("=" * 70)
    
    # Determine winner
    if kalman_stats['std'] < comp_stats['std']:
        winner_noise = "Kalman"
        noise_diff = (comp_stats['std'] - kalman_stats['std']) / comp_stats['std'] * 100
    else:
        winner_noise = "Complementary"
        noise_diff = (kalman_stats['std'] - comp_stats['std']) / kalman_stats['std'] * 100
    
    print(f"\n  Winner (noise): {winner_noise} ({noise_diff:.1f}% better)")
    
    # Create time axis
    t = np.linspace(0, len(acc_deg) * SAMPLE_RATE_MS / 1000, len(acc_deg))
    
    # Create figure with subplots
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Filter Comparison: Kalman vs Complementary', fontsize=14, fontweight='bold')
    
    # Color scheme
    color_acc = '#2196F3'     # Blue
    color_kalman = '#F44336'   # Red
    color_comp = '#4CAF50'     # Green
    
    # Plot 1: Time series comparison (all three)
    ax1 = axes[0, 0]
    ax1.plot(t, acc_deg, color=color_acc, alpha=0.5, linewidth=0.5, label=f'Accelerometer (σ={acc_stats["std"]:.3f}°)')
    ax1.plot(t, kalman, color=color_kalman, alpha=0.9, linewidth=1.0, label=f'Kalman (σ={kalman_stats["std"]:.3f}°)')
    ax1.plot(t, comp, color=color_comp, alpha=0.9, linewidth=1.0, label=f'Complementary (σ={comp_stats["std"]:.3f}°)')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Angle (°)')
    ax1.set_title('Angle Measurement Over Time')
    ax1.legend(loc='upper right', fontsize=9)
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Zoomed view (first 2 seconds)
    ax2 = axes[0, 1]
    zoom_samples = min(200, len(acc_deg))
    t_zoom = t[:zoom_samples]
    ax2.plot(t_zoom, acc_deg[:zoom_samples], color=color_acc, alpha=0.5, linewidth=0.8, label='Accelerometer')
    ax2.plot(t_zoom, kalman[:zoom_samples], color=color_kalman, alpha=0.9, linewidth=1.2, label='Kalman')
    ax2.plot(t_zoom, comp[:zoom_samples], color=color_comp, alpha=0.9, linewidth=1.2, label='Complementary')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Angle (°)')
    ax2.set_title('Zoomed View (First 2 seconds)')
    ax2.legend(loc='upper right', fontsize=9)
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Histogram comparison
    ax3 = axes[1, 0]
    bins = 50
    ax3.hist(acc_deg, bins=bins, alpha=0.4, color=color_acc, label=f'Accelerometer (σ={acc_stats["std"]:.3f}°)', density=True)
    ax3.hist(kalman, bins=bins, alpha=0.6, color=color_kalman, label=f'Kalman (σ={kalman_stats["std"]:.3f}°)', density=True)
    ax3.hist(comp, bins=bins, alpha=0.6, color=color_comp, label=f'Complementary (σ={comp_stats["std"]:.3f}°)', density=True)
    ax3.axvline(x=acc_stats['mean'], color=color_acc, linestyle='--', linewidth=2, alpha=0.7)
    ax3.axvline(x=kalman_stats['mean'], color=color_kalman, linestyle='--', linewidth=2)
    ax3.axvline(x=comp_stats['mean'], color=color_comp, linestyle='--', linewidth=2)
    ax3.set_xlabel('Angle (°)')
    ax3.set_ylabel('Density')
    ax3.set_title('Distribution Comparison')
    ax3.legend(loc='upper right', fontsize=9)
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Statistics bar chart
    ax4 = axes[1, 1]
    metrics = ['Std Dev (°)', 'Range (°)']
    acc_values = [acc_stats['std'], acc_stats['range']]
    kalman_values = [kalman_stats['std'], kalman_stats['range']]
    comp_values = [comp_stats['std'], comp_stats['range']]
    
    x_pos = np.arange(len(metrics))
    width = 0.25
    
    bars1 = ax4.bar(x_pos - width, acc_values, width, label='Accelerometer', color=color_acc, alpha=0.7)
    bars2 = ax4.bar(x_pos, kalman_values, width, label='Kalman', color=color_kalman, alpha=0.7)
    bars3 = ax4.bar(x_pos + width, comp_values, width, label='Complementary', color=color_comp, alpha=0.7)
    
    # Add value labels on bars
    for bars, vals in [(bars1, acc_values), (bars2, kalman_values), (bars3, comp_values)]:
        for bar, val in zip(bars, vals):
            ax4.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01, f'{val:.3f}', 
                     ha='center', va='bottom', fontsize=8)
    
    ax4.set_ylabel('Value')
    ax4.set_title('Noise Metrics Comparison')
    ax4.set_xticks(x_pos)
    ax4.set_xticklabels(metrics)
    ax4.legend(loc='upper right', fontsize=9)
    ax4.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    output_path = '../img/filter_comparison.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\nPlot saved to: {output_path}")
    plt.show()
