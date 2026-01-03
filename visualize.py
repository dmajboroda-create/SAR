import sys
import os
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')  # Headless mode (no GUI)

def main():
    """Main visualization function"""

    # Check input file existence
    input_file = 'simulation_results.csv'
    if not os.path.exists(input_file):
        print(f"✗ Error: File '{input_file}' not found!")
        print("   Run the simulation program first.")
        sys.exit(1)

    try:
        # Read data from CSV
        df = pd.read_csv(input_file, sep=';', encoding='utf-8')

        # Clean column names from spaces
        df.columns = df.columns.str.strip()

        # Check for required columns
        required_columns = ['t', 'x', 'x_d', 'x_dd', 'x_ddd', 'F']
        missing_columns = [col for col in required_columns if col not in df.columns]

        if missing_columns:
            print(f"✗ Error: Missing columns in file: {missing_columns}")
            print(f"   Available columns: {df.columns.tolist()}")
            sys.exit(1)

        print(f"✓ Loaded {len(df)} data points")

    except Exception as e:
        print(f"✗ Error reading file: {e}")
        sys.exit(1)

    try:
        # Create figure with subplots
        fig, axes = plt.subplots(3, 2, figsize=(14, 10))
        fig.suptitle('Solution of Equation (1) for Aircraft Engine Control System\n4th Order Runge-Kutta Method',
                     fontsize=14, fontweight='bold')

        # Plot 1: x(t) - engine revolutions
        axes[0, 0].plot(df['t'], df['x'], 'b-', linewidth=1.5)
        axes[0, 0].set_xlabel('Time t (s)', fontsize=10)
        axes[0, 0].set_ylabel('x(t) - revolutions (rev/s)', fontsize=10)
        axes[0, 0].set_title('Engine Revolutions')
        axes[0, 0].grid(True, alpha=0.3)

        # Plot 2: x'(t) - rate of change of revolutions
        axes[0, 1].plot(df['t'], df['x_d'], 'g-', linewidth=1.5)
        axes[0, 1].set_xlabel('Time t (s)', fontsize=10)
        axes[0, 1].set_ylabel('x\'(t) (rev/s²)', fontsize=10)
        axes[0, 1].set_title('First Derivative (Rate of Change)')
        axes[0, 1].grid(True, alpha=0.3)

        # Plot 3: x''(t) - acceleration
        axes[1, 0].plot(df['t'], df['x_dd'], 'r-', linewidth=1.5)
        axes[1, 0].set_xlabel('Time t (s)', fontsize=10)
        axes[1, 0].set_ylabel('x\'\'(t) (rev/s³)', fontsize=10)
        axes[1, 0].set_title('Second Derivative (Acceleration)')
        axes[1, 0].grid(True, alpha=0.3)

        # Plot 4: x'''(t) - third derivative
        axes[1, 1].plot(df['t'], df['x_ddd'], 'm-', linewidth=1.5)
        axes[1, 1].set_xlabel('Time t (s)', fontsize=10)
        axes[1, 1].set_ylabel('x\'\'\'(t) (rev/s⁴)', fontsize=10)
        axes[1, 1].set_title('Third Derivative')
        axes[1, 1].grid(True, alpha=0.3)

        # Plot 5: F(t) - disturbance
        axes[2, 0].plot(df['t'], df['F'], 'k-', linewidth=1.5)
        axes[2, 0].set_xlabel('Time t (s)', fontsize=10)
        axes[2, 0].set_ylabel('F(t)', fontsize=10)
        axes[2, 0].set_title('External Disturbance F(t)')
        axes[2, 0].grid(True, alpha=0.3)

        # Plot 6: x(t) and F(t) together (two Y axes)
        ax1 = axes[2, 1]
        ax2 = ax1.twinx()

        line1 = ax1.plot(df['t'], df['x'], 'b-', linewidth=1.5, label='x(t) - revolutions')
        line2 = ax2.plot(df['t'], df['F'], 'k--', linewidth=1.5, label='F(t) - disturbance')

        ax1.set_xlabel('Time t (s)', fontsize=10)
        ax1.set_ylabel('x(t) (rev/s)', fontsize=10, color='b')
        ax2.set_ylabel('F(t)', fontsize=10, color='k')
        ax1.tick_params(axis='y', labelcolor='b')
        ax2.tick_params(axis='y', labelcolor='k')
        axes[2, 1].set_title('System Response to Disturbance')
        ax1.grid(True, alpha=0.3)

        # Legend
        lines = line1 + line2
        labels = [l.get_label() for l in lines]
        ax1.legend(lines, labels, loc='upper right', fontsize=9)

        plt.tight_layout()

        # Save plots
        output_path = 'simulation_results.png'
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"✓ Plots saved to file: {output_path}")

        # Statistics
        print("\n=== SOLUTION STATISTICS ===")
        print(f"Number of points:     {len(df)}")
        print(f"Time interval:        [{df['t'].min():.2f}, {df['t'].max():.2f}] s")
        print(f"\nValues of x(t) (revolutions):")
        print(f"  Minimum:            {df['x'].min():.6f} rev/s")
        print(f"  Maximum:            {df['x'].max():.6f} rev/s")
        print(f"  Initial:            {df['x'].iloc[0]:.6f} rev/s")
        print(f"  Final:              {df['x'].iloc[-1]:.6f} rev/s")
        print(f"\nValues of F(t) (disturbance):")
        print(f"  Initial:            {df['F'].iloc[0]:.6f}")
        print(f"  Final:              {df['F'].iloc[-1]:.6f}")
        print(f"  Decrease:           {(1 - df['F'].iloc[-1]/df['F'].iloc[0])*100:.1f}%")
        print("=" * 40)

        return 0

    except Exception as e:
        print(f"✗ Error creating plots: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    sys.exit(main())