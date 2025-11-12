"""
Visual demonstration of the 2D scanning concept.
Run this to see how the scanning works conceptually.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch
from matplotlib.animation import FuncAnimation

def create_concept_diagram():
    """Create a static diagram explaining the concept."""
    fig = plt.figure(figsize=(14, 10))
    
    # Diagram 1: Side view showing height compensation
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.set_title('Side View: Height Compensation Concept', fontsize=14, fontweight='bold')
    ax1.set_xlim(-5, 105)
    ax1.set_ylim(0, 120)
    ax1.set_xlabel('Y Position (gantry movement)')
    ax1.set_ylabel('Z Height')
    ax1.grid(True, alpha=0.3)
    
    # Draw gantry at two positions
    gantry_y_positions = [20, 60]
    gantry_z_positions = [100, 80]
    
    for i, (gy, gz) in enumerate(zip(gantry_y_positions, gantry_z_positions)):
        # Gantry/laser
        color = 'blue' if i == 0 else 'green'
        ax1.add_patch(Rectangle((gy-3, gz), 6, 8, color=color, alpha=0.6))
        ax1.plot([gy], [gz-2], 'v', color=color, markersize=15, label=f'Laser at Z={gz}')
        
        # Laser beam
        ax1.plot([gy, gy], [gz-2, 30], '--', color=color, alpha=0.5)
        
        # Object
        ax1.add_patch(Rectangle((gy-5, 20), 10, 10, color='brown', alpha=0.8))
        
        # Measurements
        ax1.annotate('', xy=(gy+8, gz-2), xytext=(gy+8, 30),
                    arrowprops=dict(arrowstyle='<->', color='red', lw=2))
        ax1.text(gy+12, (gz-2+30)/2, f'Laser reads\n{gz-30} mm', fontsize=9, color='red')
        
        ax1.annotate('', xy=(gy+20, 30), xytext=(gy+20, 20),
                    arrowprops=dict(arrowstyle='<->', color='purple', lw=2))
        ax1.text(gy+23, 25, 'Object\n10 mm', fontsize=9, color='purple')
    
    # Ground
    ax1.plot([0, 100], [20, 20], 'k-', lw=2)
    ax1.text(50, 15, 'Ground / Table', ha='center', fontsize=10)
    
    ax1.legend(loc='upper right')
    
    # Diagram 2: Top view showing X-Y scanning
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.set_title('Top View: 2D Scanning Pattern', fontsize=14, fontweight='bold')
    ax2.set_xlim(-50, 50)
    ax2.set_ylim(-10, 110)
    ax2.set_xlabel('X Position (laser scan line)')
    ax2.set_ylabel('Y Position (gantry movement)')
    ax2.grid(True, alpha=0.3)
    
    # Object to scan
    object_x = [-20, 20, 20, -20, -20]
    object_y = [20, 20, 80, 80, 20]
    ax2.fill(object_x, object_y, color='lightblue', alpha=0.6, label='Object')
    ax2.plot(object_x, object_y, 'b-', lw=2)
    
    # Laser scan lines at different Y positions
    for y in range(10, 100, 15):
        alpha = 0.3 + (y/100) * 0.5
        ax2.plot([-40, 40], [y, y], 'r-', alpha=alpha, lw=1)
        ax2.plot([0], [y], 'ro', markersize=3)
    
    # Gantry path
    ax2.arrow(0, 0, 0, 95, head_width=4, head_length=3, fc='green', ec='green', lw=2)
    ax2.text(5, 50, 'Gantry\nmovement', fontsize=10, color='green')
    
    # Laser span
    ax2.arrow(-35, 5, 70, 0, head_width=2, head_length=3, fc='red', ec='red', lw=1.5)
    ax2.text(0, -5, 'Laser scan line', ha='center', fontsize=10, color='red')
    
    ax2.legend(loc='upper right')
    
    # Diagram 3: Formula explanation
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.axis('off')
    ax3.set_xlim(0, 10)
    ax3.set_ylim(0, 10)
    
    # Title
    ax3.text(5, 9, 'Height Compensation Formula', ha='center', fontsize=14, fontweight='bold')
    
    # Formula
    formula_text = [
        '',
        'Problem:',
        '  • Laser measures distance to object',
        '  • If laser moves up/down, reading changes',
        '  • But object height stays the same!',
        '',
        'Solution:',
        '  Object Height = Laser Reading - Laser Height',
        '',
        'Example:',
        '  Laser at Z=100mm, reads 70mm → Object at 30mm',
        '  Laser at Z=80mm, reads 50mm → Object still at 30mm',
        '  ✓ Compensated height: 10mm (constant!)',
        '',
        'In code:',
        '  compensated_z = laser_z - gantry_z',
    ]
    
    y_pos = 7.5
    for line in formula_text:
        if line.startswith('Problem:') or line.startswith('Solution:') or line.startswith('Example:') or line.startswith('In code:'):
            ax3.text(1, y_pos, line, fontsize=11, fontweight='bold')
        else:
            ax3.text(1, y_pos, line, fontsize=10, family='monospace')
        y_pos -= 0.5
    
    # Diagram 4: Result visualization
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.set_title('Result: 2D Height Map', fontsize=14, fontweight='bold')
    
    # Create sample data
    x = np.linspace(-30, 30, 100)
    y = np.linspace(0, 100, 100)
    X, Y = np.meshgrid(x, y)
    
    # Create height map with some features
    Z = 10 + 5 * np.sin(X/10) * np.cos(Y/20)
    Z[40:60, 30:70] += 8  # Raised platform
    
    im = ax4.imshow(Z, extent=[-30, 30, 0, 100], origin='lower', 
                    cmap='viridis', aspect='auto')
    ax4.set_xlabel('X Position (mm)')
    ax4.set_ylabel('Y Position (mm)')
    
    cbar = plt.colorbar(im, ax=ax4)
    cbar.set_label('Height (mm)')
    
    # Add annotations
    ax4.annotate('Raised\nArea', xy=(0, 50), xytext=(20, 70),
                arrowprops=dict(arrowstyle='->', color='white', lw=2),
                color='white', fontsize=10, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig('scanning_concept_diagram.png', dpi=150, bbox_inches='tight')
    print("Diagram saved as: scanning_concept_diagram.png")
    plt.show()


def create_animation():
    """Create an animation showing the scanning process."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    
    # Setup ax1 - side view
    ax1.set_xlim(-10, 110)
    ax1.set_ylim(0, 120)
    ax1.set_xlabel('Y Position')
    ax1.set_ylabel('Z Height')
    ax1.set_title('Side View: Laser Scanning')
    ax1.grid(True, alpha=0.3)
    
    # Ground
    ax1.plot([0, 100], [20, 20], 'k-', lw=2)
    
    # Object (with varying height)
    object_profile = 20 + 10 + 5*np.sin(np.linspace(0, 2*np.pi, 100))
    ax1.fill_between(range(100), 20, object_profile, color='brown', alpha=0.6)
    
    # Laser and beam
    laser, = ax1.plot([], [], 'rv', markersize=15)
    beam, = ax1.plot([], [], 'r--', alpha=0.5)
    
    # Setup ax2 - top view (2D map)
    ax2.set_xlim(-30, 30)
    ax2.set_ylim(0, 100)
    ax2.set_xlabel('X Position (mm)')
    ax2.set_ylabel('Y Position (mm)')
    ax2.set_title('Top View: Height Map Building')
    
    scan_data = np.zeros((100, 60))
    im = ax2.imshow(scan_data, extent=[-30, 30, 0, 100], origin='lower',
                    cmap='viridis', vmin=0, vmax=20, aspect='auto')
    
    plt.colorbar(im, ax=ax2, label='Height (mm)')
    
    # Gantry Z position (moves up and down)
    def gantry_z(y):
        return 90 + 10*np.sin(y/15)  # Oscillates between 80-100
    
    def init():
        laser.set_data([], [])
        beam.set_data([], [])
        return laser, beam, im
    
    def animate(frame):
        y = frame
        if y >= 100:
            return laser, beam, im
        
        gz = gantry_z(y)
        
        # Update laser position
        laser.set_data([y], [gz])
        beam.set_data([y, y], [gz, object_profile[y]])
        
        # Simulate laser scan line (X-axis)
        laser_reading = object_profile[y]
        compensated_height = laser_reading - 20  # Ground is at 20
        
        # Add to scan data
        x_samples = 60
        for xi in range(x_samples):
            # Add some noise and variation across X
            noise = np.random.normal(0, 0.5)
            scan_data[y, xi] = max(0, compensated_height + noise)
        
        im.set_data(scan_data)
        
        return laser, beam, im
    
    anim = FuncAnimation(fig, animate, init_func=init, frames=100,
                        interval=50, blit=True, repeat=True)
    
    plt.tight_layout()
    print("Animation running... Close window when done.")
    plt.show()


def main():
    print("=" * 60)
    print("2D LASER SCANNER - VISUAL CONCEPT DEMONSTRATION")
    print("=" * 60)
    print("\n1. Show concept diagram (static)")
    print("2. Show scanning animation")
    print("3. Show both")
    print("0. Exit")
    
    choice = input("\nEnter choice: ").strip()
    
    if choice == '1':
        create_concept_diagram()
    elif choice == '2':
        create_animation()
    elif choice == '3':
        create_concept_diagram()
        create_animation()
    elif choice == '0':
        print("Exiting...")
    else:
        print("Invalid choice")


if __name__ == "__main__":
    main()
