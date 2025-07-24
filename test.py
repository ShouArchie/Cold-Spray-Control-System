import time
import sys
import math
import urx

# Import our modules
import robot_functions as rf

# Configuration
ROBOT_IP = "192.168.0.101"

def test_conical_motion():
    """Test the conical motion function with safety checks."""
    
    print("=" * 60)
    print("CONICAL MOTION TEST")
    print("=" * 60)
    
    # Connect to robot
    try:
        robot = urx.Robot(ROBOT_IP)
        print(f"✓ Connected to UR10 at {ROBOT_IP}")
    except Exception as e:
        print(f"✗ Failed to connect to robot: {e}")
        return
    
    try:
        # Define home position
        HOMETest = [math.radians(a) for a in [201.64, -54.86, 109.85, 214.57, 269.77, 111.69]]
        
        # Move to home position
        print("\n1. Moving to home position...")
        rf.move_to_joint_position(robot, HOMETest, acc=0.1, vel=0.1)
        
        # Get current TCP pose for reference
        current_pose = rf.get_tcp_pose(robot)
        print(f"   Current TCP: [{current_pose[0]:.3f}, {current_pose[1]:.3f}, {current_pose[2]:.3f}]")
        
        # Wait for user confirmation
        print("\n2. Ready to start conical motion test.")
        print("   The TCP will stay FIXED while the tool traces a cone.")
        input("   Press ENTER to continue or Ctrl+C to abort...")
        
        # Test parameters
        test_configs = [
            {
                "name": "Small Cone (15°)",
                "max_angle": 15.0,
                "num_points": 16,
                "acc": 0.3,
                "vel": 0.05,
                "blend_r": 0.005
            },
            {
                "name": "Medium Cone (30°)", 
                "max_angle": 30.0,
                "num_points": 24,
                "acc": 0.5,
                "vel": 0.1,
                "blend_r": 0.01
            },
            {
                "name": "Large Cone (45°)",
                "max_angle": 45.0,
                "num_points": 32,
                "acc": 0.3,
                "vel": 0.08,
                "blend_r": 0.015
            }
        ]
        
        for i, config in enumerate(test_configs, 1):
            print(f"\n3.{i} Testing: {config['name']}")
            print(f"     Max angle: {config['max_angle']}°")
            print(f"     Points: {config['num_points']}")
            print(f"     Blend radius: {config['blend_r']}")
            
            # Ask for confirmation for each test
            response = input(f"     Run this test? (y/n/q): ").lower()
            if response == 'q':
                break
            elif response != 'y':
                continue
                
            # Execute conical motion
            start_time = time.time()
            rf.conical_motion_fixed_tcp(
                robot,
                max_angle=config['max_angle'],
                num_points=config['num_points'],
                tcp_offset_z=60.3,  # Your tool length
                acc=config['acc'],
                vel=config['vel'],
                blend_r=config['blend_r']
            )
            end_time = time.time()
            
            print(f"     ✓ Completed in {end_time - start_time:.1f} seconds")
            
            # Verify TCP position hasn't changed
            final_pose = rf.get_tcp_pose(robot)
            tcp_drift = [
                abs(final_pose[i] - current_pose[i]) for i in range(3)
            ]
            max_drift = max(tcp_drift)
            print(f"     TCP drift: {max_drift*1000:.2f} mm")
            
            if max_drift > 0.001:  # 1mm tolerance
                print(f"     ⚠ Warning: TCP drift exceeds 1mm!")
            
            time.sleep(2)  # Brief pause between tests
        
        # Return to home
        print("\n4. Returning to home position...")
        rf.move_to_joint_position(robot, HOMETest, acc=0.1, vel=0.1)
        
        print("\n✓ All tests completed successfully!")
        
    except KeyboardInterrupt:
        print("\n⚠ Test interrupted by user")
    except Exception as e:
        print(f"\n✗ Test failed: {e}")
    finally:
        print("\n5. Stopping motion and closing connection...")
        rf.stop_linear(robot)
        robot.close()
        print("✓ Disconnected safely")

if __name__ == "__main__":
    test_conical_motion() 