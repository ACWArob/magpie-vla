#!/usr/bin/env python3
import subprocess
import re
import os

TOPICS = {
    "Gripper Motor": ("/gripper/state", 5.0),
    "Force Torque Sensor": ("/ft_sensor/wrench", 250.0),
    "Arm Position": ("/arm/tcp_pose", 500.0),
    "Camera": ("/camera/gripper_camera/camera/color/image_raw", 10.0)
}

def measure_hz(topic, duration=5):
    print(f"Measuring {topic} for {duration} seconds...")
    # We use 'timeout' to let 'ros2 topic hz' run for the specified duration and then terminate it
    try:
        result = subprocess.run(
            ['timeout', str(duration), 'ros2', 'topic', 'hz', topic], 
            capture_output=True, 
            text=True
        )
        stdout = result.stdout
    except Exception as e:
        print(f"Error measuring {topic}: {e}")
        return 0.0

    # Find all occurrences of the average rate being printed
    rates = re.findall(r'average rate:\s+([\d.]+)', stdout)
    if rates:
        return float(rates[-1])
    return 0.0

if __name__ == "__main__":
    results = []
    print("--- Measuring Sensor Poll Rates ---")
    print("Make sure the full ROS stack is running before executing this script.\n")
    
    for name, (topic, expected) in TOPICS.items():
        rate = measure_hz(topic)
        results.append((name, topic, expected, rate))
        print(f"{name}: Measured = {rate:.2f} Hz, Expected = {expected} Hz")

    os.makedirs("tests", exist_ok=True)
    with open("tests/poll_rates.md", "w") as f:
        f.write("# Sensor Poll Rates\n\n")
        f.write("| Sensor | Topic | Expected (Hz) | Measured (Hz) |\n")
        f.write("|---|---|---|---|\n")
        for name, topic, expected, rate in results:
            f.write(f"| {name} | `{topic}` | {expected} | {rate:.2f} |\n")
    print("\nResults successfully saved to tests/poll_rates.md")