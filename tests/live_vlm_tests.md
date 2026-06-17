# Live Vision VLM Tests

This document tracks the results of dynamic object recognition tests using live RGB frames from the robot's camera (`/camera/gripper_camera/camera/color/image_raw`) passed to Gemini 2.5 Flash for physical prior extraction.

## Object 1: Measuring Tape
* **Recognition:** Stanley measuring tape
* **Mass:** 450.0 g
* **Spring Constant (Stiffness):** 500,000 N/m
* **Friction Coefficient:** 0.5
* **Grasp Strategy:** "In accordance with the user instruction, this grasp should be a robust parallel-jaw grasp on the main body of the Stanley measuring tape, applying sufficient force to secure and lift the object. The gripper jaws would contact the rigid, often rubberized, sides of the tape measure, leveraging the material properties for enhanced friction and stability."

## Object 2: Water Bottle
* **Recognition:** Cylindrical body of a water bottle
* **Mass:** 515.0 g
* **Spring Constant (Stiffness):** 1,000 N/m
* **Friction Coefficient:** 0.6
* **Grasp Strategy:** "In accordance with the user instruction, this grasp should be a standard parallel-jaw grasp on the cylindrical body of the water bottle, applying sufficient force for stable lifting and manipulation using force control."

## Object 3: Orange Metal Component
* **Recognition:** Orange plastic body of a current sensor/shunt module
* **Mass:** 40.0 g
* **Spring Constant (Stiffness):** 100,000 N/m
* **Friction Coefficient:** 0.6
* **Grasp Strategy:** "The robot gripper will approach the object from above, centering its aperture over the orange plastic body of the current sensor/shunt module. The gripper will then close its fingers to grasp the object firmly on its flatter, longer sides, applying a controlled force to ensure a stable hold without causing damage. The target grasping width is approximately 25-30mm."
