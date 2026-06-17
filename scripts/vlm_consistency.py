#!/usr/bin/env python3
import os
import re
import time
import argparse
import numpy as np
import matplotlib.pyplot as plt
from google import genai
from google.genai import types

# The exact DeliGrasp Prompt copied from deligrasp_node.py to keep the script standalone
DG_DESCRIPTOR_PROMPT = """Control a robot gripper with force control and contact information. \
The gripper's parameters can be adjusted corresponding to the type of object that it is trying \
to grasp as well as the kind of grasp it is attempting to perform.
The gripper has a measurable max force of 16N and min force of 0.15N, a maximum aperture of \
105mm and a minimum aperture of 1mm.

Some grasps may be incomplete, intended for observing force information about a given object.
Describe the grasp strategy using the following form:

[start of description]
* This {CHOICE: [is, is not]} a new grasp.
* In accordance with the user instruction, this grasp should be [GRASP_DESCRIPTION: <str>].
* This is a {CHOICE: [complete, incomplete]} grasp.
* This grasp {CHOICE: [does, does not]} contain multiple grasps.
* This grasp is for an object with {CHOICE: [high, medium, low]} weight.
* The object has an approximate mass of [PNUM: 0.0] grams
* This grasp is for an object with {CHOICE: [high, medium, low]} compliance.
* The object has an approximate spring constant of [PNUM: 0.0] Newtons per meter.
* The gripper and object have an approximate friction coefficient of [PNUM: 0.0]
* This grasp should set the goal aperture to [PNUM: 0.0] mm.
* If the gripper slips, this grasp should close an additional [PNUM: 0.0] mm.
* If the gripper slips, this grasp should increase the output force by [PNUM: 0.0] Newtons.
* [optional] Because of [GRASP_DESCRIPTION: <str>], this grasp sets the force to be \
{CHOICE: [lower, higher]} than the default minimum grasp force.
[end of description]"""

def parse_descriptor(text):
    """Extract grasp fields from a DeliGrasp descriptor response."""
    m = re.search(r'\[start of description\](.*?)\[end of description\]', text, re.DOTALL | re.IGNORECASE)
    if not m:
        return None
    body = m.group(1)

    def _get(pattern):
        hit = re.search(pattern, body, re.IGNORECASE)
        return float(hit.group(1)) if hit else None

    return {
        'mass_grams': _get(r'approximate mass of ([0-9.]+) grams'),
        'spring_constant': _get(r'spring constant of ([0-9.]+) Newtons per meter'),
        'friction_coeff': _get(r'friction coefficient of ([0-9.]+)')
    }

def run_consistency_test(objects, n_queries, model_name):
    api_key = os.environ.get('GEMINI_API_KEY', '')
    if not api_key:
        print("ERROR: GEMINI_API_KEY not set in environment.")
        return

    client = genai.Client(api_key=api_key)
    results = {obj: {'mass': [], 'k': [], 'mu': []} for obj in objects}

    print(f"Running {n_queries} queries for {len(objects)} objects.")
    print(f"Using exactly timestamped model label: {model_name}")

    for obj in objects:
        print(f"\n--- Testing object: {obj} ---")
        for i in range(n_queries):
            user_message = f'Pick up the {obj}.'
            try:
                response = client.models.generate_content(
                    model=model_name,
                    contents=user_message,
                    config=types.GenerateContentConfig(
                        system_instruction=DG_DESCRIPTOR_PROMPT,
                        temperature=0.7  # Inject slightly higher temperature to properly test consistency variance
                    ),
                )
                parsed = parse_descriptor(response.text)
                if parsed and None not in parsed.values():
                    results[obj]['mass'].append(parsed['mass_grams'])
                    results[obj]['k'].append(parsed['spring_constant'])
                    results[obj]['mu'].append(parsed['friction_coeff'])
                    print(f"  [{i+1}/{n_queries}] Success: Mass={parsed['mass_grams']}g, k={parsed['spring_constant']} N/m, mu={parsed['friction_coeff']}")
                else:
                    print(f"  [{i+1}/{n_queries}] Failed to parse valid parameters.")
            except Exception as e:
                print(f"  [{i+1}/{n_queries}] API Error: {e}")
            
            # Rate limiting sleep
            time.sleep(2)

    # --- Plotting ---
    os.makedirs("tests", exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    
    # Create 3 side-by-side subplots
    fig, axs = plt.subplots(1, 3, figsize=(16, 6))
    fig.suptitle(f"VLM Physics Prior Consistency (n={n_queries} queries/obj)\nModel: {model_name} | Run Date: {timestamp}", fontsize=14)

    colors = ['#1f77b4', '#ff7f0e', '#2ca02c']  # Clean hex colors
    
    for idx, obj in enumerate(objects):
        # X-axis with standard jitter so dots don't completely overlap
        x_mass = np.random.normal(idx + 1, 0.05, len(results[obj]['mass']))
        x_k    = np.random.normal(idx + 1, 0.05, len(results[obj]['k']))
        x_mu   = np.random.normal(idx + 1, 0.05, len(results[obj]['mu']))
        
        axs[0].scatter(x_mass, results[obj]['mass'], color=colors[idx], label=obj, alpha=0.7, edgecolors='black')
        axs[1].scatter(x_k, results[obj]['k'], color=colors[idx], label=obj, alpha=0.7, edgecolors='black')
        axs[2].scatter(x_mu, results[obj]['mu'], color=colors[idx], label=obj, alpha=0.7, edgecolors='black')

    axs[0].set_title('Estimated Mass (grams)')
    axs[1].set_title('Spring Constant (k) [N/m]')
    axs[2].set_title('Friction Coefficient (\u03BC)')

    for ax in axs:
        ax.set_xticks(range(1, len(objects) + 1))
        ax.set_xticklabels([obj.title() for obj in objects])
        ax.grid(True, linestyle='--', alpha=0.5)
        ax.set_axisbelow(True)

    # Deduplicate legends
    handles, labels = axs[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', ncol=3, bbox_to_anchor=(0.5, -0.05))

    plot_path = f"tests/vlm_consistency_plot_{timestamp}.png"
    plt.tight_layout(rect=[0, 0.05, 1, 0.95])
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    print(f"\nScatter plots successfully saved to {plot_path}")

if __name__ == "__main__":
    # We use 3 diverse test objects to map out differing scale profiles
    objects_to_test = ["measuring tape", "water bottle", "orange metal component"]
    
    # Being specific about the timestamped model as requested by the mentor
    run_consistency_test(objects_to_test, 10, "gemini-2.5-flash")