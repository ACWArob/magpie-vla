# VLM Consistency Test

**Date:** 2026-06-08 12:38  
**Image:** `/tmp/vlm_capture.jpg`  
**Model:** gemini-2.5-flash  
**N per test:** 20  

## Test 1 — Auto-detect (object name)

Prompt: *"What is this object? Reply with ONLY a 2-4 word description."*

| Response | Count | % |
|---|---|---|
| red eraser | 14 | 70% |
| red cube | 3 | 15% |
| red cube eraser | 2 | 10% |
| red eraser cube | 1 | 5% |

**Consistency: 70% on most common answer "red eraser"**

## Test 2 — Grasp strategy

Object: `red eraser`  
| Strategy | Count | % |
|---|---|---|
| short_side | 18 | 90% |
| long_side | 1 | 5% |
| other:the | 1 | 5% |

**Consistency: 90% on `short_side`**

## Test 3 — DeliGrasp params

Object: `red eraser`  
Parsed OK: 20/20  

| Parameter | Mean | Std Dev | CV (%) |
|---|---|---|---|
| mass_g (g) | 20.00 | 0.00 | 0.0% |
| k (N/m) | 410.00 | 169.05 | 41.2% |
| mu () | 0.73 | 0.07 | 9.1% |
| aperture_mm (mm) | 27.95 | 6.34 | 22.7% |

*CV = coefficient of variation; lower = more consistent*
