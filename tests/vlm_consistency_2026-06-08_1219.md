# VLM Consistency Test

**Date:** 2026-06-08 12:19  
**Image:** `/tmp/vlm_capture.jpg`  
**Model:** gemini-2.5-flash  
**N per test:** 20  

## Test 1 — Auto-detect (object name)

Prompt: *"What is this object? Reply with ONLY a 2-4 word description."*

| Response | Count | % |
|---|---|---|
| red rubber eraser | 18 | 90% |
| pink rubber eraser | 1 | 5% |
| red cube | 1 | 5% |

**Consistency: 90% on most common answer "red rubber eraser"**

## Test 2 — Grasp strategy

Object: `red rubber eraser`  
| Strategy | Count | % |
|---|---|---|
| short_side | 17 | 85% |
| symmetric | 2 | 10% |
| long_side | 1 | 5% |

**Consistency: 85% on `short_side`**

## Test 3 — DeliGrasp params

Object: `red rubber eraser`  
Parsed OK: 20/20  

| Parameter | Mean | Std Dev | CV (%) |
|---|---|---|---|
| mass_g (g) | 19.75 | 1.12 | 5.7% |
| k (N/m) | 276.75 | 207.32 | 74.9% |
| mu () | 0.78 | 0.06 | 7.9% |
| aperture_mm (mm) | 24.30 | 8.63 | 35.5% |

*CV = coefficient of variation; lower = more consistent*
