# VLM Consistency Test

**Date:** 2026-06-08  
**Image:** `/tmp/vlm_capture.jpg`  
**Model:** gemini-2.5-flash  
**N per test:** 20  

## Test 1 — Auto-detect (object name)

Prompt: *"What is this object? Reply with ONLY a 2-4 word description."*

| Response | Count | % |
|---|---|---|
| red rubber eraser | 17 | 85% |
| red block eraser | 2 | 10% |
| red cube eraser | 1 | 5% |

**Consistency: 85% on most common answer "red rubber eraser"**

## Test 2 — Grasp strategy

Object: `red rubber eraser`  
| Strategy | Count | % |
|---|---|---|
| short_side | 20 | 100% |

**Consistency: 100% on `short_side`**

## Test 3 — DeliGrasp params

Object: `red rubber eraser`  
Parsed OK: 20/20  

| Parameter | Mean | Std Dev | CV (%) |
|---|---|---|---|
| mass_g (g) | 20.75 | 2.45 | 11.8% |
| k (N/m) | 258.00 | 203.94 | 79.0% |
| mu () | 0.75 | 0.07 | 9.2% |
| aperture_mm (mm) | 26.60 | 5.66 | 21.3% |

*CV = coefficient of variation; lower = more consistent*
