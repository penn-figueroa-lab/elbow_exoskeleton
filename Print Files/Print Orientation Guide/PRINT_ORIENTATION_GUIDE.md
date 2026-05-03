# 🖨️ Print Guide — Elbow Exoskeleton

This guide covers the print setup for all printed parts of the elbow exoskeleton. All Bambu Studio `.3mf` project files are configured for the **Bambu Lab P1S** and are available in `Print Files/BambuStudio/`. It is recommended to view prints in **Line Type** color scheme in Bambu Studio to visualize support structures vs. actual print filament before printing.

---

## Print Settings Summary

| Part | Material | Infill | Wall Loops |
|------|----------|--------|------------|
| Forearm linkage assembly | PLA | 75% | 4 |
| Upper arm linkage - Left/Right | PLA | 75% | 4 |
| Forearm orthotic structural insert | PLA | 75% | 4 |
| Upper arm orthotic structural insert | PLA | 75% | 4 |
| Forearm orthotics (all sizes, L/R) | TPU | 15% | 4 |
| Upper arm orthotics (all sizes, L/R) | TPU | 15% | 4 |
| Mechanical stop cover | TPU | 15% | 4 |

> **Bed adhesion tip:** If parts are not sticking to the bed well, add a brim in Bambu Studio before slicing.

---

## Print Beds

Parts are grouped into 4 print beds — 2 PLA beds and 2 TPU beds, separated by left and right arm versions. Note that the forearm linkage assembly, structural inserts, and mechanical stop covers are universal (identical for both arms) but are included in their respective bed for convenience.

> **Support removal:** Some parts require support removal after printing. TPU support removal in particular can be a long and grueling process — budget extra time for this. Surface finish on TPU orthotics may not be perfect straight off the printer; consider splitting the orthotic print into two separate prints or sanding the surface down after printing for a cleaner finish.

---

### Bed 1 — Left Arm PLA Parts
*Forearm linkage assembly (universal) + Upper arm linkage Left + Forearm orthotic structural insert (universal) + Upper arm orthotic structural insert (universal)*

**File:** `Elbow_Exoskeleton_Left_Linkage_&_Inserts_Prints.3mf`

![Left PLA prints](screenshots/Elbow_Exoskeleton_Left_Linkage_&_Inserts_Prints.png)

---

### Bed 2 — Left Arm TPU Parts
*Forearm orthotic R35 Left + Upper arm orthotic R45 Left + Mechanical stop cover x2 (universal)*

**File:** `Elbow_Exoskeleton_Left_Orthotics_&_Mechanical_Stop.3mf`

![Left TPU prints](screenshots/Elbow_Exoskeleton_Left_Orthotics_&_Mechanical_Stop.png)

---

### Bed 3 — Right Arm PLA Parts
*Forearm linkage assembly (universal) + Upper arm linkage Right + Forearm orthotic structural insert (universal) + Upper arm orthotic structural insert (universal)*

**File:** `Elbow_Exoskeleton_Right_Linkage_&_Inserts_Prints.3mf`

![Right PLA prints](screenshots/Elbow_Exoskeleton_Right_Linkage_&_Inserts_Prints.png)

---

### Bed 4 — Right Arm TPU Parts
*Forearm orthotic R35 Right + Upper arm orthotic R45 Right + Mechanical stop cover x2 (universal)*

**File:** `Elbow_Exoskeleton_Right_Orthotics_&_Mechanical_Stop.3mf`

![Right TPU prints](screenshots/Elbow_Exoskeleton_Right_Orthotics_&_Mechanical_Stop.png)

---

## Forearm Linkage — Snap-Fit Interior Cross-Section

The forearm linkage assembly is a two-piece print-in-place snap-fit design. The interior geometry is specifically designed so that **no supports are generated inside the snap-fit channel or along the sliding interface**, keeping the sliding surfaces clean and functional straight off the printer.

The cross-section below shows the internal geometry in Bambu Studio Line Type view.

![Forearm linkage snap-fit cross-section](screenshots/Elbow_Exoskeleton_Forearm_Linkage_Closeup.png)

---

[⬆ Back to Main README](../../README.md)
