# MXIX
Code for our 2026 robot, Marvin 19

- An additional project will be created to generate a TunerConstants file for the actual robot

---

# Swerve Module Rezeroing Guide 

This guide is intended to help us recalculate swerve zeroes and deploy them onto the robot as fast as possible.

1. Set the module’s encoder offset in `TunerConstants` to `0.0`
   → Deploy code

2. Align the module to the **zero position**
   → Bevel gear aligned
   → Wheel straight forward

3. Read the **absolute encoder value** from NetworkTables / Elastic
   → ex. `DogLogSwerve/FLabsEncoderPos`

4. Compute the offset based on this value, from the negative of the current encoder value:

   ```
   offset = -(absolute encoder reading)
   ```

5. Update the offset in `TunerConstants`

6. Deploy code again

7. Verify drive when necessary

Time: 
- 2 code deployments (20-60 seconds)
- Bevel alignment, Setting offset to 0 in code, reading offset from dashboard: (10-20 seconds)
---
