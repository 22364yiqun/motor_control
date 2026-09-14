# Mechanical Design

The first mechanical release contains neutral STEP models for the 4310 actuator, reducer components, motor references, controller PCB, and second-encoder assembly. Editable SolidWorks source, dimensioned drawings, tolerances, materials, and manufacturing notes remain to be released.

## STEP Layout

- [`step/assembly/`](step/assembly/): complete actuator and second-encoder assemblies.
- [`step/components/`](step/components/): general component exports from the top-level CAD package.
- [`step/manufacturing/`](step/manufacturing/): part exports grouped by the author as machining STEP files.
- [`step/reference/`](step/reference/): motor, magnetic-ring rotor, and controller-PCB reference models.

The English filenames are repository-friendly aliases. [`step/MANIFEST.md`](step/MANIFEST.md) records the original Chinese filename for traceability.

> [!IMPORTANT]
> These files are design exports, not released manufacturing drawings. Verify units, materials, fits, bearing seats, gear geometry, clearances, magnet orientation, and the matching PCB revision before fabrication. The repository does not yet provide controlled tolerances or a released mechanical revision number.
