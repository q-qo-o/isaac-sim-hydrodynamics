# Changelog

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] - 2026-02-24

### Added

- 6-DOF Hydrodynamics simulation (Linear Damping, Quadratic Damping, Added Mass).
- Real-time Buoyancy calculation based on mesh volume and water level.
- Dynamic Characteristic Length calculation using AABB.
- Custom Property Widget with grouped attributes.
- Bilingual documentation (README.md and README_CN.md).

### Changed

- Refactored parameter names from nautical terms (Surge/Sway/Heave) to directional terms (Forward/Lateral/Vertical).
- Improved UI grouping logic.

### Fixed

- Fixed bug where Property Widget group overrides were not applied correctly.
