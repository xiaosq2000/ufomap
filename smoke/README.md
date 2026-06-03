# UFOMap v2 — smoke build

A minimal, reproducible, **CPU-only** build that compiles the UFOMap v2 `Map`
component against its sibling UFO dependencies and runs a few Catch2 smoke
tests. It exists so the v2 maps can be exercised without the full UFO
superproject and without the GPU/WebGPU stack.

## Run it

From the repository root, with [pixi](https://pixi.sh) installed:

```sh
pixi run smoke      # configure + build + ctest in one shot
# or step by step:
pixi run configure
pixi run build
pixi run test
```

The pixi environment (see `../pixi.toml`) pulls only: a C++ compiler, CMake,
Ninja, Catch2, TBB (the `<execution>` backend) and lz4. The sibling UFO
components are fetched by CMake (`FetchContent`), pinned to the commits that
match v2's era (June 2025).

## How it works

- The `Map` component normally builds inside a parent UFO superproject. This
  harness (`smoke/CMakeLists.txt`) stands in for that parent: it downloads the
  header-only sibling components (without configuring them) and compiles the
  tests in `../tests` directly.
- **GPU is compiled out.** `ufocontainer`'s `TreeData` unconditionally pulls in
  `ufocompute`, which force-defines `UFO_WEBGPU` and includes
  `<webgpu/webgpu.h>`. The shim in `../compat/no_gpu/ufo/compute/compute.hpp`
  shadows that header so `UFO_WEBGPU` stays undefined and every
  `#if defined(UFO_WEBGPU)` block (in `TreeData` and in the UFO maps) drops out.
  No WebGPU headers or `libwgpu_native` are required. To build _with_ GPU,
  remove `compat/no_gpu` from the include path and provide the WebGPU stack.

## What is tested

| Test                 | Status   | Notes                                                                      |
| -------------------- | -------- | -------------------------------------------------------------------------- |
| `occupancy_map_test` | ✅ real  | occupancy update, logit↔probability conversion, `contains*`, `propagate()` |
| `labels_map_test`    | ✅ real  | label set/update, ALL vs SUMMARY propagation                               |
| `integration_test`   | ✅ real  | integrates a point cloud: occupied hits + ray-cast free space + unknown    |

## Known unfinished (v2)

The base **point integrator now works**: `Integrator::operator()` / `insertPoints()`
insert occupied hits and carve free space by ray-casting from the sensor origin
(sequential and execution-policy overloads). The miss ray-cast uses uniform
sampling — simple and correct, but an exact DDA traversal would be faster and
leak-free; that is the obvious next optimization.

The more sophisticated `AngularIntegrator` (a derived, sensor-error-aware variant)
is still an orphaned work-in-progress: its helpers (`fillMisses`, `fillData`, …)
are TODO and its sequential `operator()` has a `transform` typo. It is not
included anywhere and does not affect this build.

Also stale / out of the build (v1 leftovers): `apps/`, `src/render*.cpp`,
`tests/map_test.cpp` (references the deleted `integrator/detail/inverse/`),
`include/ufo/map/integration_old/`, and the `semantic*/surfel/labels_map`
variants that still `#include <ufo/util/...>` / `<ufo/pcl/...>`.
