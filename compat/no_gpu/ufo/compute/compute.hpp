/*!
 * UFOMap smoke-build compatibility shim.
 *
 * The real <ufo/compute/compute.hpp> (from the ufocompute dependency)
 * unconditionally `#define UFO_WEBGPU` and `#include <webgpu/webgpu.h>`, which
 * pulls the whole WebGPU / wgpu-native stack into every translation unit via
 * ufocontainer's TreeData. For a minimal CPU-only build we shadow that header
 * with this empty one (placed first on the include path). Because UFO_WEBGPU is
 * left undefined, all `#if defined(UFO_WEBGPU)` GPU code in TreeData and in the
 * UFO maps is compiled out — no GPU headers or libraries required.
 *
 * To build WITH GPU support, drop this directory from the include path and
 * provide webgpu/webgpu.h + libwgpu_native instead.
 */
#ifndef UFO_COMPUTE_COMPUTE_HPP
#define UFO_COMPUTE_COMPUTE_HPP
#endif  // UFO_COMPUTE_COMPUTE_HPP
