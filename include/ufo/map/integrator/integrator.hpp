/*!
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author Daniel Duberg (dduberg@kth.se)
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * @version 1.0
 * @date 2022-05-13
 *
 * @copyright Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef UFO_MAP_INTEGRATOR_INTEGRATOR_HPP
#define UFO_MAP_INTEGRATOR_INTEGRATOR_HPP

// UFO
#include <ufo/cloud/point_cloud.hpp>
#include <ufo/container/tree/code.hpp>
#include <ufo/container/tree/coord.hpp>
#include <ufo/container/tree/index.hpp>
#include <ufo/core/label.hpp>
#include <ufo/execution/algorithm.hpp>
#include <ufo/execution/execution.hpp>
#include <ufo/map/color/map.hpp>
#include <ufo/map/occupancy/map.hpp>
// #include <ufo/map/time/map.hpp>
#include <ufo/map/integrator/count_sampling_method.hpp>
#include <ufo/map/integrator/detail/bool_grid.hpp>
#include <ufo/map/integrator/detail/count_grid.hpp>
#include <ufo/map/integrator/detail/grid_map.hpp>
#include <ufo/map/integrator/detail/hit.hpp>
#include <ufo/map/integrator/detail/hit_grid.hpp>
#include <ufo/map/integrator/detail/miss.hpp>
#include <ufo/map/integrator/detail/miss_grid.hpp>
#include <ufo/map/type.hpp>
#include <ufo/map/void_region/map.hpp>
#include <ufo/math/vec.hpp>
#include <ufo/utility/spinlock.hpp>
#include <ufo/utility/type_traits.hpp>

// STL
#include <cmath>
#include <cstddef>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace ufo
{
enum class DownSamplingMethod { NONE, FIRST, CENTER };

template <std::size_t Dim = 3>
class Integrator
{
 public:
	//
	// Tags
	//
	using occupancy_t = float;
	using logit_t     = OccupancyElement::logit_t;
	using depth_t     = unsigned;

	depth_t hit_depth  = 0;
	depth_t miss_depth = 0;

	occupancy_t occupancy_hit             = 0.75f;    // [0, 1]
	occupancy_t occupancy_miss            = 0.45f;    // [0, 1]
	logit_t     occupancy_max_clamp_thres = 0.971f;   // [0, 1]
	logit_t     occupancy_min_clamp_thres = 0.1192f;  // [0, 1]

	unsigned void_region_distance = 2;

	DownSamplingMethod sample_method = DownSamplingMethod::NONE;

	// TODO: Should this be here?
	bool free_hits = false;

	bool verbose = false;

	CountSamplingMethod count_sample_method = CountSamplingMethod::NONE;

	// A single hit in a void region will set the occupancy to max
	bool void_region_instant_max_occupancy = true;
	// A single miss in a void region will set the occupancy to min
	bool void_region_instant_min_occupancy = true;

	bool propagate      = false;
	bool prune          = true;
	bool reset_modified = true;

 public:
	/*!
	 * @brief Integrate a point cloud into `map` (sequential).
	 *
	 * Every point becomes an occupied "hit" and the free space between the sensor
	 * origin and each point is carved as "misses". `transform` is the sensor pose in
	 * the map frame: its translation is the sensor origin and its rotation brings the
	 * (sensor-frame) cloud into the map frame. A default-constructed `transform` is
	 * treated as identity -- the cloud is assumed to already be in the map frame with
	 * the sensor at the origin.
	 */
	template <class Map, class T, class... Rest>
	void operator()(Map& map, PointCloud<Dim, T, Rest...> cloud,
	                Transform<Dim, T> const& transform = {}) const
	{
		Vec<Dim, T> const sensor_origin = transform.translation;
		if (Transform<Dim, T>{} != transform) {
			transformInPlace(transform, cloud);
		}

		auto misses = buildMisses(map, cloud, Vec<Dim, float>(sensor_origin));

		insertHits(map, cloud);
		if (!misses.empty()) {
			insertMisses(map, misses);
		}

		if (propagate) {
			map.propagate();
		}
	}

	/*!
	 * @brief Integrate a point cloud into `map` using the given execution policy.
	 *
	 * @see operator()(Map&, PointCloud, Transform const&) const
	 */
	template <
	    class ExecutionPolicy, class Map, class T, class... Rest,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void operator()(ExecutionPolicy&& policy, Map& map, PointCloud<Dim, T, Rest...> cloud,
	                Transform<Dim, T> const& transform = {}) const
	{
		Vec<Dim, T> const sensor_origin = transform.translation;
		if (Transform<Dim, T>{} != transform) {
			transformInPlace(policy, transform, cloud);
		}

		auto misses = buildMisses(map, cloud, Vec<Dim, float>(sensor_origin));

		insertHits(policy, map, cloud);
		if (!misses.empty()) {
			insertMisses(policy, map, misses);
		}

		if (propagate) {
			map.propagate(policy);
		}
	}

	/*!
	 * @brief Named alias for `operator()` -- integrate a point cloud into `map`.
	 */
	template <class Map, class T, class... Rest>
	void insertPoints(Map& map, PointCloud<Dim, T, Rest...> cloud,
	                  Transform<Dim, T> const& transform = {}) const
	{
		(*this)(map, std::move(cloud), transform);
	}

	template <
	    class ExecutionPolicy, class Map, class T, class... Rest,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void insertPoints(ExecutionPolicy&& policy, Map& map, PointCloud<Dim, T, Rest...> cloud,
	                  Transform<Dim, T> const& transform = {}) const
	{
		(*this)(std::forward<ExecutionPolicy>(policy), map, std::move(cloud), transform);
	}

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                                        Hits                                         |
	|                                                                                     |
	**************************************************************************************/

	template <class Map, class Data>
	void insertHit(Map& map, TreeIndex const& node, Data const& data, logit_t occupancy,
	               logit_t occupancy_min, logit_t occupancy_max) const
	{
		if constexpr (Map::hasMapTypes(MapType::OCCUPANCY)) {
			if constexpr (Map::hasMapTypes(MapType::VOID_REGION)) {
				if (void_region_instant_max_occupancy && map.voidRegion(node)) {
					map.occupancySetLogit(node, occupancy_max, false);
				} else {
					map.occupancyUpdateLogit(node, occupancy, occupancy_min, occupancy_max, false);
				}
			} else {
				map.occupancyUpdateLogit(node, occupancy, occupancy_min, occupancy_max, false);
			}
		}

		if constexpr (Map::hasMapTypes(MapType::COLOR) && contains_type_v<Color, Data>) {
			// TODO: Make correct
			map.colorSet(node, data.template get<Color>(), false);
		}

		// TODO: Add more map types
	}

	template <class Map, class T, class... Rest>
	void insertHits(Map& map, PointCloud<Dim, T, Rest...> const& cloud) const
	{
		auto const occ     = probabilityToLogit(occupancy_hit);
		auto const occ_min = probabilityToLogit(occupancy_min_clamp_thres);
		auto const occ_max = probabilityToLogit(occupancy_max_clamp_thres);

		cached_hits_.resize(cloud.size());
		auto points = cloud.template view<0>();
		ufo::transform(
		    points.begin(), points.end(), cached_hits_.begin(),
		    [&map, d = hit_depth](auto const& p) { return map.code(TreeCoord(p, d)); });

		map.create(cached_hits_, cached_hits_.begin());

		ufo::for_each(std::size_t(0), static_cast<std::size_t>(cached_hits_.size()),
		              [this, &map, &cloud, occ, occ_min, occ_max](std::size_t i) {
			              auto node = cached_hits_[i].node;

			              // This chick wants to rule the block (node.pos being the block)
			              //  std::lock_guard lock(map.chicken(node.pos));

			              insertHit(map, node, cloud[i], occ, occ_min, occ_max);
		              });
	}

	template <
	    class ExecutionPolicy, class Map, class T, class... Rest,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void insertHits(ExecutionPolicy&& policy, Map& map,
	                PointCloud<Dim, T, Rest...> const& cloud) const
	{
		auto const occ     = probabilityToLogit(occupancy_hit);
		auto const occ_min = probabilityToLogit(occupancy_min_clamp_thres);
		auto const occ_max = probabilityToLogit(occupancy_max_clamp_thres);

		cached_hits_.resize(cloud.size());
		auto points = cloud.template view<0>();
		ufo::transform(
		    policy, points.begin(), points.end(), cached_hits_.begin(),
		    [&map, d = hit_depth](auto const& p) { return map.code(TreeCoord(p, d)); });

		map.create(policy, cached_hits_, cached_hits_.begin());

		ufo::for_each(std::forward<ExecutionPolicy>(policy), std::size_t(0),
		              static_cast<std::size_t>(cached_hits_.size()),
		              [this, &map, &cloud, occ, occ_min, occ_max](std::size_t i) {
			              auto node = cached_hits_[i].node;

			              // This chick wants to rule the block (node.pos being the block)
			              //  std::lock_guard lock(map.chicken(node.pos));

			              insertHit(map, node, cloud[i], occ, occ_min, occ_max);
		              });
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Misses                                        |
	|                                                                                     |
	**************************************************************************************/

	/*!
	 * @brief Ray-cast from `sensor_origin` to each point and collect the traversed
	 * free-space voxels at `miss_depth`.
	 *
	 * Voxels that contain a measured point are never carved. Uses uniform sampling at
	 * the voxel size -- simple and robust; this is where an exact DDA traversal would
	 * go for a faster / leak-free integrator.
	 */
	template <class Map, class T, class... Rest>
	std::vector<detail::Miss<Dim>> buildMisses(Map const&                         map,
	                                           PointCloud<Dim, T, Rest...> const& cloud,
	                                           Vec<Dim, float> const& sensor_origin) const
	{
		std::vector<detail::Miss<Dim>> misses;

		if constexpr (Map::hasMapTypes(MapType::OCCUPANCY)) {
			using Vecf        = Vec<Dim, float>;
			using Code        = TreeCode<Dim>;
			float const voxel = static_cast<float>(map.length(miss_depth)[0]);
			if (!(voxel > 0.0f)) {
				return misses;
			}

			auto points = cloud.template view<0>();

			// Voxels that contain a measured point must never be carved as free.
			std::unordered_set<Code> occupied_cells;
			for (auto const& p : points) {
				occupied_cells.insert(map.code(TreeCoord(Vecf(p), miss_depth)));
			}

			std::unordered_map<Code, std::uint_fast32_t> miss_count;
			for (auto const& p : points) {
				Vecf const  ray  = Vecf(p) - sensor_origin;
				float const dist = norm(ray);
				if (!std::isfinite(dist) || dist <= voxel) {
					continue;
				}
				Vecf const dir   = ray / dist;
				auto const steps = static_cast<long>((dist - 0.5f * voxel) / voxel);
				for (long s = 1; s <= steps; ++s) {
					Vecf const sample = sensor_origin + dir * (static_cast<float>(s) * voxel);
					Code const code   = map.code(TreeCoord(sample, miss_depth));
					if (occupied_cells.find(code) == occupied_cells.end()) {
						++miss_count[code];
					}
				}
			}

			misses.reserve(miss_count.size());
			for (auto const& [code, count] : miss_count) {
				misses.emplace_back(code, Vecf{}, count, false);
			}
		}

		return misses;
	}

	template <class Map>
	void insertMiss(Map& map, detail::Miss<Dim> const& miss, logit_t occupancy,
	                logit_t occupancy_min, logit_t occupancy_max) const
	{
		if constexpr (Map::hasMapTypes(MapType::VOID_REGION)) {
			if (miss.void_region) {
				map.voidRegionSet(miss.node, true, false);
			}
		}

		if constexpr (Map::hasMapTypes(MapType::OCCUPANCY)) {
			// TODO: Is this good?
			if constexpr (Map::hasMapTypes(MapType::VOID_REGION)) {
				if (void_region_instant_min_occupancy && map.voidRegion(miss.node)) {
					map.occupancySetLogit(miss.node, occupancy_min, false);
				} else {
					map.occupancyUpdateLogit(miss.node, miss.count * occupancy, occupancy_min,
					                         occupancy_max, false);
				}
			} else {
				map.occupancyUpdateLogit(miss.node, miss.count * occupancy, occupancy_min,
				                         occupancy_max, false);
			}
		}

		// TODO: Add more map types
	}

	template <class Map>
	void insertMisses(Map& map, std::vector<detail::Miss<Dim>>& misses) const
	{
		auto const occ     = probabilityToLogit(occupancy_miss);
		auto const occ_min = probabilityToLogit(occupancy_min_clamp_thres);
		auto const occ_max = probabilityToLogit(occupancy_max_clamp_thres);

		map.create(misses, misses.begin());

		ufo::for_each(misses.begin(), misses.end(),
		              [this, &map, occ, occ_min, occ_max](auto const& miss) {
			              // This chick wants to rule the block (node.pos being the block)
			              //  std::lock_guard lock(map.chicken(miss.index));

			              insertMiss(map, miss, occ, occ_min, occ_max);
		              });
	}

	template <
	    class ExecutionPolicy, class Map,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void insertMisses(ExecutionPolicy&& policy, Map& map,
	                  std::vector<detail::Miss<Dim>>& misses) const
	{
		auto const occ     = probabilityToLogit(occupancy_miss);
		auto const occ_min = probabilityToLogit(occupancy_min_clamp_thres);
		auto const occ_max = probabilityToLogit(occupancy_max_clamp_thres);

		map.create(std::forward<ExecutionPolicy>(policy), misses, misses.begin());

		ufo::for_each(std::forward<ExecutionPolicy>(policy), misses.begin(), misses.end(),
		              [this, &map, occ, occ_min, occ_max](auto const& miss) {
			              // This chick wants to rule the block (node.pos being the block)
			              //  std::lock_guard lock(map.chicken(miss.index));

			              insertMiss(map, miss, occ, occ_min, occ_max);
		              });
	}

 protected:
	mutable __block std::vector<detail::Hit<Dim>> cached_hits_;
};
}  // namespace ufo

#endif  // UFO_MAP_INTEGRATOR_INTEGRATOR_HPP