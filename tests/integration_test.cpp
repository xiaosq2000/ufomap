// UFO
#include <ufo/cloud/point_cloud.hpp>
#include <ufo/map/color/map.hpp>
#include <ufo/map/integrator/integrator.hpp>
#include <ufo/map/occupancy/map.hpp>
#include <ufo/map/ufomap.hpp>

// Catch2
#include <catch2/catch_test_macros.hpp>

using namespace ufo;

TEST_CASE("Integrator inserts hits and carves free space")
{
	Integrator<3>                 integrator;
	Map3D<OccupancyMap, ColorMap> map(0.1, 16);
	PointCloud<3, float, Color>   cloud;

	// A single measurement 1 m in front of a sensor placed at the origin.
	cloud.emplace_back(Vec3f(1.0f, 0.0f, 0.0f), Color(40, 50, 60));

	integrator.occupancy_hit  = 0.7f;
	integrator.occupancy_miss = 0.4f;
	integrator.propagate      = true;

	// Identity transform -> cloud already in the map frame, sensor at the origin.
	integrator(execution::seq, map, cloud);

	// The measured point is occupied (and not free).
	REQUIRE(map.containsOccupied(Vec3f(1.0f, 0.0f, 0.0f)));
	REQUIRE_FALSE(map.containsFree(Vec3f(1.0f, 0.0f, 0.0f)));

	// The space between the sensor and the point has been carved free.
	REQUIRE(map.containsFree(Vec3f(0.5f, 0.0f, 0.0f)));
	REQUIRE_FALSE(map.containsOccupied(Vec3f(0.5f, 0.0f, 0.0f)));

	// Space beyond the measured point was never observed -> still unknown.
	REQUIRE(map.containsUnknown(Vec3f(3.0f, 0.0f, 0.0f)));
}
