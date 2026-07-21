#include "render/TerrainMesh.h"

#include <algorithm>
#include <cmath>

namespace gv {

TerrainMeshData TerrainMesh::buildMesh(const BBox& area, int gridResolution,
                                       const HeightFn& height) const {
  TerrainMeshData mesh;
  const int n = gridResolution < 1 ? 1 : gridResolution;

  const double lngStep = (area.maxLng - area.minLng) / n;
  const double latStep = (area.maxLat - area.minLat) / n;

  // Regular (n+1) x (n+1) grid of vertices, sampling the height field.
  // The + 4*n reserves the perimeter skirt ring appended at the end.
  mesh.vertices.reserve((n + 1) * (n + 1) + 4 * n);
  for (int row = 0; row <= n; ++row) {
    for (int col = 0; col <= n; ++col) {
      LatLng c{ area.minLat + latStep * row, area.minLng + lngStep * col };
      double elev = height(c);
      Vec3d w = latLngToWorld(c, elev);
      mesh.vertices.push_back(TerrainVertex{ w.toFloat() });
    }
  }

  // Two triangles per cell.
  auto index = [n](int row, int col) { return static_cast<std::uint32_t>(row * (n + 1) + col); };
  // The + 24*n reserves the skirt: 4*n perimeter segments x 2 triangles x 3 indices.
  mesh.indices.reserve(n * n * 6 + 24 * n);
  for (int row = 0; row < n; ++row) {
    for (int col = 0; col < n; ++col) {
      std::uint32_t a = index(row,     col);
      std::uint32_t b = index(row,     col + 1);
      std::uint32_t c = index(row + 1, col);
      std::uint32_t d = index(row + 1, col + 1);
      mesh.indices.insert(mesh.indices.end(), { a, c, b, b, c, d });
    }
  }

  std::vector<std::uint32_t> perim;
  perim.reserve(4 * n);
  for (int col = 0;     col <= n;     ++col) perim.push_back(index(0,   col)); // south (both corners)
  for (int row = 1;     row <= n;     ++row) perim.push_back(index(row, n));   // east
  for (int col = n - 1; col >= 0;     --col) perim.push_back(index(n,   col)); // north
  for (int row = n - 1; row >= 1;     --row) perim.push_back(index(row, 0));   // west

  float minZ = mesh.vertices[perim[0]].position.z;
  float maxZ = minZ;
  for (std::uint32_t t : perim) {
    const float z = mesh.vertices[t].position.z;
    minZ = std::min(minZ, z);
    maxZ = std::max(maxZ, z);
  }
  const double worldW    = lngToWorldX(area.maxLng) - lngToWorldX(area.minLng);
  const double worldH    = latToWorldY(area.maxLat) - latToWorldY(area.minLat);
  const double cellWorld = std::min(std::abs(worldW), std::abs(worldH)) / n;
  const float  skirtDepth = std::max(maxZ - minZ, static_cast<float>(0.25 * cellWorld));

  const std::uint32_t skirtBase = static_cast<std::uint32_t>(mesh.vertices.size());
  for (std::uint32_t t : perim) {
    Vec3f p = mesh.vertices[t].position;
    p.z -= skirtDepth;
    mesh.vertices.push_back(TerrainVertex{ p });
  }

  const std::uint32_t ring = static_cast<std::uint32_t>(perim.size()); // == 4*n
  for (std::uint32_t i = 0; i < ring; ++i) {
    const std::uint32_t j  = (i + 1) % ring;
    const std::uint32_t top0 = perim[i];       // edge vertex
    const std::uint32_t top1 = perim[j];       // next edge vertex
    const std::uint32_t bot0 = skirtBase + i;  // lowered copy of top0
    const std::uint32_t bot1 = skirtBase + j;  // lowered copy of top1
    mesh.indices.insert(mesh.indices.end(), { top0, top1, bot0, top1, bot1, bot0 });
  }

  return mesh;
}

} // namespace gv
