/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "testing/testing.h"
#include "tests/blendfile_loading_base_test.h"

#include <pxr/base/tf/stringUtils.h>
#include <pxr/imaging/hd/meshSchema.h>
#include <pxr/imaging/hd/primvarsSchema.h>
#include <pxr/imaging/hd/tokens.h>

#include "BKE_appdir.hh"
#include "BKE_attribute.hh"
#include "BKE_main.hh"
#include "BKE_material.hh"
#include "BKE_mesh.hh"
#include "BKE_object.hh"

#include "BLI_listbase.hh"

#include "BLO_readfile.hh"

#include "DEG_depsgraph_query.hh"

#include "DNA_mesh_types.h"
#include "DNA_object_types.h"

#include "FN_init.hh"

#include "hydra/scene_index.hh"

namespace blender::io::hydra {

class HydraMeshTest : public BlendfileLoadingBaseTest {
 public:
  static void SetUpTestCase()
  {
    BlendfileLoadingBaseTest::SetUpTestCase();
    BKE_tempdir_init(nullptr);
    fn::multi_function::register_common_functions();
  }

 protected:
  void check_mesh(const Mesh &mesh, const pxr::HdSceneIndexPrim &prim, const Span<int> triangles)
  {
    ASSERT_EQ(prim.primType, pxr::HdPrimTypeTokens->mesh);
    const pxr::HdMeshTopologySchema topology =
        pxr::HdMeshSchema::GetFromParent(prim.dataSource).GetTopology();
    ASSERT_TRUE(topology.GetFaceVertexCounts());
    ASSERT_TRUE(topology.GetFaceVertexIndices());
    const pxr::VtIntArray counts = topology.GetFaceVertexCounts()->GetTypedValue(0);
    const pxr::VtIntArray indices = topology.GetFaceVertexIndices()->GetTypedValue(0);
    EXPECT_EQ(counts.size(), triangles.size());
    ASSERT_EQ(indices.size(), triangles.size() * 3);
    for (const int count : counts) {
      EXPECT_EQ(count, 3);
    }

    const pxr::HdPrimvarsSchema primvars = pxr::HdPrimvarsSchema::GetFromParent(prim.dataSource);
    const pxr::HdPrimvarSchema normals_schema = primvars.GetPrimvar(
        pxr::HdPrimvarsSchemaTokens->normals);
    const auto points_source = pxr::HdVec3fArrayDataSource::Cast(
        primvars.GetPrimvar(pxr::HdPrimvarsSchemaTokens->points).GetPrimvarValue());
    const auto normals_source = pxr::HdVec3fArrayDataSource::Cast(
        normals_schema.GetPrimvarValue());
    ASSERT_TRUE(points_source);
    ASSERT_TRUE(normals_source);
    ASSERT_TRUE(normals_schema.GetInterpolation());
    ASSERT_TRUE(normals_schema.GetRole());
    const pxr::VtVec3fArray points = points_source->GetTypedValue(0);
    const pxr::VtVec3fArray normals = normals_source->GetTypedValue(0);
    Set<int> referenced_vertices;
    for (const int triangle : triangles) {
      const int3 tri = mesh.corner_tris()[triangle];
      for (const int c : IndexRange(3)) {
        referenced_vertices.add(mesh.corner_verts()[tri[c]]);
      }
    }
    EXPECT_EQ(points.size(), referenced_vertices.size());
    const pxr::TfToken interpolation = normals_schema.GetInterpolation()->GetTypedValue(0);
    EXPECT_EQ(normals_schema.GetRole()->GetTypedValue(0), pxr::HdPrimvarSchemaTokens->normal);
    switch (mesh.normals_domain()) {
      case bke::MeshNormalDomain::Point:
        EXPECT_EQ(interpolation, pxr::HdPrimvarSchemaTokens->vertex);
        ASSERT_EQ(normals.size(), points.size());
        break;
      case bke::MeshNormalDomain::Face:
        EXPECT_EQ(interpolation, pxr::HdPrimvarSchemaTokens->uniform);
        ASSERT_EQ(normals.size(), counts.size());
        break;
      case bke::MeshNormalDomain::Corner:
        EXPECT_EQ(interpolation, pxr::HdPrimvarSchemaTokens->faceVarying);
        ASSERT_EQ(normals.size(), indices.size());
        break;
    }

    /* Compare at every triangle corner, including the submesh's vertex remapping. */
    const Span<float3> corner_normals = mesh.corner_normals();
    for (const int i : triangles.index_range()) {
      const int3 tri = mesh.corner_tris()[triangles[i]];
      for (const int c : IndexRange(3)) {
        const int point = indices[i * 3 + c];
        ASSERT_GE(point, 0);
        ASSERT_LT(point, points.size());
        const float3 position = mesh.vert_positions()[mesh.corner_verts()[tri[c]]];
        EXPECT_EQ(points[point], pxr::GfVec3f(position.x, position.y, position.z));
        const float3 normal = corner_normals[tri[c]];
        const int normal_index = interpolation == pxr::HdPrimvarSchemaTokens->vertex  ? point :
                                 interpolation == pxr::HdPrimvarSchemaTokens->uniform ? i :
                                                                                        i * 3 + c;
        ASSERT_LT(normal_index, normals.size());
        for (const int axis : IndexRange(3)) {
          EXPECT_FLOAT_EQ(normals[normal_index][axis], normal[axis]);
        }
      }
    }
  }

  void check_scene(const bool material_subsets = false)
  {
    depsgraph_create(DAG_EVAL_RENDER);
    HydraSceneIndex scene_index(pxr::SdfPath("/scene"), nullptr, false);
    scene_index.populate(depsgraph, nullptr);
    bool has_point = false;
    bool has_face = false;
    bool has_corner = false;
    int meshes_num = 0;
    for (Object &object : bfile->main->objects) {
      if (object.type != OB_MESH) {
        continue;
      }
      Object *evaluated = DEG_get_evaluated(depsgraph, &object);
      const Mesh *mesh = BKE_object_get_evaluated_mesh(evaluated);
      ASSERT_NE(mesh, nullptr);
      SCOPED_TRACE(object.id.name + 2);
      meshes_num++;
      has_point |= mesh->normals_domain() == bke::MeshNormalDomain::Point;
      has_face |= mesh->normals_domain() == bke::MeshNormalDomain::Face;
      has_corner |= mesh->normals_domain() == bke::MeshNormalDomain::Corner;
      const pxr::SdfPath path(pxr::TfStringPrintf("/scene/O_%p", &evaluated->id));
      const pxr::SdfPathVector submeshes = scene_index.retained()->GetChildPrimPaths(path);
      ASSERT_EQ(submeshes.size(), material_subsets ? 2 : 1);
      const VArray<int> materials = *mesh->attributes().lookup_or_default<int>(
          "material_index", bke::AttrDomain::Face, 0);
      for (const int submesh : IndexRange(submeshes.size())) {
        Vector<int> triangles;
        for (const int i : mesh->corner_tris().index_range()) {
          /* Slot 1 is deliberately unused to also check empty submesh removal. */
          if (!material_subsets || materials[mesh->corner_tri_faces()[i]] == submesh * 2) {
            triangles.append(i);
          }
        }
        ASSERT_FALSE(triangles.is_empty());
        const pxr::SdfPath submesh_path = path.AppendChild(
            pxr::TfToken(pxr::TfStringPrintf("SM_%04d", submesh)));
        check_mesh(*mesh, scene_index.retained()->GetPrim(submesh_path), triangles);
      }
    }
    EXPECT_GT(meshes_num, 0);
    EXPECT_TRUE(has_point);
    EXPECT_TRUE(has_face);
    EXPECT_TRUE(has_corner);
  }
};

TEST_F(HydraMeshTest, NormalDomains)
{
  ASSERT_TRUE(blendfile_load("usd/usd_mesh_normals.blend"));
  check_scene();
}

TEST_F(HydraMeshTest, MaterialSubsets)
{
  ASSERT_TRUE(blendfile_load("usd/usd_mesh_normals.blend"));
  for (Mesh &mesh : bfile->main->meshes) {
    BKE_id_material_resize(bfile->main, &mesh.id, 3, true);
    bke::SpanAttributeWriter<int> materials =
        mesh.attributes_for_write().lookup_or_add_for_write_span<int>("material_index",
                                                                      bke::AttrDomain::Face);
    for (const int face : materials.span.index_range()) {
      materials.span[face] = (face % 2) * 2;
    }
    materials.finish();
  }
  check_scene(true);
}

}  // namespace blender::io::hydra
