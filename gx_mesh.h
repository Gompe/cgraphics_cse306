#pragma once

#include <algorithm>
#include <iostream>
#include <stdio.h>
#include <string>
#include <cstring>
#include <vector>

#include "gx_vector.h"

class TriangleIndices {
public:
  TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1,
                  int nj = -1, int nk = -1, int uvi = -1, int uvj = -1,
                  int uvk = -1, int group = -1, bool added = false)
      : vtxi(vtxi), vtxj(vtxj), vtxk(vtxk), uvi(uvi), uvj(uvj), uvk(uvk),
        ni(ni), nj(nj), nk(nk), group(group){};
        
  int vtxi, vtxj, vtxk; // indices within the vertex coordinates array
  int uvi, uvj, uvk;    // indices within the uv coordinates array
  int ni, nj, nk;       // indices within the normals array
  int group;            // face group
};

class TriangleMesh {
public:
  TriangleMesh() = default;
  ~TriangleMesh() = default;

  void readOBJ(const char *obj);

  std::vector<TriangleIndices> indices;
  std::vector<Vector> vertices;
  std::vector<Vector> normals;
  std::vector<Vector> uvs;
  std::vector<Vector> vertexcolors;
};