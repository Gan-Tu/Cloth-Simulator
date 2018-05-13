#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm) {
  // Handle collisions with spheres.
  Vector3D op = pm.position - this->origin;
  if (op.norm() < this->radius) {
    Vector3D tangent = this->origin + op.unit() * this->radius;
    Vector3D correction = tangent - pm.last_position;
    pm.position = pm.last_position + correction * (1 - this->friction);
  }
}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  Misc::draw_sphere(shader, origin, radius * 0.92);
}
