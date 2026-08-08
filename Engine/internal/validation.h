#ifndef TINY2DENGINE_ENGINE_INTERNAL_VALIDATION_H_
#define TINY2DENGINE_ENGINE_INTERNAL_VALIDATION_H_

#include "tiny2d_engine.h"

// Engine-internal input validation. Every Validate* function throws
// std::invalid_argument on invalid state and otherwise returns normally, so
// public entry points can complete all checks before mutating anything.

namespace tiny2d::internal {

constexpr float kMinimumDynamicMass = 0.000001f;

void Require(bool condition, const char* message);
void RequireFloatResult(double value, const char* message);

bool UsesWorldMaterial(const CollisionMaterial& material);
void ValidateMaterial(const CollisionMaterial& material);

void ValidateGeometry(const Rectangle& rectangle);
void ValidateGeometry(const Circle& circle);
void ValidateRectangle(const Rectangle& rectangle);
void ValidateCircle(const Circle& circle);

bool FitsInArea(const Rectangle& rectangle, float area_width,
                float area_height);
bool FitsInArea(const Circle& circle, float area_width, float area_height);

}  // namespace tiny2d::internal

#endif  // TINY2DENGINE_ENGINE_INTERNAL_VALIDATION_H_
