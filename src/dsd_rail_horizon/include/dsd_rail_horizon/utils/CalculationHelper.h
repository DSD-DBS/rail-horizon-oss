/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_RAIL_HORIZON_CALCULATION_HELPER_H
#define DSD_RAIL_HORIZON_CALCULATION_HELPER_H

#include <dsd_common_types/GeometryTypes.h>
/**
 * @brief Type alias for rotation matrix
 */
using RotationMatrix = boost::qvm::mat<double, 4, 4>;

/**
 * @brief Calculates the rotation matrix based on the coupled localization orientation
 * @param angle point with three double values for the angles
 * @return rotation matrix with dimensions 4x4
 */
inline RotationMatrix calculate_rotation_matrix(const PointXyz& angle)
{
    double roll = angle.get<0>();
    double pitch = angle.get<1>();
    double yaw = angle.get<2>();

    RotationMatrix rotation_matrix = {};
    boost::qvm::set_rot_zyx(rotation_matrix, yaw, pitch, roll);
    return rotation_matrix;
}
/**
 * @brief Helper class for geometry transformations (rotation and translation)
 */
class GeometryTransformation
{
public:
    /**
     * @brief Constructs GeometryTransformation object to be used for filling the local horizon
     * @param shift Constant reference to 3D point which refers to the position of the train
     * @param rotation Constat reference to RotationMatrix of type boost::qvm::mat<double, 4, 4>
     */
    GeometryTransformation(const PointXyz& shift, const RotationMatrix& rotation) : shift_{shift}, rotation_{rotation}
    {
    }
    /**
     * @brief Transforms the UTM geometry by shifting the geometry relative to the position of the train and then
     * rotating it
     * @param geometry geometry of an object
     * @return new_geometry of the object after the rotation
     */
    template <typename GeometryXyz>
    [[nodiscard]] GeometryXyz transform(const GeometryXyz& geometry) const
    {
        GeometryXyz local_geometry;
        std::transform(begin(geometry), end(geometry), back_inserter(local_geometry), [&](const auto& point) {
            return subtract_points(point, shift_);
        });

        return rotate_geometry(rotation_, local_geometry);
    }

private:
    /**
     * @brief Subtracts two points
     * @param p1 point in 3D space
     * @param p2 point in 3D space
     * @return point that gives the result of the substraction of two points
     */
    [[nodiscard]] PointXyz subtract_points(const PointXyz& p1, const PointXyz& p2) const
    {
        return {p1.get<0>() - p2.get<0>(), p1.get<1>() - p2.get<1>(), p1.get<2>() - p2.get<2>()};
    }
    /**
     * @brief Rotates point with respect to a given rotation matrix
     * @param point Constant reference to 3D point
     * @param rotation Constat reference to RotationMatrix of type boost::qvm::mat<double, 4, 4>
     */
    [[nodiscard]] PointXyz rotate_point(const PointXyz& point, const RotationMatrix& rotation) const
    {
        boost::qvm::mat<double, 4, 1> point_4d = {point.get<0>(), point.get<1>(), point.get<2>(), 0};
        auto result = rotation * point_4d;
        return PointXyz{boost::qvm::A00(result), boost::qvm::A10(result), boost::qvm::A20(result)};
    }

    /**
     * @brief Rotates the local horizon. This functions should be used only with UTM coordinates
     * @param rotation rotation matrix with dimensions 4x4
     * @param geometry geometry of an object
     * @return new_geometry of the object after the rotation
     */
    template <typename GeometryXyz>
    [[nodiscard]] GeometryXyz rotate_geometry(const RotationMatrix& rotation, const GeometryXyz& geometry) const
    {
        GeometryXyz rotated_geometry;
        std::transform(begin(geometry), end(geometry), back_inserter(rotated_geometry), [&](const PointXyz& point) {
            return rotate_point(point, rotation);
        });
        return rotated_geometry;
    }
    /**
     * @brief 3D Point which is origin of the local coordinate system
     */
    PointXyz shift_;
    /**
     * @brief rotation matrix R(yaw,pitch,roll)
     */
    RotationMatrix rotation_;
};
#endif // DSD_RAIL_HORIZON_CALCULATION_HELPER_H