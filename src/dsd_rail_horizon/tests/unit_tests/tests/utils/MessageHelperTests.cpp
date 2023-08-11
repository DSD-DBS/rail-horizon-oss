/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include <dsd_rail_horizon/utils/MessageHelper.h>

#include <dsd_rail_horizon_core/MapModel.h>

#include <dsd_common_types/GeometryTypes.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

using testing::Eq;

namespace
{
std::string strip(std::string text)
{
    boost::algorithm::erase_all(text, " ");
    boost::algorithm::erase_all(text, "\n");
    return text;
}
} // namespace

class MessageHelperFixture : public ::testing::Test
{
};


TEST_F(MessageHelperFixture, createColorMessage)
{
    // Act
    auto message = create_color_message(1, 2, 3, 4);
    std::string message_content = strip(to_yaml(message, true));

    // Assert
    std::string expected_message = strip(R"({
        r: 1.00000,
        g: 2.00000,
        b: 3.00000,
        a: 4.00000
    })");
    ASSERT_THAT(message_content, Eq(expected_message));
}

TEST_F(MessageHelperFixture, createScaleMessage)
{
    // Act
    auto message = create_scale_message(1, 2, 3);
    std::string message_content = strip(to_yaml(message, true));

    // Assert
    std::string expected_message = strip(R"({
        x: 1.00000,
        y: 2.00000,
        z: 3.00000
    })");
    ASSERT_THAT(message_content, Eq(expected_message));
}


TEST_F(MessageHelperFixture, createKeyValueMapMessage)
{
    // Act
    auto message = create_key_value_map_message({{"key1", "value1"}, {"key2", "value2"}});
    std::string message_content = strip(to_yaml(message, true));

    // Assert
    std::string expected_message = strip(R"({
        data: [
            {
                key: "key1",
                value: "value1"
            },
            {
                key: "key2",
                value: "value2"
            }
        ]
    })");
    ASSERT_THAT(message_content, Eq(expected_message));
}

TEST_F(MessageHelperFixture, createPointMessage)
{
    // Act
    auto message = create_point_message(PointXyz{1, 2, 3});
    std::string message_content = strip(to_yaml(message, true));

    // Assert
    std::string expected_message = strip(R"({
        x: 1.00000,
        y: 2.00000,
        z: 3.00000
    })");
    ASSERT_THAT(message_content, Eq(expected_message));
}

TEST_F(MessageHelperFixture, geometryToPointVectorLineString)
{
    // Act
    LineStringXyz geometry = {PointXyz{1, 2, 3}, PointXyz{4, 5, 6}};
    std::vector<geometry_msgs::msg::Point> point_vector = geometry_to_point_vector(geometry);

    // Assert
    std::vector<geometry_msgs::msg::Point> expected_point_vector = {
        create_point_message(PointXyz{1, 2, 3}), create_point_message(PointXyz{4, 5, 6})};
    ASSERT_THAT(point_vector, Eq(expected_point_vector));
}

TEST_F(MessageHelperFixture, geometryToPointVectorPolygon)
{
    // Act
    PolygonXyz geometry{{PointXyz{1, 2, 3}, PointXyz{4, 5, 6}}};
    std::vector<geometry_msgs::msg::Point> point_vector =
        geometry_to_point_vector<LineStringXyz>({PointXyz{1, 2, 3}, PointXyz{4, 5, 6}});

    // Assert
    std::vector<geometry_msgs::msg::Point> expected_point_vector = {
        create_point_message(PointXyz{1, 2, 3}), create_point_message(PointXyz{4, 5, 6})};
    ASSERT_THAT(point_vector, Eq(expected_point_vector));
}

TEST_F(MessageHelperFixture, createIdMessage)
{
    // Act
    auto message = create_id_message(100);
    std::string message_content = strip(to_yaml(message, true));

    // Assert
    std::string expected_message = strip(R"({
        id: 100
    })");
    ASSERT_THAT(message_content, Eq(expected_message));
}

template <typename StructureType>
struct LandmarkTestParam
{
    StructureType type;
    int raw_type;

    friend std::ostream& operator<<(std::ostream& os, const LandmarkTestParam& param)
    {
        return os << "type = " << param.type << ", raw_type = " << param.raw_type;
    }
};

class VerticalLandmarkTest : public MessageHelperFixture,
                             public testing::WithParamInterface<LandmarkTestParam<VerticalStructure::Type>>
{
};

TEST_P(VerticalLandmarkTest, createVerticalLandmark)
{
    // Arrange
    auto [type, raw_type] = GetParam();

    // Act
    auto message = create_vertical_landmark(Id{100}, type, PointXyz{1, 2, 3}, 4, PointXyz{5, 6, 7}, 8);
    std::string message_content = strip(to_yaml(message, true));

    // Assert
    std::string expected_message = strip((boost::format{R"({
        base: {
            timestamp: { sec: 0, nanosec: 0},
            existence_probability: { probability: 0},
            pose: {
                pose: {
                    position:{x:0.00000,y:0.00000,z:0.00000},
                    orientation:{x:0.00000,y:0.00000,z:0.00000,w:1.00000}
                },
                covariance: [0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000]
            },
            shape: {
                type: 101,
                parameters: [1.00000,2.00000,3.00000,4.00000,5.00000,6.00000,7.00000,8.00000],
                uncertainties: []
            },
            optional_attributes: {data:[]}
        },
        classification:[
            {
                motion_type: 0,
                class_name: 1,
                type_name: %1%,
                sub_type_name:0,
                motion_type_probability: {probability: 0},
                class_name_probability :{probability:0},
                type_name_probability:{probability:0},
                sub_type_probability:{probability:0},
                optional_attributes:{data:[]}
            }
        ],
        tracking: [
            {
                id: { id: 100},
                velocities: {
                    linear: {x:0.00000,y:0.00000,z:0.00000},
                    angular: {x:0.00000,y:0.00000,z:0.00000}
                },
                velocities_covariance: {covariance: []},
                accelerations:{
                    linear: {x:0.00000,y:0.00000,z:0.00000},
                    angular:{x:0.00000,y:0.00000,z:0.00000}
                },
                accelerations_covariance: {covariance:[]},
                lifetime: {sec:0,nanosec:0},
                historic_path: [],
                predicted_path: [],
                optional_attributes: {data: []}
            }
        ],
        optional_attributes: {data: []}
    })"} % raw_type)
                                             .str());
    ASSERT_THAT(message_content, Eq(expected_message));
}

using VerticalLandmarkTestParam = LandmarkTestParam<VerticalStructure::Type>;

INSTANTIATE_TEST_SUITE_P(VerticalLandmarkTestSuite, VerticalLandmarkTest,
    testing::Values(VerticalLandmarkTestParam{VerticalStructure::Type::UNDEFINED,
                        dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_VERTICAL_TYPE_UNDEFINED},
        VerticalLandmarkTestParam{VerticalStructure::Type::LIGHT,
            dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_VERTICAL_TYPE_LIGHT},
        VerticalLandmarkTestParam{
            VerticalStructure::Type::SIGN, dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_VERTICAL_TYPE_SIGN},
        VerticalLandmarkTestParam{VerticalStructure::Type::SIGNAL,
            dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_VERTICAL_TYPE_SIGNAL},
        VerticalLandmarkTestParam{VerticalStructure::Type::CATENARY,
            dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_VERTICAL_TYPE_CATENARY},
        VerticalLandmarkTestParam{VerticalStructure::Type::PILLAR,
            dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_VERTICAL_TYPE_PILLAR},
        VerticalLandmarkTestParam{VerticalStructure::Type::CAMERA,
            dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_VERTICAL_TYPE_CAMERA},
        VerticalLandmarkTestParam{VerticalStructure::Type::POLE_INFORMATION,
            dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_VERTICAL_TYPE_POLE_INFORMATION},
        VerticalLandmarkTestParam{VerticalStructure::Type::POLE_SHELTER,
            dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_VERTICAL_TYPE_POLE_SHELTER},
        VerticalLandmarkTestParam{VerticalStructure::Type::POLE_BOARD_INFORMATION,
            dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_VERTICAL_TYPE_POLE_BOARD_INFORMATION},
        VerticalLandmarkTestParam{VerticalStructure::Type::BAR_BARRIER,
            dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_VERTICAL_TYPE_BAR_BARRIER}));


class BodyLandmarkTest : public MessageHelperFixture,
                         public testing::WithParamInterface<LandmarkTestParam<BodyStructure::Type>>
{
};

TEST_P(BodyLandmarkTest, createBodyLandmark)
{
    // Act
    auto [type, raw_type] = GetParam();
    auto message = create_body_landmark(Id{100}, type, LineStringXyz{PointXyz{1, 2, 3}, PointXyz{4, 5, 6}}, 200);
    std::string message_content = strip(to_yaml(message, true));

    // Assert
    std::string expected_message = strip((boost::format{R"({
        base: {
            timestamp: { sec: 0, nanosec: 0},
            existence_probability: { probability: 0},
            pose: {
                pose: {
                    position:{x:0.00000,y:0.00000,z:0.00000},
                    orientation:{x:0.00000,y:0.00000,z:0.00000,w:1.00000}
                },
                covariance: [0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000]
            },
            shape: {
                type: 12,
                parameters: [1.00000,2.00000,3.00000,4.00000,5.00000,6.00000, 200.000],
                uncertainties: []
            },
            optional_attributes: {data:[]}
        },
        classification:[
            {
                motion_type: 0,
                class_name: 3,
                type_name: %1%,
                sub_type_name:0,
                motion_type_probability: {probability: 0},
                class_name_probability :{probability:0},
                type_name_probability:{probability:0},
                sub_type_probability:{probability:0},
                optional_attributes:{data:[]}
            }
        ],
        tracking: [
            {
                id: { id: 100},
                velocities: {
                    linear: {x:0.00000,y:0.00000,z:0.00000},
                    angular: {x:0.00000,y:0.00000,z:0.00000}
                },
                velocities_covariance: {covariance: []},
                accelerations:{
                    linear: {x:0.00000,y:0.00000,z:0.00000},
                    angular:{x:0.00000,y:0.00000,z:0.00000}
                },
                accelerations_covariance: {covariance:[]},
                lifetime: {sec:0,nanosec:0},
                historic_path: [],
                predicted_path: [],
                optional_attributes: {data: []}
            }
        ],
        optional_attributes: {data: []}
    })"} % raw_type)
                                             .str());
    ASSERT_THAT(message_content, Eq(expected_message));
}

using BodyLandmarkTestParam = LandmarkTestParam<BodyStructure::Type>;

INSTANTIATE_TEST_SUITE_P(BodyLandmarkTestSuite, BodyLandmarkTest,
    testing::Values(BodyLandmarkTestParam{BodyStructure::Type::UNDEFINED,
                        dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_BODY_TYPE_UNDEFINED},
        BodyLandmarkTestParam{
            BodyStructure::Type::FUSE_BOX, dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_BODY_TYPE_FUSE_BOX},
        BodyLandmarkTestParam{BodyStructure::Type::SWITCH_CABINET,
            dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_BODY_TYPE_SWITCH_CABINET},
        BodyLandmarkTestParam{
            BodyStructure::Type::BUILDING, dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_BODY_TYPE_BUILDING},
        BodyLandmarkTestParam{
            BodyStructure::Type::PLATFORM, dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_BODY_TYPE_PLATFORM},
        BodyLandmarkTestParam{BodyStructure::Type::TRASH_CAN,
            dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_BODY_TYPE_TRASH_CAN},
        BodyLandmarkTestParam{BodyStructure::Type::CONTAINER,
            dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_BODY_TYPE_CONTAINER},
        BodyLandmarkTestParam{BodyStructure::Type::FOUNDATION,
            dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_BODY_TYPE_FOUNDATION},
        BodyLandmarkTestParam{BodyStructure::Type::BARRIER_BODY,
            dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_BODY_TYPE_BARRIER_BODY}));

class PlaneLandmarkTest : public MessageHelperFixture,
                          public testing::WithParamInterface<LandmarkTestParam<PlaneStructure::Type>>
{
};

TEST_P(PlaneLandmarkTest, createPlaneLandmark)
{
    // Act
    auto [type, raw_type] = GetParam();
    auto message = create_plane_landmark(Id{100}, type, LineStringXyz{PointXyz{1, 2, 3}, PointXyz{4, 5, 6}});
    std::string message_content = strip(to_yaml(message, true));

    // Assert
    std::string expected_message = strip((boost::format{R"({
        base: {
            timestamp: { sec: 0, nanosec: 0},
            existence_probability: { probability: 0},
            pose: {
                pose: {
                    position:{x:0.00000,y:0.00000,z:0.00000},
                    orientation:{x:0.00000,y:0.00000,z:0.00000,w:1.00000}
                },
                covariance: [0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000]
            },
            shape: {
                type: 13,
                parameters: [1.00000,2.00000,3.00000,4.00000,5.00000,6.00000],
                uncertainties: []
            },
            optional_attributes: {data:[]}
        },
        classification:[
            {
                motion_type: 0,
                class_name: 4,
                type_name: %1%,
                sub_type_name:0,
                motion_type_probability: {probability: 0},
                class_name_probability :{probability:0},
                type_name_probability:{probability:0},
                sub_type_probability:{probability:0},
                optional_attributes:{data:[]}
            }
        ],
        tracking: [
            {
                id: { id: 100},
                velocities: {
                    linear: {x:0.00000,y:0.00000,z:0.00000},
                    angular: {x:0.00000,y:0.00000,z:0.00000}
                },
                velocities_covariance: {covariance: []},
                accelerations:{
                    linear: {x:0.00000,y:0.00000,z:0.00000},
                    angular:{x:0.00000,y:0.00000,z:0.00000}
                },
                accelerations_covariance: {covariance:[]},
                lifetime: {sec:0,nanosec:0},
                historic_path: [],
                predicted_path: [],
                optional_attributes: {data: []}
            }
        ],
        optional_attributes: {data: []}
    })"} % raw_type)
                                             .str());
    ASSERT_THAT(message_content, Eq(expected_message));
}

using PlaneLandmarkTestParam = LandmarkTestParam<PlaneStructure::Type>;

INSTANTIATE_TEST_SUITE_P(PlaneLandmarkTestSuite, PlaneLandmarkTest,
    testing::Values(PlaneLandmarkTestParam{PlaneStructure::Type::TYPE_UNDEFINED,
                        dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_PLANE_TYPE_UNDEFINED},
        PlaneLandmarkTestParam{
            PlaneStructure::Type::WALL, dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_PLANE_TYPE_WALL},
        PlaneLandmarkTestParam{PlaneStructure::Type::BILLBOARD,
            dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_PLANE_TYPE_BILLBOARD},
        PlaneLandmarkTestParam{
            PlaneStructure::Type::TUNNEL, dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_PLANE_TYPE_TUNNEL}));

class ZoneLandmarkTest : public MessageHelperFixture,
                         public testing::WithParamInterface<LandmarkTestParam<PlaneStructure::SubType>>
{
};

TEST_P(ZoneLandmarkTest, createZoneLandmark)
{
    // Act
    auto [subtype, raw_type] = GetParam();
    auto message = create_zone_landmark(
        Id{100}, PlaneStructure::Type::ZONE, subtype, LineStringXyz{PointXyz{1, 2, 3}, PointXyz{4, 5, 6}});
    std::string message_content = strip(to_yaml(message, true));

    // Assert
    std::string expected_message = strip((boost::format{R"({
        base: {
            timestamp: { sec: 0, nanosec: 0},
            existence_probability: { probability: 0},
            pose: {
                pose: {
                    position:{x:0.00000,y:0.00000,z:0.00000},
                    orientation:{x:0.00000,y:0.00000,z:0.00000,w:1.00000}
                },
                covariance: [0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000]
            },
            shape: {
                type: 13,
                parameters: [1.00000,2.00000,3.00000,4.00000,5.00000,6.00000],
                uncertainties: []
            },
            optional_attributes: {data:[]}
        },
        classification:[
            {
                motion_type: 0,
                class_name: 4,
                type_name: %2%,
                sub_type_name: %1%,
                motion_type_probability: {probability: 0},
                class_name_probability :{probability:0},
                type_name_probability:{probability:0},
                sub_type_probability:{probability:0},
                optional_attributes:{data:[]}
            }
        ],
        tracking: [
            {
                id: { id: 100},
                velocities: {
                    linear: {x:0.00000,y:0.00000,z:0.00000},
                    angular: {x:0.00000,y:0.00000,z:0.00000}
                },
                velocities_covariance: {covariance: []},
                accelerations:{
                    linear: {x:0.00000,y:0.00000,z:0.00000},
                    angular:{x:0.00000,y:0.00000,z:0.00000}
                },
                accelerations_covariance: {covariance:[]},
                lifetime: {sec:0,nanosec:0},
                historic_path: [],
                predicted_path: [],
                optional_attributes: {data: []}
            }
        ],
        optional_attributes: {data: []}
    })"} % raw_type % PlaneStructure::Type::ZONE)
                                             .str());
    ASSERT_THAT(message_content, Eq(expected_message));
}

using ZoneLandmarkTestParam = LandmarkTestParam<PlaneStructure::SubType>;

INSTANTIATE_TEST_SUITE_P(ZoneLandmarkTestSuite, ZoneLandmarkTest,
    testing::Values(ZoneLandmarkTestParam{PlaneStructure::SubType::SUBTYPE_UNDEFINED,
                        dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_PLANE_SUBTYPE_UNDEFINED},
        ZoneLandmarkTestParam{PlaneStructure::SubType::SAFE_ZONE,
            dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_PLANE_SUBTYPE_SAFE_ZONE},
        ZoneLandmarkTestParam{PlaneStructure::SubType::RISK_ZONE,
            dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_PLANE_SUBTYPE_RISK_ZONE},
        ZoneLandmarkTestParam{PlaneStructure::SubType::TRACK_ZONE,
            dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_PLANE_SUBTYPE_TRACK_ZONE},
        ZoneLandmarkTestParam{PlaneStructure::SubType::NEAR_TRACK_ZONE,
            dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_PLANE_SUBTYPE_NEAR_TRACK_ZONE},
        ZoneLandmarkTestParam{PlaneStructure::SubType::ENVIRONMENT_ZONE,
            dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_PLANE_SUBTYPE_ENVIRONMENT_ZONE},
        ZoneLandmarkTestParam{PlaneStructure::SubType::LEVEL_CROSSING_ZONE,
            dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_PLANE_SUBTYPE_LEVEL_CROSSING_ZONE}));