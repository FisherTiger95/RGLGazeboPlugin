// Copyright 2022 Robotec.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdio>

#include "RGLServerPluginInstance.hh"
#include "Utils.hh"

#include <gz/msgs/PointCloudPackedUtils.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/World.hh>

#define PARAM_UPDATE_RATE_ID "update_rate"
#define PARAM_RANGE_ID "range"
#define PARAM_TOPIC_ID "topic"
#define PARAM_FRAME_ID "frame"
#define PARAM_UPDATE_ON_PAUSED_SIM_ID "update_on_paused_sim"
#define PARAM_GAUSSIAN_NOISE_MEAN "gaussian_noise_mean"
#define PARAM_GAUSSIAN_NOISE_DEV_BASE "gaussian_noise_dev_base"
#define PARAM_GAUSSIAN_NOISE_RISE_PER_METER "gaussian_noise_rise_per_meter"

namespace rgl
{

bool RGLServerPluginInstance::LoadConfiguration(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf, gz::sim::EntityComponentManager& ecm)
{
    auto sensor = gz::sim::Sensor(entity);
    if (!sensor.Valid(ecm)) {
        gzerr << "RGLServerPluginInstance plugin should be attached to a sensor entity. "
              << "Failed to initialize.\n";
        return false;
    }

    // Required parameters
    if (!sdf->HasElement(PARAM_UPDATE_RATE_ID)) {
        gzerr << "No '" << PARAM_UPDATE_RATE_ID << "' parameter specified for the RGL lidar. Disabling plugin.\n";
        return false;
    }

    if (!sdf->HasElement(PARAM_RANGE_ID)) {
        gzerr << "No '" << PARAM_RANGE_ID << "' parameter specified for the RGL lidar. Disabling plugin.\n";
        return false;
    }

    std::vector<std::string> topics;
    if (sdf->HasElement(PARAM_TOPIC_ID)) {
        topics.push_back(sdf->Get<std::string>(PARAM_TOPIC_ID));
    }
    gz::sim::World world(worldEntity(ecm));
    if (world.Name(ecm)) {
        topics.push_back("/world/" + world.Name(ecm).value() + "/" + gz::sim::scopedName(entity, ecm, "/", true) + "/scan/points");
    }
    topicName = gz::sim::validTopic(topics);
    if (topicName.empty()) {
        gzerr << "No '" << PARAM_TOPIC_ID << "' parameter specified for the RGL lidar. Disabling plugin.\n";
        return false;
    }

    if (sdf->HasElement(PARAM_FRAME_ID)) {
        frameId = sdf->Get<std::string>(PARAM_FRAME_ID);
    } else if(sensor.Parent(ecm)) {
        frameId = gz::sim::scopedName(sensor.Parent(ecm).value(), ecm, "::", false) + "::" + sensor.Name(ecm).value();
    } else {
        gzerr << "No '" << PARAM_FRAME_ID << "' parameter specified for the RGL lidar. Disabling plugin.\n";
        return false;
    }

    if (!sdf->HasElement(PARAM_GAUSSIAN_NOISE_MEAN)){
        gzerr << "No '" << PARAM_GAUSSIAN_NOISE_MEAN << "' parameter specified for the RGL lidar. Disabling plugin.\n";
        return false;
    }

    if (!sdf->HasElement(PARAM_GAUSSIAN_NOISE_DEV_BASE)){
        gzerr << "No '" << PARAM_GAUSSIAN_NOISE_DEV_BASE << "' parameter specified for the RGL lidar. Disabling plugin.\n";
        return false;
    }

    if (!sdf->HasElement(PARAM_GAUSSIAN_NOISE_RISE_PER_METER)){
        gzerr << "No '" << PARAM_GAUSSIAN_NOISE_RISE_PER_METER << "' parameter specified for the RGL lidar. Disabling plugin.\n";
        return false;
    }

    // Optional parameters
    if (!sdf->HasElement(PARAM_UPDATE_ON_PAUSED_SIM_ID)) {
        gzwarn << "No '" << PARAM_UPDATE_ON_PAUSED_SIM_ID << "' parameter specified for the RGL lidar. "
                << "Using default value: " << updateOnPausedSim << "\n";
    } else {
        updateOnPausedSim = sdf->Get<bool>(PARAM_UPDATE_ON_PAUSED_SIM_ID);
    }

    // Load configuration
    float updateRateHz = sdf->Get<float>(PARAM_UPDATE_RATE_ID);
    raytraceIntervalTime = std::chrono::microseconds(static_cast<int64_t>(1e6 / updateRateHz));
    lidarRange = sdf->Get<float>(PARAM_RANGE_ID);
    gaussianNoiseMean = sdf->Get<float>(PARAM_GAUSSIAN_NOISE_MEAN);
    gaussianNoiseStDevBase = sdf->Get<float>(PARAM_GAUSSIAN_NOISE_DEV_BASE);
    gaussianNoiseStDevRisePerMeter = sdf->Get<float>(PARAM_GAUSSIAN_NOISE_RISE_PER_METER);

    if (!LidarPatternLoader::Load(sdf, lidarPattern)) {
        return false;
    }
    // Resize container for result point cloud
    resultPointCloud.resize(lidarPattern.size());

    return true;
}

void RGLServerPluginInstance::CreateLidar(gz::sim::Entity entity,
                                          gz::sim::EntityComponentManager& ecm)
{
    thisLidarEntity = entity;

    rgl_mat3x4f identity = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0
    };

    if (!CheckRGL(rgl_node_rays_from_mat3x4f(&rglNodeUseRays, lidarPattern.data(), lidarPattern.size())) ||
        !CheckRGL(rgl_node_rays_transform(&rglNodeLidarPose, &identity)) ||
        !CheckRGL(rgl_node_raytrace(&rglNodeRaytrace, nullptr, lidarRange)) ||
        !CheckRGL(rgl_node_gaussian_noise_distance(&rglNodeGaussianNoiseDistanceNode, gaussianNoiseMean, gaussianNoiseStDevBase, gaussianNoiseStDevRisePerMeter)) ||
        !CheckRGL(rgl_node_points_compact(&rglNodeCompact)) ||
        !CheckRGL(rgl_node_points_transform(&rglNodeToLidarFrame, &identity))) {

        gzerr << "Failed to create RGL nodes when initializing lidar. Disabling plugin.\n";
        return;
    }

    if (!CheckRGL(rgl_graph_node_add_child(rglNodeUseRays, rglNodeLidarPose)) ||
        !CheckRGL(rgl_graph_node_add_child(rglNodeLidarPose, rglNodeRaytrace)) ||
        !CheckRGL(rgl_graph_node_add_child(rglNodeRaytrace, rglNodeGaussianNoiseDistanceNode)) ||
        !CheckRGL(rgl_graph_node_add_child(rglNodeGaussianNoiseDistanceNode, rglNodeCompact)) ||
        !CheckRGL(rgl_graph_node_add_child(rglNodeCompact, rglNodeToLidarFrame))) {
        
        gzerr << "Failed to connect RGL nodes when initializing lidar. Disabling plugin.\n";
        return;
    }

    pointCloudPublisher = gazeboNode.Advertise<gz::msgs::PointCloudPacked>(topicName);
    pointCloudWorldPublisher = gazeboNode.Advertise<gz::msgs::PointCloudPacked>(topicName + worldTopicPostfix);

    isLidarInitialized = true;
}

void RGLServerPluginInstance::UpdateLidarPose(const gz::sim::EntityComponentManager& ecm)
{
    gz::math::Pose3<double> gzLidarToWorld = FindWorldPose(thisLidarEntity, ecm);
    gz::math::Pose3<double> gzWorldToLidar = gzLidarToWorld.Inverse();
    rgl_mat3x4f rglLidarToWorld = GzPose3dToRglMatrix(gzLidarToWorld);
    rgl_mat3x4f rglWorldToLidar = GzPose3dToRglMatrix(gzWorldToLidar);
    CheckRGL(rgl_node_rays_transform(&rglNodeLidarPose, &rglLidarToWorld));
    CheckRGL(rgl_node_points_transform(&rglNodeToLidarFrame, &rglWorldToLidar));
}

bool RGLServerPluginInstance::ShouldRayTrace(std::chrono::steady_clock::duration simTime,
                                             bool paused)
{
    if (!isLidarInitialized) {
        return false;
    }

    if (paused) {
        ++onPausedSimUpdateCounter;
        if (!updateOnPausedSim) {
            return false;
        }
        if (onPausedSimUpdateCounter < onPausedSimRaytraceInterval) {
            return false;
        }
        onPausedSimUpdateCounter = 0;
        return true;
    }

    // Simulation running
    if (simTime < lastRaytraceTime + raytraceIntervalTime) {
        return false;
    }
    return true;
}

void RGLServerPluginInstance::RayTrace(std::chrono::steady_clock::duration simTime)
{
    lastRaytraceTime = simTime;

    if (!CheckRGL(rgl_graph_run(rglNodeRaytrace))) {
        gzerr << "Failed to perform raytrace.\n";
        return;
    }

    int32_t hitpointCount = 0;
    if (!CheckRGL(rgl_graph_get_result_size(rglNodeToLidarFrame, RGL_FIELD_XYZ_F32, &hitpointCount, nullptr)) ||
        !CheckRGL(rgl_graph_get_result_data(rglNodeToLidarFrame, RGL_FIELD_XYZ_F32, resultPointCloud.data()))) {

        gzerr << "Failed to get result data from RGL lidar.\n";
        return;
    }

    auto msg = CreatePointCloudMsg(simTime, frameId, hitpointCount);
    pointCloudPublisher.Publish(msg);

    if (pointCloudWorldPublisher.HasConnections()) {
        if (!CheckRGL(rgl_graph_get_result_size(rglNodeToLidarFrame, RGL_FIELD_XYZ_F32, &hitpointCount, nullptr)) ||
            !CheckRGL(rgl_graph_get_result_data(rglNodeCompact, RGL_FIELD_XYZ_F32, resultPointCloud.data()))) {

            gzerr << "Failed to get visualization data from RGL lidar.\n";
            return;
        }
        auto worldMsg = CreatePointCloudMsg(simTime, worldFrameId, hitpointCount);
        pointCloudWorldPublisher.Publish(worldMsg);
    }
}

gz::msgs::PointCloudPacked RGLServerPluginInstance::CreatePointCloudMsg(std::chrono::steady_clock::duration simTime, std::string frame, int hitpointCount)
{
    gz::msgs::PointCloudPacked outMsg;
    gz::msgs::InitPointCloudPacked(outMsg, frame, false,
                                         {{"xyz", gz::msgs::PointCloudPacked::Field::FLOAT32}});
    outMsg.mutable_data()->resize(hitpointCount * outMsg.point_step());
    *outMsg.mutable_header()->mutable_stamp() = gz::msgs::Convert(simTime);
    outMsg.set_height(1);
    outMsg.set_width(hitpointCount);

    gz::msgs::PointCloudPackedIterator<float> xIterWorld(outMsg, "x");
    memcpy(&(*xIterWorld), resultPointCloud.data(), hitpointCount * sizeof(rgl_vec3f));
    return outMsg;
}

void RGLServerPluginInstance::DestroyLidar()
{
    if (!isLidarInitialized) {
        return;
    }

    if (!CheckRGL(rgl_graph_destroy(rglNodeRaytrace))) {
        gzerr << "Failed to destroy RGL lidar.\n";
    }
    // Reset publishers
    pointCloudPublisher = gz::transport::Node::Publisher();
    pointCloudWorldPublisher = gz::transport::Node::Publisher();
    isLidarInitialized = false;
}

}  // namespace rgl
