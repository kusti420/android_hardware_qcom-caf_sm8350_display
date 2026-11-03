/*
* Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above
*       copyright notice, this list of conditions and the following
*       disclaimer in the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of The Linux Foundation nor the names of its
*       contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
* ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
* Changes from Qualcomm Innovation Center are provided under the following license:
*
* Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted (subject to the limitations in the
* disclaimer below) provided that the following conditions are met:
*
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*
*    * Neither the name of Qualcomm Innovation Center, Inc. nor the
*      names of its contributors may be used to endorse or promote
*      products derived from this software without specific prior
*      written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
* GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
* HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
* IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#define __STDC_FORMAT_MACROS

#include <ctype.h>
#include <time.h>
#include <drm/drm_fourcc.h>
#include <drm_lib_loader.h>
#include <drm_master.h>
#include <drm_res_mgr.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/fb.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/system_properties.h>
#include <sys/types.h>
#include <unistd.h>
#include <utils/constants.h>
#include <utils/debug.h>
#include <utils/formats.h>
#include <utils/sys.h>
#include <display/drm/sde_drm.h>
#if defined(UDFPS_ZPOS) && defined(PP_DRM_ENABLE)
#include <display/drm/msm_drm_pp.h>  // IchthysOS: drm_msm_pgc_lut for the UDFPS mask tone curve
#endif
#include <private/color_params.h>
#include <utils/rect.h>
#include <utils/utils.h>
#include <utils/fence.h>

#include <sstream>
#include <cmath>
#include <ctime>
#include <algorithm>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <limits>

#include "hw_device_drm.h"
#include "hw_info_interface.h"

#define __CLASS__ "HWDeviceDRM"

#ifndef DRM_FORMAT_MOD_QCOM_COMPRESSED
#define DRM_FORMAT_MOD_QCOM_COMPRESSED fourcc_mod_code(QCOM, 1)
#endif
#ifndef DRM_FORMAT_MOD_QCOM_DX
#define DRM_FORMAT_MOD_QCOM_DX fourcc_mod_code(QCOM, 0x2)
#endif
#ifndef DRM_FORMAT_MOD_QCOM_TIGHT
#define DRM_FORMAT_MOD_QCOM_TIGHT fourcc_mod_code(QCOM, 0x4)
#endif

#define DEST_SCALAR_OVERFETCH_SIZE 5

using std::string;
using std::to_string;
using std::fstream;
using std::unordered_map;
using std::stringstream;
using std::ifstream;
using std::ofstream;
using drm_utils::DRMMaster;
using drm_utils::DRMResMgr;
using drm_utils::DRMLibLoader;
using drm_utils::DRMBuffer;
using sde_drm::GetDRMManager;
using sde_drm::DestroyDRMManager;
using sde_drm::DRMDisplayType;
using sde_drm::DRMDisplayToken;
using sde_drm::DRMConnectorInfo;
using sde_drm::DRMPPFeatureInfo;
using sde_drm::DRMRect;
using sde_drm::DRMRotation;
using sde_drm::DRMBlendType;
using sde_drm::DRMSrcConfig;
using sde_drm::DRMOps;
using sde_drm::DRMTopology;
using sde_drm::DRMPowerMode;
using sde_drm::DRMSecureMode;
using sde_drm::DRMSecurityLevel;
using sde_drm::DRMCscType;
using sde_drm::DRMMultiRectMode;
using sde_drm::DRMCrtcInfo;

namespace sdm {

std::atomic<uint32_t> HWDeviceDRM::hw_dest_scaler_blocks_used_(0);
bool HWDeviceDRM::planes_reset_cache_(true);

static PPBlock GetPPBlock(const HWToneMapLut &lut_type) {
  PPBlock pp_block = kPPBlockMax;
  switch (lut_type) {
    case kDma1dIgc:
    case kDma1dGc:
      pp_block = kDGM;
      break;
    case kVig1dIgc:
    case kVig3dGamut:
      pp_block = kVIG;
      break;
    default:
      DLOGE("Unknown PP Block");
      break;
  }
  return pp_block;
}

static void GetDRMFormat(LayerBufferFormat format, uint32_t *drm_format,
                         uint64_t *drm_format_modifier) {
  switch (format) {
    case kFormatRGBA8888:
      *drm_format = DRM_FORMAT_ABGR8888;
      break;
    case kFormatRGBA8888Ubwc:
      *drm_format = DRM_FORMAT_ABGR8888;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED;
      break;
    case kFormatRGBA5551:
      *drm_format = DRM_FORMAT_ABGR1555;
      break;
    case kFormatRGBA4444:
      *drm_format = DRM_FORMAT_ABGR4444;
      break;
    case kFormatBGRA8888:
      *drm_format = DRM_FORMAT_ARGB8888;
      break;
    case kFormatRGBX8888:
      *drm_format = DRM_FORMAT_XBGR8888;
      break;
    case kFormatRGBX8888Ubwc:
      *drm_format = DRM_FORMAT_XBGR8888;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED;
      break;
    case kFormatBGRX8888:
      *drm_format = DRM_FORMAT_XRGB8888;
      break;
    case kFormatRGB888:
      *drm_format = DRM_FORMAT_BGR888;
      break;
    case kFormatBGR888:
      *drm_format = DRM_FORMAT_RGB888;
      break;
    case kFormatRGB565:
      *drm_format = DRM_FORMAT_BGR565;
      break;
    case kFormatBGR565:
      *drm_format = DRM_FORMAT_RGB565;
      break;
    case kFormatBGR565Ubwc:
      *drm_format = DRM_FORMAT_BGR565;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED;
      break;
    case kFormatRGBA1010102:
      *drm_format = DRM_FORMAT_ABGR2101010;
      break;
    case kFormatRGBA1010102Ubwc:
      *drm_format = DRM_FORMAT_ABGR2101010;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED;
      break;
    case kFormatARGB2101010:
      *drm_format = DRM_FORMAT_BGRA1010102;
      break;
    case kFormatRGBX1010102:
      *drm_format = DRM_FORMAT_XBGR2101010;
      break;
    case kFormatRGBX1010102Ubwc:
      *drm_format = DRM_FORMAT_XBGR2101010;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED;
      break;
    case kFormatXRGB2101010:
      *drm_format = DRM_FORMAT_BGRX1010102;
      break;
    case kFormatBGRA1010102:
      *drm_format = DRM_FORMAT_ARGB2101010;
      break;
    case kFormatABGR2101010:
      *drm_format = DRM_FORMAT_RGBA1010102;
      break;
    case kFormatBGRX1010102:
      *drm_format = DRM_FORMAT_XRGB2101010;
      break;
    case kFormatXBGR2101010:
      *drm_format = DRM_FORMAT_RGBX1010102;
      break;
    case kFormatYCbCr420SemiPlanar:
      *drm_format = DRM_FORMAT_NV12;
      break;
    case kFormatYCbCr420SemiPlanarVenus:
      *drm_format = DRM_FORMAT_NV12;
      break;
    case kFormatYCbCr420SPVenusUbwc:
      *drm_format = DRM_FORMAT_NV12;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED;
      break;
    case kFormatYCbCr420SPVenusTile:
      *drm_format = DRM_FORMAT_NV12;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_TILE;
      break;
    case kFormatYCrCb420SemiPlanar:
      *drm_format = DRM_FORMAT_NV21;
      break;
    case kFormatYCrCb420SemiPlanarVenus:
      *drm_format = DRM_FORMAT_NV21;
      break;
    case kFormatYCbCr420P010:
    case kFormatYCbCr420P010Venus:
      *drm_format = DRM_FORMAT_NV12;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_DX;
      break;
    case kFormatYCbCr420P010Ubwc:
      *drm_format = DRM_FORMAT_NV12;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED |
        DRM_FORMAT_MOD_QCOM_DX;
      break;
    case kFormatYCbCr420P010Tile:
      *drm_format = DRM_FORMAT_NV12;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_TILE |
        DRM_FORMAT_MOD_QCOM_DX;
      break;
    case kFormatYCbCr420TP10Ubwc:
      *drm_format = DRM_FORMAT_NV12;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED |
        DRM_FORMAT_MOD_QCOM_DX | DRM_FORMAT_MOD_QCOM_TIGHT;
      break;
    case kFormatYCbCr420TP10Tile:
      *drm_format = DRM_FORMAT_NV12;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_TILE |
        DRM_FORMAT_MOD_QCOM_DX | DRM_FORMAT_MOD_QCOM_TIGHT;
      break;
    case kFormatYCbCr422H2V1SemiPlanar:
      *drm_format = DRM_FORMAT_NV16;
      break;
    case kFormatYCrCb422H2V1SemiPlanar:
      *drm_format = DRM_FORMAT_NV61;
      break;
    case kFormatYCrCb420PlanarStride16:
      *drm_format = DRM_FORMAT_YVU420;
      break;
    default:
      DLOGW("Unsupported format %s", GetFormatString(format));
  }
}

class FrameBufferObject : public LayerBufferObject {
 public:
  explicit FrameBufferObject(uint32_t fb_id, LayerBufferFormat format,
                             uint32_t width, uint32_t height)
    :fb_id_(fb_id), format_(format), width_(width), height_(height) {
  }

  ~FrameBufferObject() {
    DRMMaster *master;
    DRMMaster::GetInstance(&master);
    int ret = master->RemoveFbId(fb_id_);
    if (ret < 0) {
      DLOGE("Removing fb_id %d failed with error %d", fb_id_, errno);
    }
  }
  uint32_t GetFbId() { return fb_id_; }
  bool IsEqual(LayerBufferFormat format, uint32_t width, uint32_t height) {
    return (format == format_ && width == width_ && height == height_);
  }

 private:
  uint32_t fb_id_;
  LayerBufferFormat format_;
  uint32_t width_;
  uint32_t height_;
};

HWDeviceDRM::Registry::Registry(BufferAllocator *buffer_allocator) :
  buffer_allocator_(buffer_allocator) {
  int value = 0;
  if (Debug::GetProperty(DISABLE_FBID_CACHE, &value) == kErrorNone) {
    disable_fbid_cache_ = (value == 1);
  }
}

void HWDeviceDRM::Registry::Register(HWLayers *hw_layers) {
  HWLayersInfo &hw_layer_info = hw_layers->info;
  uint32_t hw_layer_count = UINT32(hw_layer_info.hw_layers.size());

  for (uint32_t i = 0; i < hw_layer_count; i++) {
    Layer &layer = hw_layer_info.hw_layers.at(i);
    LayerBuffer input_buffer = layer.input_buffer;
    HWRotatorSession *hw_rotator_session = &hw_layers->config[i].hw_rotator_session;
    HWRotateInfo *hw_rotate_info = &hw_rotator_session->hw_rotate_info[0];
    fbid_cache_limit_ = input_buffer.flags.video ? VIDEO_FBID_LIMIT : UI_FBID_LIMIT;

    if (hw_rotator_session->mode == kRotatorOffline && hw_rotate_info->valid) {
      input_buffer = hw_rotator_session->output_buffer;
      fbid_cache_limit_ = OFFLINE_ROTATOR_FBID_LIMIT;
    }

    if (input_buffer.flags.interlace) {
      input_buffer.width *= 2;
      input_buffer.height /= 2;
    }
    MapBufferToFbId(&layer, input_buffer);
  }
}

int HWDeviceDRM::Registry::CreateFbId(const LayerBuffer &buffer, uint32_t *fb_id) {
  DRMMaster *master = nullptr;
  DRMMaster::GetInstance(&master);
  int ret = -1;

  if (!master) {
    DLOGE("Failed to acquire DRM Master instance");
    return ret;
  }

  DRMBuffer layout{};
  AllocatedBufferInfo buf_info{};
  buf_info.fd = layout.fd = buffer.planes[0].fd;
  buf_info.aligned_width = layout.width = buffer.width;
  buf_info.aligned_height = layout.height = buffer.height;
  buf_info.format = buffer.format;
  GetDRMFormat(buf_info.format, &layout.drm_format, &layout.drm_format_modifier);
  buffer_allocator_->GetBufferLayout(buf_info, layout.stride, layout.offset, &layout.num_planes);
  ret = master->CreateFbId(layout, fb_id);
  if (ret < 0) {
    DLOGE("CreateFbId failed. width %d, height %d, format: %s, stride %u, error %d",
        layout.width, layout.height, GetFormatString(buf_info.format), layout.stride[0], errno);
  }

  return ret;
}

void HWDeviceDRM::Registry::MapBufferToFbId(Layer* layer, const LayerBuffer &buffer) {
  if (buffer.planes[0].fd < 0) {
    return;
  }

  uint64_t handle_id = buffer.handle_id;
  if (!handle_id || disable_fbid_cache_) {
    // In legacy path, clear fb_id map in each frame.
    layer->buffer_map->buffer_map.clear();
  } else {
    auto it = layer->buffer_map->buffer_map.find(handle_id);
    if (it != layer->buffer_map->buffer_map.end()) {
      FrameBufferObject *fb_obj = static_cast<FrameBufferObject*>(it->second.get());
      if (fb_obj->IsEqual(buffer.format, buffer.width, buffer.height)) {
        // Found fb_id for given handle_id key
        return;
      } else {
        // Erase from fb_id map if format or size have been modified
        layer->buffer_map->buffer_map.erase(it);
      }
    }

    if (layer->buffer_map->buffer_map.size() >= fbid_cache_limit_) {
      // Clear fb_id map, if the size reaches cache limit.
      layer->buffer_map->buffer_map.clear();
    }
  }

  uint32_t fb_id = 0;
  if (CreateFbId(buffer, &fb_id) >= 0) {
    // Create and cache the fb_id in map
    layer->buffer_map->buffer_map[handle_id] = std::make_shared<FrameBufferObject>(fb_id,
        buffer.format, buffer.width, buffer.height);
  }
}

void HWDeviceDRM::Registry::MapOutputBufferToFbId(LayerBuffer *output_buffer) {
  if (output_buffer->planes[0].fd < 0) {
    return;
  }

  uint64_t handle_id = output_buffer->handle_id;
  if (!handle_id || disable_fbid_cache_) {
    // In legacy path, clear output buffer map in each frame.
    output_buffer_map_.clear();
  } else {
    auto it = output_buffer_map_.find(handle_id);
    if (it != output_buffer_map_.end()) {
      FrameBufferObject *fb_obj = static_cast<FrameBufferObject*>(it->second.get());
      if (fb_obj->IsEqual(output_buffer->format, output_buffer->width, output_buffer->height)) {
        return;
      } else {
        output_buffer_map_.erase(it);
      }
    }

    if (output_buffer_map_.size() >= UI_FBID_LIMIT) {
      // Clear output buffer map, if the size reaches cache limit.
      output_buffer_map_.clear();
    }
  }

  uint32_t fb_id = 0;
  if (CreateFbId(*output_buffer, &fb_id) >= 0) {
    output_buffer_map_[handle_id] = std::make_shared<FrameBufferObject>(fb_id,
        output_buffer->format, output_buffer->width, output_buffer->height);
  }
}

void HWDeviceDRM::Registry::Clear() {
  output_buffer_map_.clear();
}

uint32_t HWDeviceDRM::Registry::GetFbId(Layer *layer, uint64_t handle_id) {
  auto it = layer->buffer_map->buffer_map.find(handle_id);
  if (it != layer->buffer_map->buffer_map.end()) {
    FrameBufferObject *fb_obj = static_cast<FrameBufferObject*>(it->second.get());
    return fb_obj->GetFbId();
  }

  return 0;
}

uint32_t HWDeviceDRM::Registry::GetOutputFbId(uint64_t handle_id) {
  auto it = output_buffer_map_.find(handle_id);
  if (it != output_buffer_map_.end()) {
    FrameBufferObject *fb_obj = static_cast<FrameBufferObject*>(it->second.get());
    return fb_obj->GetFbId();
  }

  return 0;
}

HWDeviceDRM::HWDeviceDRM(BufferAllocator *buffer_allocator, HWInfoInterface *hw_info_intf)
    : hw_info_intf_(hw_info_intf), registry_(buffer_allocator) {
  hw_info_intf_ = hw_info_intf;
}

DisplayError HWDeviceDRM::Init() {
  int ret = 0;
  DRMMaster *drm_master = {};
  DRMMaster::GetInstance(&drm_master);
  drm_master->GetHandle(&dev_fd_);
  DRMLibLoader::GetInstance()->FuncGetDRMManager()(dev_fd_, &drm_mgr_intf_);

  if (-1 == display_id_) {
    if (drm_mgr_intf_->RegisterDisplay(disp_type_, &token_)) {
      DLOGE("RegisterDisplay (by type) failed for %s", device_name_);
      return kErrorResources;
    }
  } else if (drm_mgr_intf_->RegisterDisplay(display_id_, &token_)) {
    DLOGE("RegisterDisplay (by id) failed for %s - %d", device_name_, display_id_);
    return kErrorResources;
  }

  if (token_.conn_id > INT32_MAX) {
    DLOGE("Connector id %u beyond supported range", token_.conn_id);
    drm_mgr_intf_->UnregisterDisplay(&token_);
    return kErrorNotSupported;
  }

  display_id_ = static_cast<int32_t>(token_.conn_id);

  ret = drm_mgr_intf_->CreateAtomicReq(token_, &drm_atomic_intf_);
  if (ret) {
    DLOGE("Failed creating atomic request for connector id %u. Error: %d.", token_.conn_id, ret);
    drm_mgr_intf_->UnregisterDisplay(&token_);
    return kErrorResources;
  }

  ret = drm_mgr_intf_->GetConnectorInfo(token_.conn_id, &connector_info_);
  if (ret) {
    DLOGE("Failed getting info for connector id %u. Error: %d.", token_.conn_id, ret);
    drm_mgr_intf_->DestroyAtomicReq(drm_atomic_intf_);
    drm_atomic_intf_ = {};
    drm_mgr_intf_->UnregisterDisplay(&token_);
    return kErrorHardware;
  }

  if (!connector_info_.is_connected || connector_info_.modes.empty()) {
    DLOGW("Device removal detected on connector id %u. Connector status %s and %zu modes.",
          token_.conn_id, connector_info_.is_connected ? "connected":"disconnected",
          connector_info_.modes.size());
    drm_mgr_intf_->DestroyAtomicReq(drm_atomic_intf_);
    drm_atomic_intf_ = {};
    drm_mgr_intf_->UnregisterDisplay(&token_);
    return kErrorDeviceRemoved;
  }

  hw_info_intf_->GetHWResourceInfo(&hw_resource_);

  InitializeConfigs();
  PopulateHWPanelInfo();
  UpdateMixerAttributes();

  // TODO(user): In future, remove has_qseed3 member, add version and pass version to constructor
  if (hw_resource_.has_qseed3) {
    hw_scale_ = new HWScaleDRM(HWScaleDRM::Version::V2);
  }

  std::unique_ptr<HWColorManagerDrm> hw_color_mgr(new HWColorManagerDrm());
  hw_color_mgr_ = std::move(hw_color_mgr);

  return kErrorNone;
}

DisplayError HWDeviceDRM::Deinit() {
  DisplayError err = kErrorNone;
  // first_null_cycle_ = false after the first power-mode NullCommit is completed, or the first
  // frame commit handled the power transition.
  // Power-on will set the CRTC_SET_MODE anytime power mode is not deferred to first commit.
  // Without first commit, if display is disconnected, CRTC_SET_MODE is not set to NULL,
  // this leads to a synchronization issue.
  // So because of previously successful NullCommit, set CRTC_SET_MODE to NULL here for proper sync.

  // first_cycle = false after the first frame commit is completed.
  // A null-commit here is also needed if the first commit has gone through. e.g., If a
  // display is connected and disconnected, HWDeviceDRM::Deinit() may be called
  // before any driver commit happened on the device. The driver may have removed any not-in-use
  // connector (i.e., any connector which did not have a display commit on it and a crtc path
  // setup), so token_.conn_id may have been removed if there was no commit, resulting in
  // drmModeAtomicCommit() failure with ENOENT, 'No such file or directory'.
  if (!first_cycle_ || !first_null_cycle_) {
    ClearSolidfillStages();
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_CRTC, token_.conn_id, 0);
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_POWER_MODE, token_.conn_id, DRMPowerMode::OFF);
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_MODE, token_.crtc_id, nullptr);
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ACTIVE, token_.crtc_id, 0);
#ifdef TRUSTED_VM
    drm_atomic_intf_->Perform(sde_drm::DRMOps::CRTC_SET_VM_REQ_STATE, token_.crtc_id,
                              sde_drm::DRMVMRequestState::RELEASE);
#endif
    int ret = NullCommit(true /* synchronous */, false /* retain_planes */);
    if (ret) {
      DLOGE("Commit failed with error: %d", ret);
      err = kErrorHardware;
    }
  }
  delete hw_scale_;
  registry_.Clear();
  display_attributes_ = {};
  drm_mgr_intf_->DestroyAtomicReq(drm_atomic_intf_);
  drm_atomic_intf_ = {};
  drm_mgr_intf_->UnregisterDisplay(&token_);
  hw_dest_scaler_blocks_used_ -= dest_scaler_blocks_used_;
  return err;
}

DisplayError HWDeviceDRM::GetDisplayId(int32_t *display_id) {
  *display_id = display_id_;
  return kErrorNone;
}

void HWDeviceDRM::InitializeConfigs() {
  current_mode_index_ = 0;
  // Update current mode with preferred mode
  for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
      if (connector_info_.modes[mode_index].mode.type & DRM_MODE_TYPE_PREFERRED) {
        DLOGI("Updating current display mode %d to preferred mode %d.", current_mode_index_,
              mode_index);
        current_mode_index_ = mode_index;
        break;
      }
  }

  display_attributes_.resize(connector_info_.modes.size());

  uint32_t width = connector_info_.modes[current_mode_index_].mode.hdisplay;
  uint32_t height = connector_info_.modes[current_mode_index_].mode.vdisplay;
  for (uint32_t i = 0; i < connector_info_.modes.size(); i++) {
    auto &mode = connector_info_.modes[i].mode;
    if (mode.hdisplay != width || mode.vdisplay != height) {
      resolution_switch_enabled_ = true;
    }
    PopulateDisplayAttributes(i);
  }
  SetDisplaySwitchMode(current_mode_index_);
}

DisplayError HWDeviceDRM::PopulateDisplayAttributes(uint32_t index) {
  drmModeModeInfo mode = {};
  uint32_t mm_width = 0;
  uint32_t mm_height = 0;
  DRMTopology topology = DRMTopology::SINGLE_LM;

  if (default_mode_) {
    DRMResMgr *res_mgr = nullptr;
    int ret = DRMResMgr::GetInstance(&res_mgr);
    if (ret < 0) {
      DLOGE("Failed to acquire DRMResMgr instance");
      return kErrorResources;
    }

    res_mgr->GetMode(&mode);
    res_mgr->GetDisplayDimInMM(&mm_width, &mm_height);
  } else {
    mode = connector_info_.modes[index].mode;
    mm_width = connector_info_.mmWidth;
    mm_height = connector_info_.mmHeight;
    topology = connector_info_.modes[index].topology;
    if (mode.flags & DRM_MODE_FLAG_CMD_MODE_PANEL) {
      display_attributes_[index].smart_panel = true;
    }
  }

  display_attributes_[index].x_pixels = mode.hdisplay;
  display_attributes_[index].y_pixels = mode.vdisplay;
  display_attributes_[index].fps = mode.vrefresh;
  display_attributes_[index].vsync_period_ns =
    UINT32(1000000000L / display_attributes_[index].fps);

  /*
              Active                 Front           Sync           Back
              Region                 Porch                          Porch
     <-----------------------><----------------><-------------><-------------->
     <----- [hv]display ----->
     <------------- [hv]sync_start ------------>
     <--------------------- [hv]sync_end --------------------->
     <-------------------------------- [hv]total ----------------------------->
   */

  display_attributes_[index].v_front_porch = mode.vsync_start - mode.vdisplay;
  display_attributes_[index].v_pulse_width = mode.vsync_end - mode.vsync_start;
  display_attributes_[index].v_back_porch = mode.vtotal - mode.vsync_end;
  display_attributes_[index].v_total = mode.vtotal;
  display_attributes_[index].h_total = mode.htotal;
  display_attributes_[index].is_device_split =
      (topology == DRMTopology::DUAL_LM || topology == DRMTopology::DUAL_LM_MERGE ||
       topology == DRMTopology::DUAL_LM_MERGE_DSC || topology == DRMTopology::DUAL_LM_DSC ||
       topology == DRMTopology::DUAL_LM_DSCMERGE || topology == DRMTopology::QUAD_LM_MERGE ||
       topology == DRMTopology::QUAD_LM_DSCMERGE || topology == DRMTopology::QUAD_LM_MERGE_DSC ||
       topology == DRMTopology::QUAD_LM_DSC4HSMERGE);
  display_attributes_[index].clock_khz = mode.clock;

  // If driver doesn't return panel width/height information, default to 320 dpi
  if (INT(mm_width) <= 0 || INT(mm_height) <= 0) {
    mm_width  = UINT32(((FLOAT(mode.hdisplay) * 25.4f) / 320.0f) + 0.5f);
    mm_height = UINT32(((FLOAT(mode.vdisplay) * 25.4f) / 320.0f) + 0.5f);
    DLOGW("Driver doesn't report panel physical width and height - defaulting to 320dpi");
  }

  display_attributes_[index].x_dpi = (FLOAT(mode.hdisplay) * 25.4f) / FLOAT(mm_width);
  display_attributes_[index].y_dpi = (FLOAT(mode.vdisplay) * 25.4f) / FLOAT(mm_height);
  SetTopology(topology, &display_attributes_[index].topology);

  DLOGI("Display attributes[%d]: WxH: %dx%d, DPI: %fx%f, FPS: %d, LM_SPLIT: %d, V_BACK_PORCH: %d," \
        " V_FRONT_PORCH: %d, V_PULSE_WIDTH: %d, V_TOTAL: %d, H_TOTAL: %d, CLK: %dKHZ," \
        " TOPOLOGY: %d, HW_SPLIT: %d", index, display_attributes_[index].x_pixels,
        display_attributes_[index].y_pixels, display_attributes_[index].x_dpi,
        display_attributes_[index].y_dpi, display_attributes_[index].fps,
        display_attributes_[index].is_device_split, display_attributes_[index].v_back_porch,
        display_attributes_[index].v_front_porch, display_attributes_[index].v_pulse_width,
        display_attributes_[index].v_total, display_attributes_[index].h_total,
        display_attributes_[index].clock_khz, display_attributes_[index].topology,
        mixer_attributes_.split_type);

  return kErrorNone;
}

void HWDeviceDRM::PopulateHWPanelInfo() {
  hw_panel_info_ = {};

  snprintf(hw_panel_info_.panel_name, sizeof(hw_panel_info_.panel_name), "%s",
           connector_info_.panel_name.c_str());

  uint32_t index = current_mode_index_;
  hw_panel_info_.split_info.left_split = display_attributes_[index].x_pixels;
  if (display_attributes_[index].is_device_split) {
    hw_panel_info_.split_info.left_split = hw_panel_info_.split_info.right_split =
        display_attributes_[index].x_pixels / 2;
  }

  hw_panel_info_.partial_update = connector_info_.modes[index].num_roi;
  hw_panel_info_.left_roi_count = UINT32(connector_info_.modes[index].num_roi);
  hw_panel_info_.right_roi_count = UINT32(connector_info_.modes[index].num_roi);
  hw_panel_info_.left_align = connector_info_.modes[index].xstart;
  hw_panel_info_.top_align = connector_info_.modes[index].ystart;
  hw_panel_info_.width_align = connector_info_.modes[index].walign;
  hw_panel_info_.height_align = connector_info_.modes[index].halign;
  hw_panel_info_.min_roi_width = connector_info_.modes[index].wmin;
  hw_panel_info_.min_roi_height = connector_info_.modes[index].hmin;
  hw_panel_info_.needs_roi_merge = connector_info_.modes[index].roi_merge;
  hw_panel_info_.transfer_time_us = connector_info_.modes[index].transfer_time_us;
  hw_panel_info_.allowed_mode_switch = connector_info_.modes[index].allowed_mode_switch;
  hw_panel_info_.dynamic_fps = connector_info_.dynamic_fps;
  hw_panel_info_.qsync_support = connector_info_.qsync_support;
  drmModeModeInfo current_mode = connector_info_.modes[current_mode_index_].mode;
  if (hw_panel_info_.dynamic_fps) {
    uint32_t min_fps = current_mode.vrefresh;
    uint32_t max_fps = current_mode.vrefresh;
    for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
      if ((current_mode.vdisplay == connector_info_.modes[mode_index].mode.vdisplay) &&
          (current_mode.hdisplay == connector_info_.modes[mode_index].mode.hdisplay)) {
        if (min_fps > connector_info_.modes[mode_index].mode.vrefresh)  {
          min_fps = connector_info_.modes[mode_index].mode.vrefresh;
        }
        if (max_fps < connector_info_.modes[mode_index].mode.vrefresh)  {
          max_fps = connector_info_.modes[mode_index].mode.vrefresh;
        }
      }
    }
    hw_panel_info_.min_fps = min_fps;
    hw_panel_info_.max_fps = max_fps;
  } else {
    hw_panel_info_.min_fps = current_mode.vrefresh;
    hw_panel_info_.max_fps = current_mode.vrefresh;
  }

  hw_panel_info_.is_primary_panel = connector_info_.is_primary;
  hw_panel_info_.is_pluggable = 0;
  hw_panel_info_.hdr_enabled = connector_info_.panel_hdr_prop.hdr_enabled;
  // Convert the luminance values to cd/m^2 units.
  hw_panel_info_.peak_luminance = FLOAT(connector_info_.panel_hdr_prop.peak_brightness) / 10000.0f;
  hw_panel_info_.blackness_level = FLOAT(connector_info_.panel_hdr_prop.blackness_level) / 10000.0f;
  hw_panel_info_.average_luminance = FLOAT(connector_info_.panel_hdr_prop.peak_brightness +
                                           connector_info_.panel_hdr_prop.blackness_level) /
                                           (2 * 10000.0f);
  hw_panel_info_.primaries.white_point[0] = connector_info_.panel_hdr_prop.display_primaries[0];
  hw_panel_info_.primaries.white_point[1] = connector_info_.panel_hdr_prop.display_primaries[1];
  hw_panel_info_.primaries.red[0] = connector_info_.panel_hdr_prop.display_primaries[2];
  hw_panel_info_.primaries.red[1] = connector_info_.panel_hdr_prop.display_primaries[3];
  hw_panel_info_.primaries.green[0] = connector_info_.panel_hdr_prop.display_primaries[4];
  hw_panel_info_.primaries.green[1] = connector_info_.panel_hdr_prop.display_primaries[5];
  hw_panel_info_.primaries.blue[0] = connector_info_.panel_hdr_prop.display_primaries[6];
  hw_panel_info_.primaries.blue[1] = connector_info_.panel_hdr_prop.display_primaries[7];
  hw_panel_info_.dyn_bitclk_support = connector_info_.dyn_bitclk_support;

  // no supprt for 90 rotation only flips or 180 supported
  hw_panel_info_.panel_orientation.rotation = 0;
  hw_panel_info_.panel_orientation.flip_horizontal =
    (connector_info_.panel_orientation == DRMRotation::FLIP_H) ||
    (connector_info_.panel_orientation == DRMRotation::ROT_180);
  hw_panel_info_.panel_orientation.flip_vertical =
    (connector_info_.panel_orientation == DRMRotation::FLIP_V) ||
    (connector_info_.panel_orientation == DRMRotation::ROT_180);

  GetHWDisplayPortAndMode();
  GetHWPanelMaxBrightness();

  if (current_mode.flags & DRM_MODE_FLAG_CMD_MODE_PANEL) {
    hw_panel_info_.mode = kModeCommand;
  }
  if (current_mode.flags & DRM_MODE_FLAG_VID_MODE_PANEL) {
    hw_panel_info_.mode = kModeVideo;
  }

  DLOGI_IF(kTagDriverConfig, "%s, Panel Interface = %s, Panel Mode = %s, Is Primary = %d",
           device_name_, interface_str_.c_str(),
           hw_panel_info_.mode == kModeVideo ? "Video" : "Command",
           hw_panel_info_.is_primary_panel);
  DLOGI_IF(kTagDriverConfig, "Partial Update = %d, Dynamic FPS = %d, HDR Panel = %d QSync = %d",
           hw_panel_info_.partial_update, hw_panel_info_.dynamic_fps, hw_panel_info_.hdr_enabled,
           hw_panel_info_.qsync_support);
  DLOGI_IF(kTagDriverConfig, "Align: left = %d, width = %d, top = %d, height = %d",
           hw_panel_info_.left_align, hw_panel_info_.width_align, hw_panel_info_.top_align,
           hw_panel_info_.height_align);
  DLOGI_IF(kTagDriverConfig, "ROI: min_width = %d, min_height = %d, need_merge = %d",
           hw_panel_info_.min_roi_width, hw_panel_info_.min_roi_height,
           hw_panel_info_.needs_roi_merge);
  DLOGI_IF(kTagDriverConfig, "FPS: min = %d, max = %d", hw_panel_info_.min_fps,
           hw_panel_info_.max_fps);
  DLOGI_IF(kTagDriverConfig, "Left Split = %d, Right Split = %d",
           hw_panel_info_.split_info.left_split, hw_panel_info_.split_info.right_split);
  DLOGI_IF(kTagDriverConfig, "Panel Transfer time = %d us", hw_panel_info_.transfer_time_us);
  DLOGI_IF(kTagDriverConfig, "Dynamic Bit Clk Support = %d", hw_panel_info_.dyn_bitclk_support);
}

DisplayError HWDeviceDRM::GetDisplayIdentificationData(uint8_t *out_port, uint32_t *out_data_size,
                                                       uint8_t *out_data) {
  *out_port = token_.hw_port;
  std::vector<uint8_t> &edid = connector_info_.edid;

  if (out_data == nullptr) {
    *out_data_size = (uint32_t)(edid.size());
    if (*out_data_size == 0) {
      DLOGE("EDID blob is empty, no data to return");
      return kErrorDriverData;
    }
  } else {
    *out_data_size = std::min(*out_data_size, (uint32_t)(edid.size()));
    memcpy(out_data, edid.data(), *out_data_size);
  }

  return kErrorNone;
}

void HWDeviceDRM::GetHWDisplayPortAndMode() {
  hw_panel_info_.port = kPortDefault;
  hw_panel_info_.mode =
      (connector_info_.panel_mode == sde_drm::DRMPanelMode::VIDEO) ? kModeVideo : kModeCommand;

  if (default_mode_) {
    return;
  }

  switch (connector_info_.type) {
    case DRM_MODE_CONNECTOR_DSI:
      hw_panel_info_.port = kPortDSI;
      interface_str_ = "DSI";
      break;
    case DRM_MODE_CONNECTOR_LVDS:
      hw_panel_info_.port = kPortLVDS;
      interface_str_ = "LVDS";
      break;
    case DRM_MODE_CONNECTOR_eDP:
      hw_panel_info_.port = kPortEDP;
      interface_str_ = "EDP";
      break;
    case DRM_MODE_CONNECTOR_TV:
    case DRM_MODE_CONNECTOR_HDMIA:
    case DRM_MODE_CONNECTOR_HDMIB:
      hw_panel_info_.port = kPortDTV;
      interface_str_ = "HDMI";
      break;
    case DRM_MODE_CONNECTOR_VIRTUAL:
      hw_panel_info_.port = kPortWriteBack;
      interface_str_ = "Virtual";
      break;
    case DRM_MODE_CONNECTOR_DisplayPort:
      hw_panel_info_.port = kPortDP;
      interface_str_ = "DisplayPort";
      break;
  }

  return;
}

DisplayError HWDeviceDRM::GetActiveConfig(uint32_t *active_config) {
  *active_config = current_mode_index_;
  return kErrorNone;
}

DisplayError HWDeviceDRM::GetNumDisplayAttributes(uint32_t *count) {
  *count = UINT32(display_attributes_.size());
  return kErrorNone;
}

DisplayError HWDeviceDRM::GetDisplayAttributes(uint32_t index,
                                               HWDisplayAttributes *display_attributes) {
  if (index >= display_attributes_.size()) {
    return kErrorParameters;
  }
  *display_attributes = display_attributes_[index];
  return kErrorNone;
}

DisplayError HWDeviceDRM::GetHWPanelInfo(HWPanelInfo *panel_info) {
  *panel_info = hw_panel_info_;
  return kErrorNone;
}

void HWDeviceDRM::SetDisplaySwitchMode(uint32_t index) {
  uint32_t mode_flag = 0;
  uint32_t curr_mode_flag = 0, switch_mode_flag = 0;
  drmModeModeInfo to_set = connector_info_.modes[index].mode;
  drmModeModeInfo current_mode = connector_info_.modes[current_mode_index_].mode;
  uint64_t current_bit_clk = connector_info_.modes[current_mode_index_].bit_clk_rate;
  uint32_t switch_index  = 0;

  if (to_set.flags & DRM_MODE_FLAG_CMD_MODE_PANEL) {
    mode_flag = DRM_MODE_FLAG_CMD_MODE_PANEL;
    switch_mode_flag = DRM_MODE_FLAG_VID_MODE_PANEL;
  } else if (to_set.flags & DRM_MODE_FLAG_VID_MODE_PANEL) {
    mode_flag = DRM_MODE_FLAG_VID_MODE_PANEL;
    switch_mode_flag = DRM_MODE_FLAG_CMD_MODE_PANEL;
  }

  if (current_mode.flags & DRM_MODE_FLAG_CMD_MODE_PANEL) {
    curr_mode_flag = DRM_MODE_FLAG_CMD_MODE_PANEL;
  } else if (current_mode.flags & DRM_MODE_FLAG_VID_MODE_PANEL) {
    curr_mode_flag = DRM_MODE_FLAG_VID_MODE_PANEL;
  }

  if (curr_mode_flag != mode_flag) {
    panel_mode_changed_ = mode_flag;
  }

  for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
    if ((to_set.vdisplay == connector_info_.modes[mode_index].mode.vdisplay) &&
        (to_set.hdisplay == connector_info_.modes[mode_index].mode.hdisplay) &&
        (to_set.vrefresh == connector_info_.modes[mode_index].mode.vrefresh) &&
        (current_bit_clk == connector_info_.modes[mode_index].bit_clk_rate) &&
        (mode_flag & connector_info_.modes[mode_index].mode.flags)) {
      index = mode_index;
      break;
    }
  }

  current_mode_index_ = index;

  switch_mode_valid_ = false;
  for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
    if ((to_set.vdisplay == connector_info_.modes[mode_index].mode.vdisplay) &&
        (to_set.hdisplay == connector_info_.modes[mode_index].mode.hdisplay) &&
        (to_set.vrefresh == connector_info_.modes[mode_index].mode.vrefresh) &&
        (switch_mode_flag & connector_info_.modes[mode_index].mode.flags)) {
      switch_index = mode_index;
      switch_mode_valid_ = true;
      break;
    }
  }

  if (!switch_mode_valid_) {
    // in case there is no corresponding switch mode with same fps, try for a switch
    // mode with lowest fps. This is to handle cases where there are multiple video mode fps
    // but only one command mode for doze like 30 fps.
    uint32_t refresh_rate = 0;
    for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
      if ((to_set.vdisplay == connector_info_.modes[mode_index].mode.vdisplay) &&
          (to_set.hdisplay == connector_info_.modes[mode_index].mode.hdisplay) &&
          (switch_mode_flag & connector_info_.modes[mode_index].mode.flags)) {
        if (!refresh_rate || (refresh_rate > connector_info_.modes[mode_index].mode.vrefresh)) {
          switch_index = mode_index;
          switch_mode_valid_ = true;
          refresh_rate = connector_info_.modes[mode_index].mode.vrefresh;
        }
      }
    }
  }

  if (switch_mode_valid_) {
    if (mode_flag & DRM_MODE_FLAG_VID_MODE_PANEL) {
      video_mode_index_ = current_mode_index_;
      cmd_mode_index_ = switch_index;
    } else {
      video_mode_index_ = switch_index;
      cmd_mode_index_ = current_mode_index_;
    }
  }
  if (current_mode.hdisplay == to_set.hdisplay &&
    current_mode.vdisplay == to_set.vdisplay) {
    seamless_mode_switch_ = true;
  }
}

DisplayError HWDeviceDRM::SetDisplayAttributes(uint32_t index) {
  if (index >= display_attributes_.size()) {
    DLOGE("Invalid mode index %d mode size %d", index, UINT32(display_attributes_.size()));
    return kErrorParameters;
  }

  SetDisplaySwitchMode(index);
  PopulateHWPanelInfo();
  UpdateMixerAttributes();

  DLOGI_IF(kTagDriverConfig,
        "Display attributes[%d]: WxH: %dx%d, DPI: %fx%f, FPS: %d, LM_SPLIT: %d, V_BACK_PORCH: %d," \
        " V_FRONT_PORCH: %d, V_PULSE_WIDTH: %d, V_TOTAL: %d, H_TOTAL: %d, CLK: %dKHZ, " \
        "TOPOLOGY: %d, PanelMode %s", index, display_attributes_[index].x_pixels,
        display_attributes_[index].y_pixels, display_attributes_[index].x_dpi,
        display_attributes_[index].y_dpi, display_attributes_[index].fps,
        display_attributes_[index].is_device_split, display_attributes_[index].v_back_porch,
        display_attributes_[index].v_front_porch, display_attributes_[index].v_pulse_width,
        display_attributes_[index].v_total, display_attributes_[index].h_total,
        display_attributes_[index].clock_khz, display_attributes_[index].topology,
        (connector_info_.modes[index].mode.flags & DRM_MODE_FLAG_VID_MODE_PANEL) ?
        "Video" : "Command");

  return kErrorNone;
}

DisplayError HWDeviceDRM::SetDisplayAttributes(const HWDisplayAttributes &display_attributes) {
  return kErrorNotSupported;
}

DisplayError HWDeviceDRM::GetConfigIndex(char *mode, uint32_t *index) {
  return kErrorNone;
}

DisplayError HWDeviceDRM::PowerOn(const HWQosData &qos_data, shared_ptr<Fence> *release_fence) {
  SetQOSData(qos_data);

  if (tui_state_ != kTUIStateNone) {
    DLOGI("Request deferred TUI state %d", tui_state_);
    pending_power_state_ = kPowerStateOn;
    return kErrorDeferred;
  }

  int64_t release_fence_fd = -1;
  int64_t retire_fence_fd = -1;

  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ACTIVE, token_.crtc_id, 1);
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_POWER_MODE, token_.conn_id, DRMPowerMode::ON);
  if (release_fence) {
    drm_atomic_intf_->Perform(DRMOps::CRTC_GET_RELEASE_FENCE, token_.crtc_id, &release_fence_fd);
  }
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_GET_RETIRE_FENCE, token_.conn_id, &retire_fence_fd);

  int ret = NullCommit(false /* synchronous */, true /* retain_planes */);
  if (ret) {
    DLOGE("Failed with error: %d", ret);
    return kErrorHardware;
  }

  shared_ptr<Fence> retire_fence = Fence::Create(INT(retire_fence_fd), "retire_power_on");

  if (release_fence) {
    *release_fence = Fence::Create(INT(release_fence_fd), "release_power_on");
    DLOGD_IF(kTagDriverConfig, "RELEASE fence: fd: %s", Fence::GetStr(*release_fence).c_str());
  }
  pending_power_state_ = kPowerStateNone;

  Fence::Wait(retire_fence, kTimeoutMsPowerOn);

  last_power_mode_ = DRMPowerMode::ON;

  return kErrorNone;
}

DisplayError HWDeviceDRM::PowerOff(bool teardown) {
  DTRACE_SCOPED();
  if (!drm_atomic_intf_) {
    DLOGE("DRM Atomic Interface is null!");
    return kErrorUndefined;
  }

  if (first_cycle_) {
    return kErrorNone;
  }

  if (tui_state_ != kTUIStateNone && tui_state_ != kTUIStateEnd) {
    DLOGI("Request deferred TUI state %d", tui_state_);
    pending_power_state_ = kPowerStateOff;
    return kErrorDeferred;
  }

  ResetROI();
  int64_t retire_fence_fd = -1;
  drmModeModeInfo current_mode = connector_info_.modes[current_mode_index_].mode;
  if (!IsSeamlessTransition()) {
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_MODE, token_.crtc_id, &current_mode);
  }
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_POWER_MODE, token_.conn_id, DRMPowerMode::OFF);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ACTIVE, token_.crtc_id, 0);
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_GET_RETIRE_FENCE, token_.conn_id, &retire_fence_fd);

  int ret = NullCommit(false /* synchronous */, false /* retain_planes */);
  if (ret) {
    DLOGE("Failed with error: %d, dynamic_fps=%d, seamless_mode_switch_=%d, vrefresh_=%d,"
     "panel_mode_changed_=%d bit_clk_rate_=%" PRIu64 , ret, hw_panel_info_.dynamic_fps,
     seamless_mode_switch_, vrefresh_, panel_mode_changed_, bit_clk_rate_);
    return kErrorHardware;
  }

  shared_ptr<Fence> retire_fence = Fence::Create(INT(retire_fence_fd), "retire_power_off");
  pending_power_state_ = kPowerStateNone;

  Fence::Wait(retire_fence, kTimeoutMsPowerOff);

  last_power_mode_ = DRMPowerMode::OFF;

  return kErrorNone;
}

DisplayError HWDeviceDRM::Doze(const HWQosData &qos_data, shared_ptr<Fence> *release_fence) {
  DTRACE_SCOPED();

  if (first_cycle_ || tui_state_ != kTUIStateNone || last_power_mode_ != DRMPowerMode::OFF) {
    pending_power_state_ = kPowerStateDoze;
    return kErrorDeferred;
  }

  SetQOSData(qos_data);

  int64_t release_fence_fd = -1;
  int64_t retire_fence_fd = -1;

  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_CRTC, token_.conn_id, token_.crtc_id);
  drmModeModeInfo current_mode = connector_info_.modes[current_mode_index_].mode;
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_MODE, token_.crtc_id, &current_mode);

  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ACTIVE, token_.crtc_id, 1);
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_POWER_MODE, token_.conn_id, DRMPowerMode::DOZE);
  if (release_fence) {
    drm_atomic_intf_->Perform(DRMOps::CRTC_GET_RELEASE_FENCE, token_.crtc_id, &release_fence_fd);
  }
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_GET_RETIRE_FENCE, token_.conn_id, &retire_fence_fd);

  int ret = NullCommit(false /* synchronous */, true /* retain_planes */);
  if (ret) {
    DLOGE("Failed with error: %d", ret);
    return kErrorHardware;
  }

  shared_ptr<Fence> retire_fence = Fence::Create(INT(retire_fence_fd), "retire_doze");

  if (release_fence) {
    *release_fence = Fence::Create(release_fence_fd, "release_doze");
    DLOGD_IF(kTagDriverConfig, "RELEASE fence: fd: %s", Fence::GetStr(*release_fence).c_str());
  }

  Fence::Wait(retire_fence, kTimeoutMsDoze);

  last_power_mode_ = DRMPowerMode::DOZE;

  return kErrorNone;
}

DisplayError HWDeviceDRM::DozeSuspend(const HWQosData &qos_data,
                                      shared_ptr<Fence> *release_fence) {
  DTRACE_SCOPED();

  if (tui_state_ != kTUIStateNone && tui_state_ != kTUIStateEnd) {
    pending_power_state_ = kPowerStateDozeSuspend;
    return kErrorDeferred;
  }

  SetQOSData(qos_data);

  int64_t release_fence_fd = -1;
  int64_t retire_fence_fd = -1;

  if (first_cycle_) {
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_CRTC, token_.conn_id, token_.crtc_id);
    drmModeModeInfo current_mode = connector_info_.modes[current_mode_index_].mode;
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_MODE, token_.crtc_id, &current_mode);
  }
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ACTIVE, token_.crtc_id, 1);
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_POWER_MODE, token_.conn_id,
                            DRMPowerMode::DOZE_SUSPEND);
  if (release_fence) {
    drm_atomic_intf_->Perform(DRMOps::CRTC_GET_RELEASE_FENCE, token_.crtc_id, &release_fence_fd);
  }
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_GET_RETIRE_FENCE, token_.conn_id, &retire_fence_fd);

  int ret = NullCommit(false /* synchronous */, true /* retain_planes */);
  if (ret) {
    DLOGE("Failed with error: %d", ret);
    return kErrorHardware;
  }

  shared_ptr<Fence> retire_fence = Fence::Create(INT(retire_fence_fd), "retire_doze_suspend");

  if (release_fence) {
    *release_fence = Fence::Create(release_fence_fd, "release_doze_suspend");
    DLOGD_IF(kTagDriverConfig, "RELEASE fence: fd: %s", Fence::GetStr(*release_fence).c_str());
  }
  pending_power_state_ = kPowerStateNone;

  Fence::Wait(retire_fence, kTimeoutMsDozeSuspend);

  last_power_mode_ = DRMPowerMode::DOZE_SUSPEND;

  return kErrorNone;
}

void HWDeviceDRM::SetQOSData(const HWQosData &qos_data) {
  if (!qos_data.valid) {
    return;
  }
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_CORE_CLK, token_.crtc_id, qos_data.clock_hz);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_CORE_AB, token_.crtc_id, qos_data.core_ab_bps);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_CORE_IB, token_.crtc_id, qos_data.core_ib_bps);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_LLCC_AB, token_.crtc_id, qos_data.llcc_ab_bps);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_LLCC_IB, token_.crtc_id, qos_data.llcc_ib_bps);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_DRAM_AB, token_.crtc_id, qos_data.dram_ab_bps);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_DRAM_IB, token_.crtc_id, qos_data.dram_ib_bps);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ROT_PREFILL_BW, token_.crtc_id,
                            qos_data.rot_prefill_bw_bps);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ROT_CLK, token_.crtc_id, qos_data.rot_clock_hz);
}

DisplayError HWDeviceDRM::Standby() {
  return kErrorNone;
}

void HWDeviceDRM::SetupAtomic(Fence::ScopedRef &scoped_ref, HWLayers *hw_layers, bool validate,
                              int64_t *release_fence_fd, int64_t *retire_fence_fd) {
  if (default_mode_) {
    return;
  }

  DTRACE_SCOPED();
  HWLayersInfo &hw_layer_info = hw_layers->info;
  uint32_t hw_layer_count = UINT32(hw_layer_info.hw_layers.size());
  HWQosData &qos_data = hw_layers->qos_data;
  DRMSecurityLevel crtc_security_level = DRMSecurityLevel::SECURE_NON_SECURE;
  uint32_t index = current_mode_index_;
  drmModeModeInfo current_mode = connector_info_.modes[index].mode;
  uint64_t current_bit_clk = connector_info_.modes[index].bit_clk_rate;

  solid_fills_.clear();
  bool resource_update = hw_layers->updates_mask.test(kUpdateResources);
  bool buffer_update = hw_layers->updates_mask.test(kSwapBuffers);
  bool update_config = resource_update || buffer_update || tui_state_ == kTUIStateEnd ||
                       hw_layer_info.stack->flags.geometry_changed;

  if (hw_panel_info_.partial_update && update_config) {
    if (IsFullFrameUpdate(hw_layer_info)) {
      ResetROI();
    } else {
      const int kNumMaxROIs = 4;
      DRMRect crtc_rects[kNumMaxROIs] = {{0, 0, mixer_attributes_.width, mixer_attributes_.height}};
      DRMRect conn_rects[kNumMaxROIs] = {{0, 0, display_attributes_[index].x_pixels,
                                          display_attributes_[index].y_pixels}};

      for (uint32_t i = 0; i < hw_layer_info.left_frame_roi.size(); i++) {
        auto &roi = hw_layer_info.left_frame_roi.at(i);
        // TODO(user): In multi PU, stitch ROIs vertically adjacent and upate plane destination
        crtc_rects[i].left = UINT32(roi.left);
        crtc_rects[i].right = UINT32(roi.right);
        crtc_rects[i].top = UINT32(roi.top);
        crtc_rects[i].bottom = UINT32(roi.bottom);
        conn_rects[i].left = UINT32(roi.left);
        conn_rects[i].right = UINT32(roi.right);
        conn_rects[i].top = UINT32(roi.top);
        conn_rects[i].bottom = UINT32(roi.bottom);
      }

      uint32_t num_rects = std::max(1u, static_cast<uint32_t>(hw_layer_info.left_frame_roi.size()));
      drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ROI, token_.crtc_id, num_rects, crtc_rects);
      drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_ROI, token_.conn_id, num_rects, conn_rects);
    }
  }
  // Fix pipe lut clearing issue during composer stat stop or crash.
  // Todo(user): lut clearing should be handled by driver
  // during composer crash.
  if (planes_reset_cache_) {
    drm_atomic_intf_->Perform(sde_drm::DRMOps::PLANES_RESET_LUT, token_.crtc_id);
    planes_reset_cache_ = false;
  }

  for (uint32_t i = 0; i < hw_layer_count; i++) {
    Layer &layer = hw_layer_info.hw_layers.at(i);
    LayerBuffer *input_buffer = &layer.input_buffer;
    HWPipeInfo *left_pipe = &hw_layers->config[i].left_pipe;
    HWPipeInfo *right_pipe = &hw_layers->config[i].right_pipe;
    HWLayerConfig &layer_config = hw_layers->config[i];
    HWRotatorSession *hw_rotator_session = &layer_config.hw_rotator_session;

    if (hw_layers->config[i].use_solidfill_stage) {
      hw_layers->config[i].hw_solidfill_stage.solid_fill_info = layer.solid_fill_info;
      AddSolidfillStage(hw_layers->config[i].hw_solidfill_stage, layer.plane_alpha);
      continue;
    }

    for (uint32_t count = 0; count < 2; count++) {
      HWPipeInfo *pipe_info = (count == 0) ? left_pipe : right_pipe;
      HWRotateInfo *hw_rotate_info = &hw_rotator_session->hw_rotate_info[count];

      if (hw_rotator_session->mode == kRotatorOffline && hw_rotate_info->valid) {
        input_buffer = &hw_rotator_session->output_buffer;
      }

      uint32_t fb_id = registry_.GetFbId(&layer, input_buffer->handle_id);

      if (pipe_info->valid && fb_id) {
        uint32_t pipe_id = pipe_info->pipe_id;

        if (update_config) {
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_ALPHA, pipe_id, layer.plane_alpha);

          // IchthysOS: the FOD bit is deliberately NOT put on the plane's zpos. On this kernel it
          // makes the mixer insert its own full-screen dim layer under the plane (sde_crtc
          // fod_dim_lut), on top of SystemUI's nits-matched dim. The FOD layer is reported through
          // the connector's fingerprint_mask instead (SetupFingerprintMask), which also works when
          // the layer was composed by the GPU and has no plane at all.
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_ZORDER, pipe_id, pipe_info->z_order);

          // Account for PMA block activation directly at translation time to preserve layer
          // blending definition and avoid issues when a layer structure is reused.
          DRMBlendType blending = DRMBlendType::UNDEFINED;
          LayerBlending layer_blend = layer.blending;
          if (layer_blend ==  kBlendingPremultiplied && pipe_info->inverse_pma_info.inverse_pma) {
            layer_blend = kBlendingCoverage;
            DLOGI_IF(kTagDriverConfig, "PMA handled by Inverse PMA block - Pipe id: %u", pipe_id);
          }
          SetBlending(layer_blend, &blending);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_BLEND_TYPE, pipe_id, blending);

          DRMRect src = {};
          SetRect(pipe_info->src_roi, &src);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_SRC_RECT, pipe_id, src);

          DRMRect dst = {};
          SetRect(pipe_info->dst_roi, &dst);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_DST_RECT, pipe_id, dst);

          DRMRect excl = {};
          SetRect(pipe_info->excl_rect, &excl);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_EXCL_RECT, pipe_id, excl);

          uint32_t rot_bit_mask = 0;
          SetRotation(layer.transform, layer_config, &rot_bit_mask);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_ROTATION, pipe_id, rot_bit_mask);

          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_H_DECIMATION, pipe_id,
                                    pipe_info->horizontal_decimation);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_V_DECIMATION, pipe_id,
                                    pipe_info->vertical_decimation);

          DRMSecureMode fb_secure_mode;
          DRMSecurityLevel security_level;
          SetSecureConfig(layer.input_buffer, &fb_secure_mode, &security_level);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_FB_SECURE_MODE, pipe_id, fb_secure_mode);
          if (security_level > crtc_security_level) {
            crtc_security_level = security_level;
          }

          uint32_t config = 0;
          SetSrcConfig(layer.input_buffer, hw_rotator_session->mode, &config);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_SRC_CONFIG, pipe_id, config);;

          if (hw_scale_) {
            SDEScaler scaler_output = {};
            hw_scale_->SetScaler(pipe_info->scale_data, &scaler_output);
            // TODO(user): Remove qseed3 and add version check, then send appropriate scaler object
            if (hw_resource_.has_qseed3) {
              drm_atomic_intf_->Perform(DRMOps::PLANE_SET_SCALER_CONFIG, pipe_id,
                                        reinterpret_cast<uint64_t>(&scaler_output.scaler_v2));
            }
          }

          DRMCscType csc_type = DRMCscType::kCscTypeMax;
          SelectCscType(layer.input_buffer, &csc_type);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_CSC_CONFIG, pipe_id, &csc_type);

          DRMMultiRectMode multirect_mode;
          SetMultiRectMode(pipe_info->flags, &multirect_mode);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_MULTIRECT_MODE, pipe_id, multirect_mode);

          SetSsppTonemapFeatures(pipe_info);
        }

        drm_atomic_intf_->Perform(DRMOps::PLANE_SET_FB_ID, pipe_id, fb_id);
        drm_atomic_intf_->Perform(DRMOps::PLANE_SET_CRTC, pipe_id, token_.crtc_id);

        if (!validate && input_buffer->acquire_fence) {
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_INPUT_FENCE, pipe_id,
                                    scoped_ref.Get(input_buffer->acquire_fence));
        }
      }
    }
  }

  if (update_config) {
    SetSolidfillStages();
    SetQOSData(qos_data);
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_SECURITY_LEVEL, token_.crtc_id, crtc_security_level);
  }

  if (hw_layers->hw_avr_info.update) {
    sde_drm::DRMQsyncMode mode = sde_drm::DRMQsyncMode::NONE;
    if (hw_layers->hw_avr_info.mode == kContinuousMode) {
      mode = sde_drm::DRMQsyncMode::CONTINUOUS;
    } else if (hw_layers->hw_avr_info.mode == kOneShotMode) {
      mode = sde_drm::DRMQsyncMode::ONESHOT;
    }
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_QSYNC_MODE, token_.conn_id, mode);
  }

  SetupFingerprintMask(hw_layer_info, validate);

  drm_atomic_intf_->Perform(DRMOps::DPPS_COMMIT_FEATURE, 0 /* argument is not used */);
  drm_atomic_intf_->Perform(DRMOps::COMMIT_PANEL_FEATURES, 0 /* argument is not used */);

  if (reset_output_fence_offset_ && !validate) {
    // Change back the fence_offset
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_OUTPUT_FENCE_OFFSET, token_.crtc_id, 0);
    reset_output_fence_offset_ = false;
  }

  // Set panel mode
  if (panel_mode_changed_ & DRM_MODE_FLAG_VID_MODE_PANEL)  {
    if (!validate) {
      // Switch to video mode, corresponding change the fence_offset
      drm_atomic_intf_->Perform(DRMOps::CRTC_SET_OUTPUT_FENCE_OFFSET, token_.crtc_id, 1);
    }
    ResetROI();
  }

  if (!validate && release_fence_fd && retire_fence_fd) {
    drm_atomic_intf_->Perform(DRMOps::CRTC_GET_RELEASE_FENCE, token_.crtc_id, release_fence_fd);
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_GET_RETIRE_FENCE, token_.conn_id, retire_fence_fd);
  }

  DLOGI_IF(kTagDriverConfig, "%s::%s System Clock=%d Hz, Core: AB=%f KBps, IB=%f Bps, " \
           "LLCC: AB=%f Bps, IB=%f Bps, DRAM AB=%f Bps, IB=%f Bps, "\
           "Rot: Bw=%f Bps, Clock=%d Hz", validate ? "Validate" : "Commit", device_name_,
           qos_data.clock_hz, qos_data.core_ab_bps / 1000.f, qos_data.core_ib_bps / 1000.f,
           qos_data.llcc_ab_bps / 1000.f, qos_data.llcc_ib_bps / 1000.f,
           qos_data.dram_ab_bps / 1000.f, qos_data.dram_ib_bps / 1000.f,
           qos_data.rot_prefill_bw_bps / 1000.f, qos_data.rot_clock_hz);

  // Set refresh rate
  if (vrefresh_) {
    for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
      if ((current_mode.vdisplay == connector_info_.modes[mode_index].mode.vdisplay) &&
          (current_mode.hdisplay == connector_info_.modes[mode_index].mode.hdisplay) &&
          (current_bit_clk == connector_info_.modes[mode_index].bit_clk_rate) &&
          (current_mode.flags == connector_info_.modes[mode_index].mode.flags) &&
          (vrefresh_ == connector_info_.modes[mode_index].mode.vrefresh)) {
        current_mode = connector_info_.modes[mode_index].mode;
        break;
      }
    }
  }

  if (bit_clk_rate_) {
    for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
      if ((current_mode.vdisplay == connector_info_.modes[mode_index].mode.vdisplay) &&
          (current_mode.hdisplay == connector_info_.modes[mode_index].mode.hdisplay) &&
          (current_mode.vrefresh == connector_info_.modes[mode_index].mode.vrefresh) &&
          (current_mode.flags == connector_info_.modes[mode_index].mode.flags) &&
          (bit_clk_rate_ == connector_info_.modes[mode_index].bit_clk_rate)) {
        current_mode = connector_info_.modes[mode_index].mode;
        break;
      }
    }
  }

  if (first_cycle_) {
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_TOPOLOGY_CONTROL, token_.conn_id,
                              topology_control_);
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ACTIVE, token_.crtc_id, 1);
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_CRTC, token_.conn_id, token_.crtc_id);
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_POWER_MODE, token_.conn_id, DRMPowerMode::ON);
#ifdef TRUSTED_VM
    drm_atomic_intf_->Perform(sde_drm::DRMOps::CRTC_SET_VM_REQ_STATE, token_.crtc_id,
                              sde_drm::DRMVMRequestState::ACQUIRE);
#endif
    last_power_mode_ = DRMPowerMode::ON;
  } else if (pending_power_state_ != kPowerStateNone && !validate) {
    DRMPowerMode power_mode;
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ACTIVE, token_.crtc_id, 1);
    if (GetDRMPowerMode(pending_power_state_, &power_mode) == kErrorNone) {
      drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_POWER_MODE, token_.conn_id, power_mode);
      last_power_mode_ = power_mode;
    }
  }

  // Set CRTC mode, only if display config changes
  if (first_cycle_ || vrefresh_ || update_mode_ || panel_mode_changed_) {
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_MODE, token_.crtc_id, &current_mode);
  }

  if (!validate && (hw_layer_info.set_idle_time_ms >= 0)) {
    DLOGI_IF(kTagDriverConfig, "Setting idle timeout to = %d ms",
             hw_layer_info.set_idle_time_ms);
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_IDLE_TIMEOUT, token_.crtc_id,
                              hw_layer_info.set_idle_time_ms);
  }

  if (hw_panel_info_.mode == kModeCommand) {
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_AUTOREFRESH, token_.conn_id, autorefresh_);
  }
}

void HWDeviceDRM::AddSolidfillStage(const HWSolidfillStage &sf, uint32_t plane_alpha) {
  sde_drm::DRMSolidfillStage solidfill;
  solidfill.bounding_rect.left = UINT32(sf.roi.left);
  solidfill.bounding_rect.top = UINT32(sf.roi.top);
  solidfill.bounding_rect.right = UINT32(sf.roi.right);
  solidfill.bounding_rect.bottom = UINT32(sf.roi.bottom);
  solidfill.is_exclusion_rect  = sf.is_exclusion_rect;
  solidfill.plane_alpha = plane_alpha;
  solidfill.z_order = sf.z_order;
  if (!sf.solid_fill_info.bit_depth) {
    solidfill.color_bit_depth = 8;
    solidfill.alpha = (0xff000000 & sf.color) >> 24;
    solidfill.red = (0xff0000 & sf.color) >> 16;
    solidfill.green = (0xff00 & sf.color) >> 8;
    solidfill.blue = 0xff & sf.color;
  } else {
    solidfill.color_bit_depth = sf.solid_fill_info.bit_depth;
    solidfill.alpha = sf.solid_fill_info.alpha;
    solidfill.red = sf.solid_fill_info.red;
    solidfill.green = sf.solid_fill_info.green;
    solidfill.blue = sf.solid_fill_info.blue;
  }
  solid_fills_.push_back(solidfill);
  DLOGI_IF(kTagDriverConfig, "Add a solidfill stage at z_order:%d argb_color:%x plane_alpha:%x",
           solidfill.z_order, solidfill.color, solidfill.plane_alpha);
}

void HWDeviceDRM::SetSolidfillStages() {
  if (hw_resource_.num_solidfill_stages) {
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_SOLIDFILL_STAGES, token_.crtc_id,
                              reinterpret_cast<uint64_t> (&solid_fills_));
  }
}

void HWDeviceDRM::ClearSolidfillStages() {
  solid_fills_.clear();
  SetSolidfillStages();
}

#ifdef UDFPS_ZPOS
// IchthysOS: frame-synced UDFPS mask HBM (Samsung optical fingerprint panels).
// persist.ichthys.udfps_sync (bool, default false) switches it at runtime, read through the
// property's serial so the per-frame cost is one atomic load. SystemUI and the fingerprint HAL read
// the same switch.
static bool IchthysUdfpsSyncEnabled() {
  static const prop_info *info = nullptr;
  static uint32_t last_serial = 0;
  static bool enabled = false;
  if (!info) {
    info = __system_property_find("persist.ichthys.udfps_sync");
    if (!info) {
      return false;
    }
    last_serial = __system_property_serial(info) + 1;  // force the first read
  }
  uint32_t serial = __system_property_serial(info);
  if (serial != last_serial) {
    last_serial = serial;
    __system_property_read_callback(info, [](void *cookie, const char *, const char *value,
                                             uint32_t) {
      *static_cast<bool *>(cookie) = !strcmp(value, "1") || !strcmp(value, "true");
    }, &enabled);
  }
  return enabled;
}

// IchthysOS: the mask HBM level for the kernel's fingerprint_mask connector property: a value > 1
// is taken as the mask level itself (sde_connector.c pre_kickoff), 1 = its default (331).
// persist.ichthys.udfps_mask_level = a level 256..486 (the panel's HBM range); anything else = 1.
// UDFPS_MASK_LEVEL_BEGIN
static uint32_t IchthysUdfpsMaskLevelValue(const char *value) {
  const char *s = value;
  if (*s < '0' || *s > '9') {
    return 1;
  }
  char *stop = nullptr;
  long v = strtol(s, &stop, 10);
  if (!stop || *stop || v < 256 || v > 486) {
    return 1;
  }
  return static_cast<uint32_t>(v);
}
// UDFPS_MASK_LEVEL_END

// IchthysOS: persist.ichthys.udfps_mask_level through its serial (one atomic load when unchanged).
static uint32_t IchthysUdfpsMaskValue() {
  static const prop_info *info = nullptr;
  static uint32_t last_serial = 0;
  static uint32_t value = 1;
  if (!info) {
    info = __system_property_find("persist.ichthys.udfps_mask_level");
    if (!info) {
      return 1;
    }
    last_serial = __system_property_serial(info) + 1;  // force the first read
  }
  uint32_t serial = __system_property_serial(info);
  if (serial != last_serial) {
    last_serial = serial;
    __system_property_read_callback(info, [](void *cookie, const char *, const char *v,
                                             uint32_t) {
      *static_cast<uint32_t *>(cookie) = IchthysUdfpsMaskLevelValue(v);
    }, &value);
  }
  return value;
}
#endif  // UDFPS_ZPOS

void HWDeviceDRM::SetupFingerprintMask(const HWLayersInfo &hw_layer_info, bool validate) {
#ifdef UDFPS_ZPOS
  if (connector_info_.type != DRM_MODE_CONNECTOR_DSI) {
    return;
  }
  bool enabled = IchthysUdfpsSyncEnabled();
  if (!enabled && !fingerprint_mask_ && !udfps_lut_applied_ && !udfps_lut_resend_) {
    return;  // switched off and nothing to release: today's behaviour, property untouched
  }
  // SurfaceFlinger flags SystemUI's mask layer (window "UdfpsFodMask") with
  // FOD_PRESSED_LAYER_ZORDER; HWCLayer::SetLayerZOrder turns that into flags.fod_pressed. All
  // layers of the stack count, device- or GPU-composed.
  bool fod_present = false;
  if (enabled && hw_layer_info.stack) {
    for (const Layer *layer : hw_layer_info.stack->layers) {
      if (layer && layer->flags.fod_pressed) {
        fod_present = true;
        break;
      }
    }
  }
  // 1 = mask on at the kernel's default level (331, what the HAL writes). The kernel reacts to
  // changes only, in the kickoff of the commit that carries them, i.e. together with the frame
  // that first shows / first no longer shows the mask layer.
  // IchthysOS: or the mask level itself (persist.ichthys.udfps_mask_level, 256..486), read on the
  // mask's first frame and kept for the whole mask session.
  uint32_t value = 0;
  if (fod_present) {
    value = fingerprint_mask_ ? fingerprint_mask_ : IchthysUdfpsMaskValue();
  }
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_FINGERPRINT_MASK, token_.conn_id, value);
  if (!validate) {
    if (value != fingerprint_mask_) {
      DLOGI("IchthysOS udfps_sync: fingerprint_mask %u -> %u", fingerprint_mask_, value);
    }
    bool mask_on_edge = value && !fingerprint_mask_;  // IchthysOS: first frame of this mask
    fingerprint_mask_ = value;
    // IchthysOS: the mask tone curve goes into the same request as fingerprint_mask, so the DSPP
    // GC flips with the frame whose kickoff sends the mask HBM. Only the real commit carries it:
    // the GC does not take part in validation, and SDE colour-processing properties are latched
    // by the kernel even from a TEST_ONLY commit.
    SetupFingerprintLut(fod_present, mask_on_edge);
  }
#else
  (void)hw_layer_info;
  (void)validate;
#endif
}

#if defined(UDFPS_ZPOS) && defined(PP_DRM_ENABLE)
// IchthysOS: UDFPS mask tone curve.
// Under the global mask HBM the panel's response curve differs from normal mode (whites redder,
// shadows deeper), which a uniform dim plus a colour matrix cannot undo. A per-channel 1D curve is
// therefore put on the DSPP gamma correction block (GC v1.8, CRTC property "SDE_DSPP_GC_V1",
// struct drm_msm_pgc_lut) for exactly the frames that carry the mask layer.
//
// Source: three persist props per calibration point, one per channel, each 66 characters =
// 33 knots x 2 base64url characters (A-Z a-z 0-9 - _, big-endian: value = 64 * idx(c0) + idx(c1),
// 0..4095):
//   point 0: persist.ichthys.udfps_lut_r  / _g  / _b      (the original single curve)
//   point n: persist.ichthys.udfps_lut<n>_r / _g / _b     (n = 1..3)
// Knot k (0..32) is the 12-bit output for normalised input k/32. Expansion to the 1024-entry
// 10-bit GC table, integer-exact so the calibration script can reproduce it bit for bit:
//   num = i * 32; k = num / 1023; rem = num % 1023;            (i = input code 0..1023)
//   acc = (k == 32) ? y[32] * 1023 : y[k] * (1023 - rem) + y[k + 1] * rem;
//   out[i] = (2 * acc + 4095) / (2 * 4095);                      (round half up, 0..1023)
// Identity curve: "AACAEAGAIAKAMAOAQASAUAWAYAaAcAeAgAh_j_l_n_p_r_t_v_x_z_1_3_5_7_9___".
// The curve is applied after the colour mode's own GC (if the mode uses one):
//   gc[i] = curve[mode_gc[i]]   (curve = identity-domain function of the final 10-bit code)
// A point counts only when all three of its props are valid. Empty or unset = no curve.
//
// IchthysOS: brightness-dependent calibration (up to 4 points). The panel's normal-mode response
// differs per brightness, so one curve fits one brightness only.
//   persist.ichthys.udfps_lut_bl = comma list of screen brightness levels, field n = point n,
//   e.g. "8,128,230" (an empty field = that point has no level). Unit: Android's 0..255 brightness
//   setting (Settings.System.screen_brightness; float brightness f = (v - 1) / 254), the same unit
//   as SystemUI's Settings.Global ichthys_udfps_cal_alpha.
// Unset / no usable level: point 0 alone, exactly the old single-curve behaviour, no brightness
// read. Otherwise the points with a valid curve AND a level form the table (if none: point 0 alone
// when valid). With two or more points the panel's backlight level is read once at mask-on
// (GetUdfpsBrightness), converted to the 0..255 setting unit, and the knots are interpolated
// linearly between the two nearest points (clamped to the end points):
//   t = (bl - bl_a) / (bl_b - bl_a);  y[k] = round(y_a[k] + (y_b[k] - y_a[k]) * t)
// then expanded as above. Props are re-checked on every mask frame through the property area's
// serial (one atomic load when nothing changed).
//
// IchthysOS: parametric, automatically brightness-dependent curve. Used only when NO knot point
// above has all three props valid (knot props keep precedence; SystemUI applies the same rule).
// In this mode the dim window is only a separator (default alpha 1, keep = 254/255, i.e. no
// visible dim) so the content's codes stay <= 255 * keep and the illumination disc (code 255) can
// be told apart; the curve does ALL the darkening plus the colour / tone correction, at the GC's
// full 10-bit output.
// persist.ichthys.udfps_model = 12 numbers, separated by spaces or commas, per channel r, g, b in
// that order: a0 a1 p0 p1. For the panel's backlight level L (sysfs, 0..486, clamped to 1..486),
// read once at mask-on like above:
//   A_c = clamp(exp(a0_c + a1_c * ln L), 0.01, 4)     p_c = clamp(p0_c + p1_c * ln L, 0.6, 1.6)
// A_c is the TOTAL factor (normal-mode look of a content level, expressed in mask-HBM codes), not
// relative to any dim. With keep = 1 - alpha / 255 (the separator) and x = i / 1023:
//   knee = min(1022 / 1023, keep + 1 / 1023)   full = max(knee, 1020 / 1023)
//   x <= knee:         g(x) = min(1, A_c * (x / keep)^p_c)                 (content)
//   knee < x < full:   g(x) = g_top + (x - knee) * (1 - g_top) / (full - knee),  g_top = g(knee)
//   x >= full:         g(x) = 1                                            (the disc)
//   gc[i] = clamp(floor(g(x) * 1023 + 0.5), 0, 1023)     (double precision; keep >= 0.02)
// The top content code is 255 * keep (8-bit client composition: 254 at alpha 1) or 1023 * keep
// (device blending); expanded to 10 bits it is at most 1023 * keep + 0.8 (bit replication,
// 254 -> 1019), so it stays at or below the knee. The disc is 255, which the 8 -> 10-bit expansion
// makes 1023 (replication) or 1020 (shift), so everything from 1020 up is full light.
// udfps_lut_ref.py model_table() is the reference implementation. Identity tone with only the
// separator undone: "0 0 1 0" per channel.
// keep: ichthys.udfps_applied_alpha (0..255), published by SystemUI right before it adds the mask
// window, i.e. the separator that frame really carries (Settings.Global ichthys_udfps_sep_alpha
// overrides included). Unset / invalid (older SystemUI): persist.ichthys.udfps_sep_keep, the
// separator SystemUI uses by default (0.5..1, default 254 / 255):
//   alpha = max(1, round((1 - keep) * 255)).
// Both alpha props are polled every mask frame (a changed separator rebuilds the curve).
//
// IchthysOS: physics curve, the DEFAULT when no knot point is complete and no valid
// persist.ichthys.udfps_model is set (persist.ichthys.udfps_physics = 0 / false / off / n / no
// turns it off: no curve, as before). No calibration: per channel
//   A_c = w_c * (cd(L) / cd(M))^(1 / gamma),  p = 1,  same curve and separator as the model
// cd = the panel's candela per backlight level (kUdfpsCandela, from the kernel DT), L = the panel
// level at mask-on, M = max(mask level, L): the mask level is the one sent in fingerprint_mask
// (persist.ichthys.udfps_mask_level, else 331 = 547 nits) and the kernel drives max(mask, normal)
// while the mask is on, so L >= M gives A = w (only the separator undone).
// gamma: persist.ichthys.udfps_gamma (1..4, default 2.2); w: persist.ichthys.udfps_wb "r g b"
// (each 0.25..2, default "1 1 1"). Invalid values = the defaults.
// Precedence: complete knot point > persist.ichthys.udfps_model > physics; SystemUI uses the
// separator dim under the same rule (UdfpsController.isUdfpsHalCurveActive), sync mode only.
namespace {

constexpr int kUdfpsLutKnots = 33;
constexpr int kUdfpsLutStrLen = kUdfpsLutKnots * 2;
constexpr int kUdfpsLutEntries = 1024;
constexpr int kUdfpsLutPoints = 4;
// DRM GC component order is the SDE pixel order C0 = G, C1 = B, C2 = R (sde_hw_mdss.h C0_G_Y...);
// index here is that component, the name the user-facing channel.
const char *const kUdfpsLutProps[kUdfpsLutPoints][3] = {
  {"persist.ichthys.udfps_lut_g", "persist.ichthys.udfps_lut_b", "persist.ichthys.udfps_lut_r"},
  {"persist.ichthys.udfps_lut1_g", "persist.ichthys.udfps_lut1_b", "persist.ichthys.udfps_lut1_r"},
  {"persist.ichthys.udfps_lut2_g", "persist.ichthys.udfps_lut2_b", "persist.ichthys.udfps_lut2_r"},
  {"persist.ichthys.udfps_lut3_g", "persist.ichthys.udfps_lut3_b", "persist.ichthys.udfps_lut3_r"},
};
const char *const kUdfpsLutBlProp = "persist.ichthys.udfps_lut_bl";
const char *const kUdfpsModelProp = "persist.ichthys.udfps_model";
const char *const kUdfpsAlphaProp = "ichthys.udfps_applied_alpha";
const char *const kUdfpsSepKeepProp = "persist.ichthys.udfps_sep_keep";
const char *const kUdfpsPhysicsProp = "persist.ichthys.udfps_physics";
const char *const kUdfpsGammaProp = "persist.ichthys.udfps_gamma";
const char *const kUdfpsWbProp = "persist.ichthys.udfps_wb";
constexpr int kUdfpsModelParams = 12;
constexpr int kUdfpsLevels = 487;  // panel backlight levels 0..486
constexpr int kUdfpsDefaultMaskLevel = 331;  // the kernel's fingerprint_mask = 1 level

struct IchthysUdfpsProp {
  const prop_info *info = nullptr;
  uint32_t serial = 0;
};

struct IchthysUdfpsLut {
  bool polled = false;
  uint32_t area_serial = 0;
  IchthysUdfpsProp curve_prop[kUdfpsLutPoints][3];
  IchthysUdfpsProp bl_prop;
  // Calibration table, ascending brightness. One entry = the single-curve case (level unused).
  int num_points = 0;
  float level[kUdfpsLutPoints] = {};
  uint16_t knots[kUdfpsLutPoints][3][kUdfpsLutKnots] = {};
  float fallback_level = 0.0f;  // used when the brightness cannot be read: point 0's level
  // IchthysOS: parametric model (persist.ichthys.udfps_model), used when no knot point is valid.
  IchthysUdfpsProp model_prop;
  bool knots_present = false;  // some point has all three knot props valid
  bool model_valid = false;
  double model[kUdfpsModelParams] = {};
  // IchthysOS: physics curve (default on), used when no knot point and no model is set.
  IchthysUdfpsProp physics_prop;
  IchthysUdfpsProp gamma_prop;
  IchthysUdfpsProp wb_prop;
  bool physics = true;
  double gamma = 2.2;
  double wb[3] = {1.0, 1.0, 1.0};
  // The expanded curve last built and the brightness (model / physics: panel level, alpha and
  // mask level) it was built for.
  bool built = false;
  float built_level = -1.0f;
  int built_alpha = -1;
  int built_mask = -1;
  bool valid = false;
  uint16_t lut[3][kUdfpsLutEntries] = {};
};
IchthysUdfpsLut g_udfps_lut;

// IchthysOS: the separator alpha for the model: SystemUI's published one, else from the keep prop.
struct IchthysUdfpsAlpha {
  bool polled = false;
  uint32_t area_serial = 0;
  IchthysUdfpsProp applied_prop;
  IchthysUdfpsProp keep_prop;
  int applied = -1;
  int fallback = -1;
};
IchthysUdfpsAlpha g_udfps_alpha;

int UdfpsLutB64(char c) {
  if (c >= 'A' && c <= 'Z') return c - 'A';
  if (c >= 'a' && c <= 'z') return c - 'a' + 26;
  if (c >= '0' && c <= '9') return c - '0' + 52;
  if (c == '-') return 62;
  if (c == '_') return 63;
  return -1;
}

bool ParseUdfpsKnots(const std::string &value, uint16_t *y) {
  if (value.size() != kUdfpsLutStrLen) {
    return false;
  }
  for (int k = 0; k < kUdfpsLutKnots; k++) {
    int hi = UdfpsLutB64(value[2 * k]);
    int lo = UdfpsLutB64(value[2 * k + 1]);
    if (hi < 0 || lo < 0) {
      return false;
    }
    y[k] = static_cast<uint16_t>(hi * 64 + lo);
  }
  return true;
}

void ExpandUdfpsKnots(const uint16_t *y, uint16_t *out) {
  for (uint32_t i = 0; i < kUdfpsLutEntries; i++) {
    uint32_t num = i * (kUdfpsLutKnots - 1);
    uint32_t k = num / 1023;
    uint32_t rem = num % 1023;
    uint32_t acc = (k >= kUdfpsLutKnots - 1) ? UINT32(y[kUdfpsLutKnots - 1]) * 1023
                                               : UINT32(y[k]) * (1023 - rem) + UINT32(y[k + 1]) * rem;
    out[i] = static_cast<uint16_t>((2 * acc + 4095) / (2 * 4095));
  }
}

// Returns true when the prop appeared or got a new serial since the last call.
bool PollUdfpsProp(IchthysUdfpsProp *p, const char *name) {
  if (!p->info) {
    p->info = __system_property_find(name);
    if (!p->info) {
      return false;
    }
    p->serial = __system_property_serial(p->info);
    return true;
  }
  uint32_t serial = __system_property_serial(p->info);
  if (serial == p->serial) {
    return false;
  }
  p->serial = serial;
  return true;
}

std::string ReadUdfpsProp(const IchthysUdfpsProp &p) {
  std::string value;
  if (p.info) {
    __system_property_read_callback(p.info, [](void *cookie, const char *, const char *v,
                                               uint32_t) {
      *static_cast<std::string *>(cookie) = v;
    }, &value);
  }
  return value;
}

// "8,128,230" -> level[n] / has[n] per field; returns the number of usable levels.
int ParseUdfpsLevels(const std::string &value, float *level, bool *has) {
  int count = 0;
  size_t pos = 0;
  for (int n = 0; n < kUdfpsLutPoints; n++) {
    has[n] = false;
    if (pos > value.size()) {
      continue;
    }
    size_t end = value.find(',', pos);
    std::string field = value.substr(pos, end == std::string::npos ? std::string::npos
                                                                     : end - pos);
    pos = (end == std::string::npos) ? value.size() + 1 : end + 1;
    const char *s = field.c_str();
    while (*s == ' ') s++;
    if (!*s) {
      continue;
    }
    char *stop = nullptr;
    float v = strtof(s, &stop);
    while (stop && *stop == ' ') stop++;
    if (!stop || *stop || !(v >= 0.0f && v <= 1000.0f)) {
      continue;
    }
    level[n] = v;
    has[n] = true;
    count++;
  }
  return count;
}

// IchthysOS: parametric mask curve model (see the comment at the top of this section). Pure
// functions; the calibration dir's udfps_model_test.py compiles the block between the markers on
// the host and compares it with udfps_lut_ref.py.
// UDFPS_MODEL_MATH_BEGIN
// DRM component c (0 = G, 1 = B, 2 = R) -> channel index in the prop (r, g, b).
constexpr int kUdfpsModelChannel[3] = {1, 2, 0};

bool UdfpsModelSep(char c) {
  return c == ' ' || c == ',' || c == '\t';
}

// "a0r a1r p0r p1r a0g ... p1b" -> params[12]; false unless exactly 12 finite numbers.
bool ParseUdfpsModel(const std::string &value, double *params) {
  const char *s = value.c_str();
  int n = 0;
  for (;;) {
    while (UdfpsModelSep(*s)) s++;
    if (!*s) {
      break;
    }
    if (n == kUdfpsModelParams) {
      return false;
    }
    char *stop = nullptr;
    double v = strtod(s, &stop);
    if (stop == s || (*stop && !UdfpsModelSep(*stop)) || !std::isfinite(v) || fabs(v) > 100.0) {
      return false;
    }
    params[n++] = v;
    s = stop;
  }
  return n == kUdfpsModelParams;
}

// "0".."255" -> alpha, else -1.
int ParseUdfpsAlpha(const std::string &value) {
  const char *s = value.c_str();
  while (*s == ' ') s++;
  if (*s < '0' || *s > '9') {
    return -1;
  }
  char *stop = nullptr;
  long v = strtol(s, &stop, 10);
  while (stop && *stop == ' ') stop++;
  if (!stop || *stop || v < 0 || v > 255) {
    return -1;
  }
  return static_cast<int>(v);
}

// persist.ichthys.udfps_sep_keep -> separator alpha, as SystemUI computes it (udfpsSeparatorAlpha):
// keep in 0.5..1 (unset / invalid = 254 / 255), alpha = max(1, round((1 - keep) * 255)).
int UdfpsSepAlpha(const std::string &value) {
  double keep = 254.0 / 255.0;
  const char *s = value.c_str();
  char *stop = nullptr;
  double v = strtod(s, &stop);
  if (stop != s && !*stop && std::isfinite(v)) {
    keep = std::max(0.5, std::min(1.0, v));
  }
  return std::max(1, static_cast<int>(lround((1.0 - keep) * 255.0)));
}

// Total gain A and exponent p of channel ch (0 = r, 1 = g, 2 = b) at the panel level.
void UdfpsModelShape(const double *params, int ch, double level, double *gain, double *expo) {
  const double *q = params + 4 * ch;
  double ln_l = log(std::max(1.0, std::min(486.0, level)));
  *gain = std::max(0.01, std::min(4.0, exp(q[0] + q[1] * ln_l)));
  *expo = std::max(0.6, std::min(1.6, q[2] + q[3] * ln_l));
}

// One channel's 1024-entry GC curve: total gain and exponent below the knee, ramp to the disc.
void BuildUdfpsCurve(double gain, double expo, int alpha, uint16_t *out) {
  double keep = std::max(0.02, 1.0 - std::max(0, std::min(255, alpha)) / 255.0);
  double knee = std::min(1022.0 / 1023.0, keep + 1.0 / 1023.0);
  double full = std::max(knee, 1020.0 / 1023.0);
  double y_top = std::min(1.0, gain * pow(knee / keep, expo));
  for (int i = 0; i < kUdfpsLutEntries; i++) {
    double x = i / 1023.0;
    double y;
    if (x <= knee) {
      y = std::min(1.0, gain * pow(x / keep, expo));
    } else if (x >= full) {
      y = 1.0;
    } else {
      y = y_top + (x - knee) * (1.0 - y_top) / (full - knee);
    }
    y = std::max(0.0, std::min(1.0, y));
    out[i] = static_cast<uint16_t>(std::min(1023.0, floor(y * 1023.0 + 0.5)));
  }
}

// The 1024-entry GC curve per DRM component for the panel level and separator alpha.
void BuildUdfpsModelTable(const double *params, double level, int alpha,
                          uint16_t (*out)[kUdfpsLutEntries]) {
  for (int c = 0; c < 3; c++) {
    double gain = 1.0;
    double expo = 1.0;
    UdfpsModelShape(params, kUdfpsModelChannel[c], level, &gain, &expo);
    BuildUdfpsCurve(gain, expo, alpha, out[c]);
  }
}

// IchthysOS: the panel's luminance (cd/m2) per backlight level 0..486. Generated by
// /mnt/B1/OneUi9/scratch/udfps_cal/gen_candela_table.py from candela.json, i.e. the a52sxq panel
// DT (kernel/samsung/sm7325/arch/arm64/boot/dts/samsung/a52/a52sxq/a52sxq_kor_single_w00_r02.dts):
// samsung,candela_map_table_revA for 0..255 and samsung,hbm_candela_map_table_revI for 256..486
// (the HBM revA table has the same candela column; only its DBV column differs).
const uint16_t kUdfpsCandela[kUdfpsLevels] = {
  2, 2, 3, 3, 4, 5, 5, 6, 7, 7, 8, 9, 10, 11, 12, 13,
  13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 25, 26, 27, 28, 29,
  30, 31, 32, 34, 35, 36, 37, 38, 40, 41, 42, 43, 45, 46, 47, 48,
  50, 51, 52, 54, 55, 56, 58, 59, 60, 62, 63, 64, 66, 67, 68, 70,
  71, 73, 74, 76, 77, 78, 80, 81, 83, 84, 86, 87, 89, 90, 92, 93,
  95, 96, 98, 99, 101, 102, 104, 105, 107, 108, 110, 112, 113, 115, 116, 118,
  119, 121, 123, 124, 126, 127, 129, 131, 132, 134, 136, 137, 139, 140, 142, 144,
  145, 147, 149, 150, 152, 154, 156, 157, 159, 161, 162, 164, 166, 167, 169, 171,
  173, 174, 176, 178, 180, 181, 183, 185, 187, 188, 190, 192, 194, 195, 197, 199,
  201, 203, 204, 206, 208, 210, 212, 214, 215, 217, 219, 221, 223, 225, 226, 228,
  230, 232, 234, 236, 237, 239, 241, 243, 245, 247, 249, 251, 253, 254, 256, 258,
  260, 262, 264, 266, 268, 270, 272, 274, 275, 277, 279, 281, 283, 285, 287, 289,
  291, 293, 295, 297, 299, 301, 303, 305, 307, 309, 311, 313, 315, 317, 319, 321,
  323, 325, 327, 329, 331, 333, 335, 337, 339, 341, 343, 345, 347, 349, 351, 353,
  355, 357, 359, 361, 363, 365, 368, 370, 372, 374, 376, 378, 380, 382, 384, 386,
  388, 390, 393, 395, 397, 399, 401, 403, 405, 407, 409, 411, 414, 416, 418, 420,
  425, 426, 428, 430, 432, 433, 435, 436, 438, 440, 441, 443, 444, 446, 448, 450,
  451, 453, 454, 456, 458, 459, 461, 462, 464, 466, 468, 469, 471, 472, 474, 476,
  477, 479, 480, 482, 484, 485, 487, 488, 490, 492, 494, 495, 497, 498, 500, 502,
  503, 505, 506, 508, 510, 512, 513, 515, 516, 518, 520, 521, 523, 524, 526, 528,
  529, 531, 533, 534, 536, 538, 539, 541, 542, 544, 546, 547, 549, 550, 552, 554,
  555, 557, 559, 560, 562, 564, 565, 567, 568, 570, 572, 573, 575, 577, 578, 580,
  582, 583, 585, 586, 588, 590, 591, 593, 595, 596, 598, 599, 601, 603, 604, 606,
  608, 609, 611, 612, 614, 616, 617, 619, 621, 622, 624, 626, 627, 629, 630, 632,
  634, 635, 637, 639, 640, 642, 643, 645, 647, 648, 650, 652, 653, 655, 657, 658,
  660, 661, 663, 665, 666, 668, 670, 671, 673, 675, 676, 678, 679, 681, 683, 684,
  686, 687, 689, 691, 692, 694, 696, 697, 699, 701, 702, 704, 705, 707, 709, 710,
  712, 713, 715, 717, 719, 720, 722, 723, 725, 727, 728, 730, 731, 733, 735, 737,
  738, 740, 741, 743, 745, 746, 748, 749, 751, 753, 754, 756, 757, 759, 761, 763,
  764, 766, 767, 769, 771, 772, 774, 775, 777, 779, 781, 782, 784, 785, 787, 789,
  790, 792, 793, 795, 797, 799, 800,
};

// persist.ichthys.udfps_physics: off only for Android's false spellings (android::base::ParseBool).
bool UdfpsPhysicsEnabled(const std::string &value) {
  return !(value == "0" || value == "n" || value == "no" || value == "off" || value == "false");
}

// persist.ichthys.udfps_gamma: 1..4, else 2.2.
double ParseUdfpsGamma(const std::string &value) {
  const char *s = value.c_str();
  char *stop = nullptr;
  double v = strtod(s, &stop);
  if (stop == s || *stop || !std::isfinite(v) || v < 1.0 || v > 4.0) {
    return 2.2;
  }
  return v;
}

// persist.ichthys.udfps_wb "r g b": three numbers 0.25..2 (separated like the model), else 1 1 1.
void ParseUdfpsWb(const std::string &value, double *wb) {
  double v[3] = {1.0, 1.0, 1.0};
  const char *s = value.c_str();
  int n = 0;
  bool ok = true;
  for (;;) {
    while (UdfpsModelSep(*s)) s++;
    if (!*s) {
      break;
    }
    char *stop = nullptr;
    double x = strtod(s, &stop);
    if (n == 3 || stop == s || (*stop && !UdfpsModelSep(*stop)) || !std::isfinite(x) ||
        x < 0.25 || x > 2.0) {
      ok = false;
      break;
    }
    v[n++] = x;
    s = stop;
  }
  for (int c = 0; c < 3; c++) {
    wb[c] = (ok && n == 3) ? v[c] : 1.0;
  }
}

// (cd(L) / cd(M))^(1 / gamma), M = max(mask level, L).
double UdfpsPhysicsBase(double level, int mask_level, double gamma) {
  int l = std::max(0, std::min(kUdfpsLevels - 1, static_cast<int>(level)));
  int m = std::max(l, std::max(0, std::min(kUdfpsLevels - 1, mask_level)));
  return pow(static_cast<double>(kUdfpsCandela[l]) / kUdfpsCandela[m], 1.0 / gamma);
}

// The physics GC curve per DRM component (wb in r, g, b order).
void BuildUdfpsPhysicsTable(double level, int mask_level, double gamma, const double *wb,
                            int alpha, uint16_t (*out)[kUdfpsLutEntries]) {
  double base = UdfpsPhysicsBase(level, mask_level, gamma);
  for (int c = 0; c < 3; c++) {
    BuildUdfpsCurve(wb[kUdfpsModelChannel[c]] * base, 1.0, alpha, out[c]);
  }
}
// UDFPS_MODEL_MATH_END

// Re-reads the calibration table when any of its props changed. Returns true when it did (the
// curve must then be rebuilt).
bool PollUdfpsLut() {
  IchthysUdfpsLut &t = g_udfps_lut;
  uint32_t area_serial = __system_property_area_serial();
  if (t.polled && area_serial == t.area_serial) {
    return false;  // no property anywhere was added or changed since the last poll
  }
  bool changed = false;
  t.polled = true;
  t.area_serial = area_serial;
  for (int p = 0; p < kUdfpsLutPoints; p++) {
    for (int c = 0; c < 3; c++) {
      changed |= PollUdfpsProp(&t.curve_prop[p][c], kUdfpsLutProps[p][c]);
    }
  }
  changed |= PollUdfpsProp(&t.bl_prop, kUdfpsLutBlProp);
  changed |= PollUdfpsProp(&t.model_prop, kUdfpsModelProp);
  changed |= PollUdfpsProp(&t.physics_prop, kUdfpsPhysicsProp);
  changed |= PollUdfpsProp(&t.gamma_prop, kUdfpsGammaProp);
  changed |= PollUdfpsProp(&t.wb_prop, kUdfpsWbProp);
  if (!changed) {
    return false;  // (no prop at all: the struct defaults are the physics defaults)
  }

  bool curve_ok[kUdfpsLutPoints] = {};
  uint16_t knots[kUdfpsLutPoints][3][kUdfpsLutKnots] = {};
  for (int p = 0; p < kUdfpsLutPoints; p++) {
    curve_ok[p] = true;
    for (int c = 0; c < 3 && curve_ok[p]; c++) {
      curve_ok[p] = ParseUdfpsKnots(ReadUdfpsProp(t.curve_prop[p][c]), knots[p][c]);
    }
  }
  float level[kUdfpsLutPoints] = {};
  bool has_level[kUdfpsLutPoints] = {};
  int num_levels = ParseUdfpsLevels(ReadUdfpsProp(t.bl_prop), level, has_level);

  int order[kUdfpsLutPoints];
  int n = 0;
  if (num_levels > 0) {
    for (int p = 0; p < kUdfpsLutPoints; p++) {
      if (curve_ok[p] && has_level[p]) {
        order[n++] = p;
      }
    }
    std::stable_sort(order, order + n, [&level](int a, int b) { return level[a] < level[b]; });
  }
  if (n == 0 && curve_ok[0]) {
    order[n++] = 0;  // no brightness list (or none usable): point 0 alone, the old behaviour
    level[0] = 0.0f;
  }
  int prev_points = t.num_points;
  bool prev_auto = (t.model_valid || t.physics) && !t.knots_present;
  t.knots_present = false;
  for (int p = 0; p < kUdfpsLutPoints; p++) {
    t.knots_present |= curve_ok[p];
  }
  t.model_valid = ParseUdfpsModel(ReadUdfpsProp(t.model_prop), t.model);
  t.physics = UdfpsPhysicsEnabled(ReadUdfpsProp(t.physics_prop));
  t.gamma = ParseUdfpsGamma(ReadUdfpsProp(t.gamma_prop));
  ParseUdfpsWb(ReadUdfpsProp(t.wb_prop), t.wb);
  t.num_points = n;
  for (int i = 0; i < n; i++) {
    t.level[i] = level[order[i]];
    memcpy(t.knots[i], knots[order[i]], sizeof(t.knots[i]));
  }
  t.fallback_level = (n > 0 && curve_ok[0] && has_level[0]) ? level[0] : (n > 0 ? t.level[0] : 0.0f);
  t.built = false;
  if (n == 0) {
    if (t.model_valid && !t.knots_present) {
      const double *m = t.model;
      DLOGI("IchthysOS udfps_lut: parametric model loaded, r %.3f %.3f %.3f %.3f g %.3f %.3f %.3f "
            "%.3f b %.3f %.3f %.3f %.3f", m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8],
            m[9], m[10], m[11]);
    } else if (t.physics && !t.knots_present) {
      DLOGI("IchthysOS udfps_lut: physics curve, gamma %.3f, wb r/g/b %.3f/%.3f/%.3f", t.gamma,
            t.wb[0], t.wb[1], t.wb[2]);
    } else if (prev_points > 0 || prev_auto) {
      DLOGI("IchthysOS udfps_lut: curve invalid or unset, off");
    }
  } else if (n == 1) {
    DLOGI("IchthysOS udfps_lut: curve loaded (point %d alone)", order[0]);
  } else {
    DLOGI("IchthysOS udfps_lut: %d calibration points loaded, brightness %.1f .. %.1f", n,
          t.level[0], t.level[n - 1]);
  }
  return true;
}

// Builds g_udfps_lut.lut for the screen brightness bl (0..255 setting unit; ignored with fewer
// than two points). Returns true when the curve content may have changed.
bool BuildUdfpsLut(float bl) {
  IchthysUdfpsLut &t = g_udfps_lut;
  if (t.num_points == 0) {
    bool was_valid = t.valid;
    t.valid = false;
    t.built = true;
    return was_valid;
  }
  if (t.num_points == 1) {
    if (t.built) {
      return false;
    }
    for (int c = 0; c < 3; c++) {
      ExpandUdfpsKnots(t.knots[0][c], t.lut[c]);
    }
    t.valid = t.built = true;
    t.built_level = -1.0f;
    return true;
  }
  if (t.built && bl == t.built_level) {
    return false;
  }
  int a = 0;
  int b = 0;
  float f = 0.0f;
  if (bl <= t.level[0]) {
    a = b = 0;
  } else if (bl >= t.level[t.num_points - 1]) {
    a = b = t.num_points - 1;
  } else {
    for (int i = 0; i + 1 < t.num_points; i++) {
      if (bl >= t.level[i] && bl < t.level[i + 1]) {
        a = i;
        b = i + 1;
        f = (bl - t.level[i]) / (t.level[i + 1] - t.level[i]);
        break;
      }
    }
  }
  for (int c = 0; c < 3; c++) {
    uint16_t y[kUdfpsLutKnots];
    for (int k = 0; k < kUdfpsLutKnots; k++) {
      float ya = t.knots[a][c][k];
      float yb = t.knots[b][c][k];
      long v = lroundf(ya + (yb - ya) * f);
      y[k] = static_cast<uint16_t>(std::max(0L, std::min(4095L, v)));
    }
    ExpandUdfpsKnots(y, t.lut[c]);
  }
  DLOGI("IchthysOS udfps_lut: curve for brightness %.1f (%.1f..%.1f, t %.3f)", bl, t.level[a],
        t.level[b], f);
  t.valid = t.built = true;
  t.built_level = bl;
  return true;
}

// IchthysOS: the separator alpha for the model curve; *published = it came from SystemUI. One
// atomic load per call while no property changed.
int PollUdfpsSepAlpha(bool *published) {
  IchthysUdfpsAlpha &a = g_udfps_alpha;
  uint32_t area_serial = __system_property_area_serial();
  if (!a.polled || area_serial != a.area_serial) {
    bool first = !a.polled;
    a.polled = true;
    a.area_serial = area_serial;
    if (PollUdfpsProp(&a.applied_prop, kUdfpsAlphaProp)) {
      a.applied = ParseUdfpsAlpha(ReadUdfpsProp(a.applied_prop));
    }
    if (PollUdfpsProp(&a.keep_prop, kUdfpsSepKeepProp) || first) {
      a.fallback = UdfpsSepAlpha(ReadUdfpsProp(a.keep_prop));
    }
  }
  *published = a.applied >= 0;
  return *published ? a.applied : a.fallback;
}

// IchthysOS: builds g_udfps_lut.lut from the parametric model, else the physics curve, for the
// panel level (sysfs units, < 0 = unknown: no curve), separator alpha and mask level. Returns true
// when the curve content may have changed.
bool BuildUdfpsAutoLut(float level, int alpha, bool published, int mask_level) {
  IchthysUdfpsLut &t = g_udfps_lut;
  if (level < 0.0f) {
    bool was_valid = t.valid;
    t.valid = false;
    return was_valid;
  }
  if (t.built && t.valid && level == t.built_level && alpha == t.built_alpha &&
      mask_level == t.built_mask) {
    return false;
  }
  t.valid = t.built = true;
  t.built_level = level;
  t.built_alpha = alpha;
  t.built_mask = mask_level;
  if (!t.model_valid) {
    BuildUdfpsPhysicsTable(level, mask_level, t.gamma, t.wb, alpha, t.lut);
    int l = std::max(0, std::min(kUdfpsLevels - 1, static_cast<int>(level)));
    int m = std::max(l, std::max(0, std::min(kUdfpsLevels - 1, mask_level)));
    double base = UdfpsPhysicsBase(level, mask_level, t.gamma);
    int cd_l = kUdfpsCandela[l];
    int cd_m = kUdfpsCandela[m];
    DLOGI("IchthysOS udfps_lut: physics curve for panel level %d (%d cd) under mask level %d "
          "(%d cd), separator alpha %d (%s): A r/g/b %.3f/%.3f/%.3f", l, cd_l, m, cd_m, alpha,
          published ? "SystemUI" : "sep_keep", t.wb[0] * base, t.wb[1] * base, t.wb[2] * base);
    return true;
  }
  BuildUdfpsModelTable(t.model, level, alpha, t.lut);
  double gain[3] = {};
  double expo[3] = {};
  for (int ch = 0; ch < 3; ch++) {
    UdfpsModelShape(t.model, ch, level, &gain[ch], &expo[ch]);
  }
  DLOGI("IchthysOS udfps_lut: model curve for panel level %.0f, separator alpha %d (%s): A r/g/b "
        "%.3f/%.3f/%.3f, p %.3f/%.3f/%.3f", level, alpha, published ? "SystemUI" : "sep_keep",
        gain[0], gain[1], gain[2], expo[0], expo[1], expo[2]);
  return true;
}

}  // namespace
#endif  // UDFPS_ZPOS && PP_DRM_ENABLE

// IchthysOS: remembers the GC the colour manager sets (SetPPFeatures), so mask-off can restore it
// exactly and mask-on can compose the curve on top of it.
void HWDeviceDRM::CacheFingerprintLutBase(const DRMPPFeatureInfo &pgc) {
#if defined(UDFPS_ZPOS) && defined(PP_DRM_ENABLE)
  const drm_msm_pgc_lut *lut = static_cast<const drm_msm_pgc_lut *>(pgc.payload);
  if (!lut || pgc.payload_size != sizeof(drm_msm_pgc_lut)) {
    udfps_pgc_base_.clear();
    udfps_pgc_base_flags_ = 0;
    return;
  }
  const uint32_t *src[3] = {lut->c0, lut->c1, lut->c2};
  udfps_pgc_base_.resize(3 * kUdfpsLutEntries);
  for (int c = 0; c < 3; c++) {
    uint16_t *dst = &udfps_pgc_base_[c * kUdfpsLutEntries];
    for (int i = 0; i < PGC_TBL_LEN; i++) {  // two 10-bit entries per word: even low, odd << 16
      dst[2 * i] = static_cast<uint16_t>(src[c][i] & 0x3FF);
      dst[2 * i + 1] = static_cast<uint16_t>((src[c][i] >> 16) & 0x3FF);
    }
  }
  udfps_pgc_base_flags_ = lut->flags;
#else
  (void)pgc;
#endif
}

// IchthysOS: builds the GC payload to commit: the colour mode's GC alone (mask_curve false; nullptr
// = GC off, which is the colour mode's state when it set none) or the mask curve composed on it.
// Caller owns the result (delete as drm_msm_pgc_lut).
void *HWDeviceDRM::BuildFingerprintLutPgc(bool mask_curve) {
#if defined(UDFPS_ZPOS) && defined(PP_DRM_ENABLE)
  bool have_base = udfps_pgc_base_.size() == 3 * kUdfpsLutEntries;
  if (!mask_curve && !have_base) {
    return nullptr;
  }
  drm_msm_pgc_lut *out = new drm_msm_pgc_lut();
  out->flags = have_base ? udfps_pgc_base_flags_ : 0;
  if (mask_curve) {
    // IchthysOS: the mask curve does the dark end in 10 bits, so the GC must not round its output
    // to 8 bits (GC_8B_ROUND_EN); the PP dither after the DSPP takes it to the panel's 8 bpc. The
    // colour manager never sets the flag today (GetDrmPGC: flags 0), this only makes it certain.
    out->flags &= ~static_cast<uint64_t>(PGC_8B_ROUND);
  }
  uint32_t *dst[3] = {out->c0, out->c1, out->c2};
  for (int c = 0; c < 3; c++) {
    uint16_t entry[2];
    for (int i = 0; i < PGC_TBL_LEN; i++) {
      for (int j = 0; j < 2; j++) {
        uint32_t idx = UINT32(2 * i + j);
        uint32_t v = have_base ? udfps_pgc_base_[c * kUdfpsLutEntries + idx] : idx;
        if (mask_curve) {
          v = g_udfps_lut.lut[c][v & 0x3FF];
        }
        entry[j] = static_cast<uint16_t>(v & 0x3FF);
      }
      dst[c][i] = UINT32(entry[0]) | (UINT32(entry[1]) << 16);
    }
  }
  return out;
#else
  (void)mask_curve;
  return nullptr;
#endif
}

// IchthysOS: the screen brightness for picking the UDFPS mask curve, in the unit of the
// calibration table (Android's 0..255 setting). Source: the panel's backlight node
// /sys/class/backlight/panel<N>-backlight/brightness (HWPeripheralDRM::GetPanelBrightness; the
// same node SetPanelBrightness writes, so it holds the level actually sent to the panel, after
// auto-brightness / dimming). Its scale is 0..max_brightness (486 on this panel: 0..255 normal
// range, 256..486 the outdoor HBM range); the composer writes level = int(f * (max - min) + min)
// for the framework's brightness float f (min = 1). That is undone here (+0.5 = the middle of the
// truncated step) and mapped to the setting unit v = 1 + 254 * f. Returns -1 when unreadable.
float HWDeviceDRM::GetUdfpsBrightness() {
  int level = 0;
  if (GetPanelBrightness(&level) != kErrorNone) {
    return -1.0f;
  }
  if (level <= 0) {
    return 0.0f;
  }
  float max = hw_panel_info_.panel_max_brightness;
  float min = hw_panel_info_.panel_min_brightness;
  if (max <= min) {
    return FLOAT(level);
  }
  float f = (FLOAT(level) + 0.5f - min) / (max - min);
  return 1.0f + 254.0f * std::max(0.0f, std::min(1.0f, f));
}

// IchthysOS: the panel's raw backlight level (same node as GetUdfpsBrightness, 0..max_brightness
// = 486 here) for the parametric mask curve model. Returns -1 when unreadable.
int HWDeviceDRM::GetUdfpsPanelLevel() {
  int level = 0;
  if (GetPanelBrightness(&level) != kErrorNone) {
    return -1;
  }
  return std::max(0, level);
}

// IchthysOS: puts the wanted GC (mask curve while the mask layer is shown and a valid curve is
// set, else the colour mode's own GC) into the request on changes only. Commit path only.
// mask_on_edge = first frame of a mask: with several calibration points the brightness is read
// then (and when the table changes), not per frame.
void HWDeviceDRM::SetupFingerprintLut(bool fod_present, bool mask_on_edge) {
#if defined(UDFPS_ZPOS) && defined(PP_DRM_ENABLE)
  bool curve_changed = false;
  bool want = false;
  if (fod_present) {
    bool table_changed = PollUdfpsLut();
    if (!g_udfps_lut.knots_present && (g_udfps_lut.model_valid || g_udfps_lut.physics)) {
      // IchthysOS: parametric model or physics curve (no knot point set). The panel level is read
      // at mask-on (and when the props change); unreadable keeps the last level, or no curve if
      // there is none. The mask level is the one this session's fingerprint_mask carries.
      float level = g_udfps_lut.built ? g_udfps_lut.built_level : -1.0f;
      if (mask_on_edge || table_changed || !g_udfps_lut.built) {
        int raw = GetUdfpsPanelLevel();
        if (raw >= 0) {
          level = FLOAT(raw);
        } else if (mask_on_edge || table_changed) {
          DLOGW("IchthysOS udfps_lut: panel level unreadable, %s",
                level >= 0.0f ? "keeping the last one" : "no model curve");
        }
      }
      bool published = false;
      int alpha = PollUdfpsSepAlpha(&published);
      int mask_level = fingerprint_mask_ > 1 ? static_cast<int>(fingerprint_mask_)
                                             : kUdfpsDefaultMaskLevel;
      curve_changed = BuildUdfpsAutoLut(level, alpha, published, mask_level);
    } else {
      float bl = g_udfps_lut.built_level;
      if (g_udfps_lut.num_points >= 2 &&
          (mask_on_edge || table_changed || !g_udfps_lut.built)) {
        bl = GetUdfpsBrightness();
        if (bl < 0.0f) {
          bl = g_udfps_lut.built ? g_udfps_lut.built_level : g_udfps_lut.fallback_level;
          DLOGW("IchthysOS udfps_lut: brightness unreadable, using %.1f", bl);
        }
      }
      curve_changed = BuildUdfpsLut(bl);
    }
    want = g_udfps_lut.valid;
  }
  if (want == udfps_lut_applied_ && !udfps_lut_resend_ && !(want && curve_changed)) {
    return;
  }
  DRMPPFeatureInfo info = {};
  info.id = sde_drm::kFeaturePgc;
  drm_mgr_intf_->GetCrtcPPInfo(token_.crtc_id, &info);
  if (info.version == std::numeric_limits<uint32_t>::max() || !hw_color_mgr_) {
    udfps_lut_applied_ = false;  // no DSPP GC on this CRTC: nothing to do, ever
    udfps_lut_resend_ = false;
    return;
  }
  drm_msm_pgc_lut *payload = static_cast<drm_msm_pgc_lut *>(BuildFingerprintLutPgc(want));
  info.id = sde_drm::kFeaturePgc;
  info.type = sde_drm::kPropBlob;
  info.payload = payload;
  info.payload_size = sizeof(drm_msm_pgc_lut);
  // Same path as the colour manager: DRMPPManager owns the blob and frees the previous one.
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_POST_PROC, token_.crtc_id, &info);
  delete payload;
  if (want != udfps_lut_applied_) {
    DLOGI("IchthysOS udfps_lut: GC %s", want ? "mask curve" : "restored");
  }
  udfps_lut_applied_ = want;
  udfps_lut_resend_ = false;
  udfps_lut_in_req_ = true;
#else
  (void)fod_present;
  (void)mask_on_edge;
#endif
}

DisplayError HWDeviceDRM::Validate(HWLayers *hw_layers) {
  DTRACE_SCOPED();

  DisplayError err = kErrorNone;
  registry_.Register(hw_layers);

  Fence::ScopedRef scoped_ref;
  SetupAtomic(scoped_ref, hw_layers, true /* validate */, nullptr, nullptr);

  int ret = drm_atomic_intf_->Validate();
  if (ret) {
    DLOGE("failed with error %d for %s", ret, device_name_);
    DumpHWLayers(hw_layers);
    vrefresh_ = 0;
    panel_mode_changed_ = 0;
    seamless_mode_switch_ = false;
    err = kErrorHardware;
  }

  return err;
}

DisplayError HWDeviceDRM::Commit(HWLayers *hw_layers) {
  DTRACE_SCOPED();

  DisplayError err = kErrorNone;
  registry_.Register(hw_layers);

  if (default_mode_) {
    err = DefaultCommit(hw_layers);
  } else {
    err = AtomicCommit(hw_layers);
  }

  return err;
}

DisplayError HWDeviceDRM::DefaultCommit(HWLayers *hw_layers) {
  DTRACE_SCOPED();

  HWLayersInfo &hw_layer_info = hw_layers->info;

  for (Layer &layer : hw_layer_info.hw_layers) {
    layer.input_buffer.release_fence = nullptr;
  }

  DRMMaster *master = nullptr;
  int ret = DRMMaster::GetInstance(&master);
  if (ret < 0) {
    DLOGE("Failed to acquire DRMMaster instance");
    return kErrorResources;
  }

  DRMResMgr *res_mgr = nullptr;
  ret = DRMResMgr::GetInstance(&res_mgr);
  if (ret < 0) {
    DLOGE("Failed to acquire DRMResMgr instance");
    return kErrorResources;
  }

  int dev_fd = -1;
  master->GetHandle(&dev_fd);

  uint32_t connector_id = 0;
  res_mgr->GetConnectorId(&connector_id);

  uint32_t crtc_id = 0;
  res_mgr->GetCrtcId(&crtc_id);

  drmModeModeInfo mode;
  res_mgr->GetMode(&mode);

  uint64_t handle_id = hw_layer_info.hw_layers.at(0).input_buffer.handle_id;
  uint32_t fb_id = registry_.GetFbId(&hw_layer_info.hw_layers.at(0), handle_id);
  ret = drmModeSetCrtc(dev_fd, crtc_id, fb_id, 0 /* x */, 0 /* y */, &connector_id,
                       1 /* num_connectors */, &mode);
  if (ret < 0) {
    DLOGE("drmModeSetCrtc failed dev fd %d, fb_id %d, crtc id %d, connector id %d, %s", dev_fd,
          fb_id, crtc_id, connector_id, strerror(errno));
    return kErrorHardware;
  }

  return kErrorNone;
}

DisplayError HWDeviceDRM::AtomicCommit(HWLayers *hw_layers) {
  DTRACE_SCOPED();

  int64_t release_fence_fd = -1;
  int64_t retire_fence_fd = -1;

  // scoped fence fds will be automatically closed when function scope ends,
  // atomic commit will have these fds already set on kernel by then.
  Fence::ScopedRef scoped_ref;
  SetupAtomic(scoped_ref, hw_layers, false /* validate */, &release_fence_fd, &retire_fence_fd);

  bool sync_commit = synchronous_commit_ ||
                    (tui_state_ == kTUIStateStart || tui_state_ == kTUIStateEnd);

  if (hw_layers->elapse_timestamp > 0) {
    struct timespec t = {0, 0};
    clock_gettime(CLOCK_MONOTONIC, &t);
    uint64_t current_time = (UINT64(t.tv_sec) * 1000000000LL + t.tv_nsec);
    if (current_time < hw_layers->elapse_timestamp) {
      usleep(UINT32((hw_layers->elapse_timestamp - current_time) / 1000));
    }
  }

  int ret = drm_atomic_intf_->Commit(sync_commit, false /* retain_planes*/);
  shared_ptr<Fence> release_fence = Fence::Create(INT(release_fence_fd), "release");
  shared_ptr<Fence> retire_fence = Fence::Create(INT(retire_fence_fd), "retire");
  if (ret && udfps_lut_in_req_) {
    udfps_lut_resend_ = true;  // IchthysOS: the UDFPS mask GC may not have landed; send it again
  }
  udfps_lut_in_req_ = false;
  if (ret) {
    DLOGE("%s failed with error %d crtc %d", __FUNCTION__, ret, token_.crtc_id);
    DumpHWLayers(hw_layers);
    vrefresh_ = 0;
    panel_mode_changed_ = 0;
    seamless_mode_switch_ = false;
    return kErrorHardware;
  }

  DLOGD_IF(kTagDriverConfig, "RELEASE fence: fd: %s", Fence::GetStr(release_fence).c_str());
  DLOGD_IF(kTagDriverConfig, "RETIRE fence: fd: %s", Fence::GetStr(retire_fence).c_str());

  HWLayersInfo &hw_layer_info = hw_layers->info;
  LayerStack *stack = hw_layer_info.stack;
  stack->retire_fence = retire_fence;

  for (uint32_t i = 0; i < hw_layer_info.hw_layers.size(); i++) {
    Layer &layer = hw_layer_info.hw_layers.at(i);
    HWRotatorSession *hw_rotator_session = &hw_layers->config[i].hw_rotator_session;
    if (hw_rotator_session->mode == kRotatorOffline) {
      hw_rotator_session->output_buffer.release_fence = release_fence;
    } else {
      layer.input_buffer.release_fence = release_fence;
    }
  }

  hw_layer_info.sync_handle = release_fence;

  if (vrefresh_) {
    // Update current mode index if refresh rate is changed
    drmModeModeInfo current_mode = connector_info_.modes[current_mode_index_].mode;
    uint64_t current_bit_clk = connector_info_.modes[current_mode_index_].bit_clk_rate;
    for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
      if ((current_mode.vdisplay == connector_info_.modes[mode_index].mode.vdisplay) &&
          (current_mode.hdisplay == connector_info_.modes[mode_index].mode.hdisplay) &&
          (current_bit_clk == connector_info_.modes[mode_index].bit_clk_rate) &&
          (vrefresh_ == connector_info_.modes[mode_index].mode.vrefresh)) {
        current_mode_index_ = mode_index;
        SetDisplaySwitchMode(mode_index);
        break;
      }
    }
    vrefresh_ = 0;
  }

  if (bit_clk_rate_) {
    // Update current mode index if bit clk rate is changed.
    drmModeModeInfo current_mode = connector_info_.modes[current_mode_index_].mode;
    for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
      if ((current_mode.vdisplay == connector_info_.modes[mode_index].mode.vdisplay) &&
          (current_mode.hdisplay == connector_info_.modes[mode_index].mode.hdisplay) &&
          (current_mode.vrefresh == connector_info_.modes[mode_index].mode.vrefresh) &&
          (bit_clk_rate_ == connector_info_.modes[mode_index].bit_clk_rate)) {
        current_mode_index_ = mode_index;
        SetDisplaySwitchMode(mode_index);
        break;
      }
    }
    bit_clk_rate_ = 0;
  }

  if (panel_mode_changed_ & DRM_MODE_FLAG_CMD_MODE_PANEL) {
    panel_mode_changed_ = 0;
    synchronous_commit_ = false;
  } else if (panel_mode_changed_ & DRM_MODE_FLAG_VID_MODE_PANEL) {
    panel_mode_changed_ = 0;
    synchronous_commit_ = false;
    reset_output_fence_offset_ = true;
  }

  first_cycle_ = false;
  update_mode_ = false;
  hw_layers->updates_mask = 0;
  pending_power_state_ = kPowerStateNone;
  // Inherently a real commit ensures null commit properties have happened, so update the member
  first_null_cycle_ = false;
  seamless_mode_switch_ = false;

  SetTUIState();

#ifdef TRUSTED_VM
  drm_atomic_intf_->Perform(sde_drm::DRMOps::CRTC_SET_VM_REQ_STATE, token_.crtc_id,
                            sde_drm::DRMVMRequestState::NONE);
#endif

  return kErrorNone;
}

DisplayError HWDeviceDRM::Flush(HWLayers *hw_layers) {
  ClearSolidfillStages();
  ResetROI();
  bool sync_commit = (tui_state_ == kTUIStateStart || tui_state_ == kTUIStateEnd ||
                      secure_display_active_);

  // TODO(user): Handle via flush call
#ifdef TRUSTED_VM
  sync_commit = true;
#endif

  int ret = NullCommit(sync_commit /* synchronous */, false /* retain_planes*/);
  if (ret) {
    DLOGE("failed with error %d", ret);
    return kErrorHardware;
  }
  return kErrorNone;
}

void HWDeviceDRM::SetBlending(const LayerBlending &source, DRMBlendType *target) {
  switch (source) {
    case kBlendingPremultiplied:
      *target = DRMBlendType::PREMULTIPLIED;
      break;
    case kBlendingOpaque:
      *target = DRMBlendType::OPAQUE;
      break;
    case kBlendingCoverage:
      *target = DRMBlendType::COVERAGE;
      break;
    default:
      *target = DRMBlendType::UNDEFINED;
  }
}

void HWDeviceDRM::SetSrcConfig(const LayerBuffer &input_buffer, const HWRotatorMode &mode,
                               uint32_t *config) {
  // In offline rotation case, rotator will handle deinterlacing.
  if (mode == kRotatorInline) {
    if (input_buffer.flags.interlace) {
      *config |= (0x01 << UINT32(DRMSrcConfig::DEINTERLACE));
    }
  }
}

void HWDeviceDRM::SelectCscType(const LayerBuffer &input_buffer, DRMCscType *type) {
  if (type == NULL) {
    return;
  }

  *type = DRMCscType::kCscTypeMax;
  if (input_buffer.format < kFormatYCbCr420Planar) {
    return;
  }

  switch (input_buffer.color_metadata.colorPrimaries) {
    case ColorPrimaries_BT601_6_525:
    case ColorPrimaries_BT601_6_625:
      *type = ((input_buffer.color_metadata.range == Range_Full) ?
               DRMCscType::kCscYuv2Rgb601FR : DRMCscType::kCscYuv2Rgb601L);
      break;
    case ColorPrimaries_BT709_5:
      *type = ((input_buffer.color_metadata.range == Range_Full) ?
               DRMCscType::kCscYuv2Rgb709FR : DRMCscType::kCscYuv2Rgb709L);
      break;
    case ColorPrimaries_BT2020:
      *type = ((input_buffer.color_metadata.range == Range_Full) ?
                DRMCscType::kCscYuv2Rgb2020FR : DRMCscType::kCscYuv2Rgb2020L);
      break;
    default:
      break;
  }
}

void HWDeviceDRM::SetRect(const LayerRect &source, DRMRect *target) {
  target->left = UINT32(source.left);
  target->top = UINT32(source.top);
  target->right = UINT32(source.right);
  target->bottom = UINT32(source.bottom);
}

void HWDeviceDRM::SetRotation(LayerTransform transform, const HWLayerConfig &layer_config,
                              uint32_t* rot_bit_mask) {
  HWRotatorMode mode = layer_config.hw_rotator_session.mode;
  // In offline rotation case, rotator will handle flips set via offline rotator interface.
  if (mode == kRotatorOffline) {
    *rot_bit_mask = 0;
    return;
  }

  // In no rotation case or inline rotation case, plane will handle flips
  // In DRM framework rotation is applied in counter-clockwise direction.
  if (layer_config.use_inline_rot && transform.rotation == 90) {
    // a) rotate 90 clockwise = rotate 270 counter-clockwise in DRM
    // rotate 270 is translated as hflip + vflip + rotate90
    // b) rotate 270 clockwise = rotate 90 counter-clockwise in DRM
    // c) hflip + rotate 90 clockwise = vflip + rotate 90 counter-clockwise in DRM
    // d) vflip + rotate 90 clockwise = hflip + rotate 90 counter-clockwise in DRM
    *rot_bit_mask = UINT32(DRMRotation::ROT_90);
    transform.flip_horizontal = !transform.flip_horizontal;
    transform.flip_vertical = !transform.flip_vertical;
  }

  if (transform.flip_horizontal) {
    *rot_bit_mask |= UINT32(DRMRotation::FLIP_H);
  }

  if (transform.flip_vertical) {
    *rot_bit_mask |= UINT32(DRMRotation::FLIP_V);
  }
}

bool HWDeviceDRM::EnableHotPlugDetection(int enable) {
  return true;
}

DisplayError HWDeviceDRM::SetCursorPosition(HWLayers *hw_layers, int x, int y) {
  DTRACE_SCOPED();
  return kErrorNone;
}

DisplayError HWDeviceDRM::GetPPFeaturesVersion(PPFeatureVersion *vers) {
  struct DRMPPFeatureInfo info = {};

  if (!hw_color_mgr_)
    return kErrorNotSupported;

  for (uint32_t i = 0; i < kMaxNumPPFeatures; i++) {
    std::vector<DRMPPFeatureID> drm_id = {};
    memset(&info, 0, sizeof(struct DRMPPFeatureInfo));
    hw_color_mgr_->ToDrmFeatureId(kDSPP, i, &drm_id);
    if (drm_id.empty())
      continue;

    info.id = drm_id.at(0);

    drm_mgr_intf_->GetCrtcPPInfo(token_.crtc_id, &info);
    vers->version[i] = hw_color_mgr_->GetFeatureVersion(info);
  }
  return kErrorNone;
}

DisplayError HWDeviceDRM::SetPPFeatures(PPFeaturesConfig *feature_list) {
  int ret = 0;
  DRMCrtcInfo crtc_info = {};
  PPFeatureInfo *feature = NULL;

  if (!hw_color_mgr_)
    return kErrorNotSupported;

  while (true) {
    std::vector<DRMPPFeatureID> drm_id = {};
    DRMPPFeatureInfo kernel_params = {};
    bool crtc_feature = true;

    ret = feature_list->RetrieveNextFeature(&feature);
    if (ret || !feature)
      break;

    hw_color_mgr_->ToDrmFeatureId(kDSPP, feature->feature_id_, &drm_id);
    if (drm_id.empty()) {
      continue;
    } else if (drm_id.at(0) == DRMPPFeatureID::kFeatureDither) {
      drm_mgr_intf_->GetCrtcInfo(token_.crtc_id, &crtc_info);
      if (crtc_info.has_spr)
        drm_id.at(0) = DRMPPFeatureID::kFeatureSprDither;
    }

    kernel_params.id = drm_id.at(0);
    drm_mgr_intf_->GetCrtcPPInfo(token_.crtc_id, &kernel_params);
    if (kernel_params.version == std::numeric_limits<uint32_t>::max())
      crtc_feature = false;

    DLOGV_IF(kTagDriverConfig, "feature_id = %d", feature->feature_id_);
    for (DRMPPFeatureID id : drm_id) {
      if (id >= kPPFeaturesMax) {
        DLOGE("Invalid feature id %d", id);
        continue;
      }

      kernel_params.id = id;
      ret = hw_color_mgr_->GetDrmFeature(feature, &kernel_params);
#if defined(UDFPS_ZPOS) && defined(PP_DRM_ENABLE)
      // IchthysOS: the colour mode's GC is the base the UDFPS mask curve is composed on and what
      // mask-off restores. While the curve is applied, a new mode GC goes in composed.
      if (!ret && crtc_feature && id == DRMPPFeatureID::kFeaturePgc) {
        CacheFingerprintLutBase(kernel_params);
        if (udfps_lut_applied_) {
          DRMPPFeatureInfo composed = kernel_params;
          drm_msm_pgc_lut *payload = static_cast<drm_msm_pgc_lut *>(BuildFingerprintLutPgc(true));
          composed.payload = payload;
          composed.payload_size = sizeof(drm_msm_pgc_lut);
          drm_atomic_intf_->Perform(DRMOps::CRTC_SET_POST_PROC, token_.crtc_id, &composed);
          delete payload;
          udfps_lut_in_req_ = true;
          hw_color_mgr_->FreeDrmFeatureData(&kernel_params);
          continue;
        }
        udfps_lut_in_req_ = true;
      }
#endif
      if (!ret && crtc_feature)
        drm_atomic_intf_->Perform(DRMOps::CRTC_SET_POST_PROC,
                                  token_.crtc_id, &kernel_params);
      else if (!ret && !crtc_feature)
        drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_POST_PROC,
                                  token_.conn_id, &kernel_params);

      hw_color_mgr_->FreeDrmFeatureData(&kernel_params);
    }
  }

  // Once all features were consumed, then destroy all feature instance from feature_list,
  feature_list->Reset();

  return kErrorNone;
}

DisplayError HWDeviceDRM::SetVSyncState(bool enable) {
  return kErrorNotSupported;
}

void HWDeviceDRM::SetIdleTimeoutMs(uint32_t timeout_ms) {
  // TODO(user): This function can be removed after fb is deprecated
}

DisplayError HWDeviceDRM::SetDisplayMode(const HWDisplayMode hw_display_mode) {
  if (!switch_mode_valid_) {
    return kErrorNotSupported;
  }

  uint32_t mode_flag = 0;

  if (hw_display_mode == kModeCommand) {
    mode_flag = DRM_MODE_FLAG_CMD_MODE_PANEL;
    current_mode_index_ = cmd_mode_index_;
    DLOGI_IF(kTagDriverConfig, "switch panel mode to command");
  } else if (hw_display_mode == kModeVideo) {
    mode_flag = DRM_MODE_FLAG_VID_MODE_PANEL;
    current_mode_index_ = video_mode_index_;
    DLOGI_IF(kTagDriverConfig, "switch panel mode to video");
  }
  PopulateHWPanelInfo();
  panel_mode_changed_ = mode_flag;
  synchronous_commit_ = true;
  return kErrorNone;
}

DisplayError HWDeviceDRM::SetRefreshRate(uint32_t refresh_rate) {
  if (bit_clk_rate_) {
    // bit rate update pending.
    // Defer any refresh rate setting.
    return kErrorNotSupported;
  }

  // Check if requested refresh rate is valid
  drmModeModeInfo current_mode = connector_info_.modes[current_mode_index_].mode;
  uint64_t current_bit_clk = connector_info_.modes[current_mode_index_].bit_clk_rate;
  for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
    if ((current_mode.vdisplay == connector_info_.modes[mode_index].mode.vdisplay) &&
        (current_mode.hdisplay == connector_info_.modes[mode_index].mode.hdisplay) &&
        (current_bit_clk == connector_info_.modes[mode_index].bit_clk_rate) &&
        (current_mode.flags == connector_info_.modes[mode_index].mode.flags) &&
        (refresh_rate == connector_info_.modes[mode_index].mode.vrefresh)) {
      vrefresh_ = refresh_rate;
      DLOGV_IF(kTagDriverConfig, "Set refresh rate to %d", refresh_rate);
      return kErrorNone;
    }
  }
  return kErrorNotSupported;
}



DisplayError HWDeviceDRM::GetHWScanInfo(HWScanInfo *scan_info) {
  return kErrorNotSupported;
}

DisplayError HWDeviceDRM::GetVideoFormat(uint32_t config_index, uint32_t *video_format) {
  return kErrorNotSupported;
}

DisplayError HWDeviceDRM::GetMaxCEAFormat(uint32_t *max_cea_format) {
  return kErrorNotSupported;
}

DisplayError HWDeviceDRM::OnMinHdcpEncryptionLevelChange(uint32_t min_enc_level) {
  DisplayError error = kErrorNone;
  int fd = -1;
  char data[kMaxStringLength] = {'\0'};

  snprintf(data, sizeof(data), "/sys/devices/virtual/hdcp/msm_hdcp/min_level_change");

  fd = Sys::open_(data, O_WRONLY);
  if (fd < 0) {
    DLOGE("File '%s' could not be opened. errno = %d, desc = %s", data, errno, strerror(errno));
    return kErrorHardware;
  }

  snprintf(data, sizeof(data), "%d", min_enc_level);

  ssize_t err = Sys::pwrite_(fd, data, strlen(data), 0);
  if (err <= 0) {
    DLOGE("Write failed, Error = %s", strerror(errno));
    error = kErrorHardware;
  }

  Sys::close_(fd);

  return error;
}

DisplayError HWDeviceDRM::SetScaleLutConfig(HWScaleLutInfo *lut_info) {
  sde_drm::DRMScalerLUTInfo drm_lut_info = {};
  drm_lut_info.cir_lut = lut_info->cir_lut;
  drm_lut_info.dir_lut = lut_info->dir_lut;
  drm_lut_info.sep_lut = lut_info->sep_lut;
  drm_lut_info.cir_lut_size = lut_info->cir_lut_size;
  drm_lut_info.dir_lut_size = lut_info->dir_lut_size;
  drm_lut_info.sep_lut_size = lut_info->sep_lut_size;
  drm_mgr_intf_->SetScalerLUT(drm_lut_info);

  return kErrorNone;
}

DisplayError HWDeviceDRM::UnsetScaleLutConfig() {
  drm_mgr_intf_->UnsetScalerLUT();

  return kErrorNone;
}

DisplayError HWDeviceDRM::SetMixerAttributes(const HWMixerAttributes &mixer_attributes) {
  if (IsResolutionSwitchEnabled()) {
    return kErrorNotSupported;
  }

  if (!dest_scaler_blocks_used_) {
    return kErrorNotSupported;
  }

  uint32_t index = current_mode_index_;

  if (mixer_attributes.width > display_attributes_[index].x_pixels ||
      mixer_attributes.height > display_attributes_[index].y_pixels) {
    DLOGW("Input resolution exceeds display resolution! input: res %dx%d display: res %dx%d",
          mixer_attributes.width, mixer_attributes.height, display_attributes_[index].x_pixels,
          display_attributes_[index].y_pixels);
    return kErrorNotSupported;
  }

  uint32_t max_input_width = hw_resource_.hw_dest_scalar_info.max_input_width;
  uint32_t dual_max_input_width = max_input_width - DEST_SCALAR_OVERFETCH_SIZE;
  if (mixer_attributes_.split_type == kDualSplit &&
     (mixer_attributes.width > dual_max_input_width)) {
    DLOGW("Input width exceeds width limit in dual LM mode. input_width %d width_limit %d",
          mixer_attributes.width, dual_max_input_width);
    return kErrorNotSupported;
  }

  if (display_attributes_[index].is_device_split) {
    max_input_width *= 2;
  }

  if (mixer_attributes.width > max_input_width) {
    DLOGW("Input width exceeds width limit! input_width %d width_limit %d", mixer_attributes.width,
          max_input_width);
    return kErrorNotSupported;
  }

  uint32_t topology_num_split = 0;
  GetTopologySplit(display_attributes_[index].topology, &topology_num_split);
  if (mixer_attributes.width % topology_num_split != 0) {
    DLOGW("Uneven LM split: topology:%d supported_split:%d mixer_width:%d",
           display_attributes_[index].topology, topology_num_split, mixer_attributes.width);
    return kErrorNotSupported;
  }

  float mixer_aspect_ratio = FLOAT(mixer_attributes.width) / FLOAT(mixer_attributes.height);
  float display_aspect_ratio =
      FLOAT(display_attributes_[index].x_pixels) / FLOAT(display_attributes_[index].y_pixels);

  if (display_aspect_ratio != mixer_aspect_ratio) {
    DLOGW("Aspect ratio mismatch! input: res %dx%d display: res %dx%d", mixer_attributes.width,
          mixer_attributes.height, display_attributes_[index].x_pixels,
          display_attributes_[index].y_pixels);
    return kErrorNotSupported;
  }

  float scale_x = FLOAT(display_attributes_[index].x_pixels) / FLOAT(mixer_attributes.width);
  float scale_y = FLOAT(display_attributes_[index].y_pixels) / FLOAT(mixer_attributes.height);
  float max_scale_up = hw_resource_.hw_dest_scalar_info.max_scale_up;
  if (scale_x > max_scale_up || scale_y > max_scale_up) {
    DLOGW(
        "Up scaling ratio exceeds for destination scalar upscale limit scale_x %f scale_y %f "
        "max_scale_up %f",
        scale_x, scale_y, max_scale_up);
    return kErrorNotSupported;
  }

  float mixer_split_ratio = FLOAT(mixer_attributes_.split_left) / FLOAT(mixer_attributes_.width);

  mixer_attributes_ = mixer_attributes;
  mixer_attributes_.split_left = mixer_attributes_.width;
  mixer_attributes_.split_type = kNoSplit;
  mixer_attributes_.dest_scaler_blocks_used = dest_scaler_blocks_used_;  // No change.
  if (display_attributes_[index].is_device_split) {
    mixer_attributes_.split_left = UINT32(FLOAT(mixer_attributes.width) * mixer_split_ratio);
    mixer_attributes_.split_type = kDualSplit;
    if (display_attributes_[index].topology == kQuadLMMerge ||
        display_attributes_[index].topology == kQuadLMDSCMerge ||
        display_attributes_[index].topology == kQuadLMMergeDSC ||
        display_attributes_[index].topology == kQuadLMDSC4HSMerge) {
      mixer_attributes_.split_type = kQuadSplit;
    }
  }

  return kErrorNone;
}

DisplayError HWDeviceDRM::GetMixerAttributes(HWMixerAttributes *mixer_attributes) {
  if (!mixer_attributes) {
    return kErrorParameters;
  }

  *mixer_attributes = mixer_attributes_;

  return kErrorNone;
}

DisplayError HWDeviceDRM::DumpDebugData() {
  string dir_path = "/data/vendor/display/hw_recovery/";
  string device_str = device_name_;

  // Attempt to make hw_recovery dir, it may exist
  if (mkdir(dir_path.c_str(), 0777) != 0 && errno != EEXIST) {
    DLOGW("Failed to create %s directory errno = %d, desc = %s", dir_path.c_str(), errno,
          strerror(errno));
    return kErrorPermission;
  }
  // If it does exist, ensure permissions are fine
  if (errno == EEXIST && chmod(dir_path.c_str(), 0777) != 0) {
    DLOGW("Failed to change permissions on %s directory", dir_path.c_str());
    return kErrorPermission;
  }

  string filename = dir_path+device_str+"_HWR_"+to_string(debug_dump_count_);
  ofstream dst(filename);
  debug_dump_count_++;

  bool failed = false;

  {
    ifstream src;
    src.open("/sys/kernel/debug/dri/0/debug/dump");
    if (src.fail()) {
      DLOGE("Unable to open dump debugfs node");
      failed = true;
    } else {
      dst << "---- Event Logs ----" << std::endl;
      dst << src.rdbuf() << std::endl;
      if (src.fail()) {
        DLOGE("Unable to read dump debugfs node");
        failed = true;
      }
      src.close();
    }
  }

  {
    ifstream src;
    src.open("/sys/kernel/debug/dri/0/debug/recovery_reg");
    if (src.fail()) {
      DLOGE("Unable to open recovery_reg debugfs node");
      failed = true;
    } else {
      dst << "---- All Registers ----" << std::endl;
      dst << src.rdbuf() << std::endl;
      if (src.fail()) {
        failed = true;
        DLOGE("Unable to read recovery_reg debugfs node");
      }
      src.close();
    }
  }

  {
    ifstream src;
    src.open("/sys/kernel/debug/dri/0/debug/recovery_dbgbus");
    if (src.fail()) {
      DLOGE("Unable to open recovery_dbgbus debugfs node");
      failed = true;
    } else {
      dst << "---- Debug Bus ----" << std::endl;
      dst << src.rdbuf() << std::endl;
      if (src.fail()) {
        DLOGE("Unable to read recovery_dbgbus debugfs node");
        failed = true;
      }
      src.close();
    }
  }

  {
    ifstream src;
    src.open("/sys/kernel/debug/dri/0/debug/recovery_vbif_dbgbus");
    if (src.fail()) {
      DLOGE("Unable to open recovery_vbif_dbgbus debugfs node");
      failed = true;
    } else {
      dst << "---- VBIF Debug Bus ----" << std::endl;
      dst << src.rdbuf() << std::endl;
      if (src.fail()) {
        DLOGE("Unable to read recovery_vbif_dbgbus debugfs node");
        failed = true;
      }
      src.close();
    }
  }

  {
    ifstream src;
    src.open("/sys/kernel/debug/dri/0/debug/recovery_dsi_dbgbus");
    if (src.fail()) {
      DLOGE("Unable to open recovery_dsi_dbgbus debugfs node");
      failed = true;
    } else {
      dst << "---- DSI Debug Bus ----" << std::endl;
      dst << src.rdbuf() << std::endl;
      if (src.fail()) {
        DLOGE("Unable to read recovery_dsi_dbgbus debugfs node");
        failed = true;
      }
    src.close();
    }
  }

  dst.close();
  DLOGI("Wrote hw_recovery file %s", filename.c_str());

  return failed ? kErrorPermission : kErrorNone;
}

void HWDeviceDRM::GetDRMDisplayToken(sde_drm::DRMDisplayToken *token) const {
  *token = token_;
}

void HWDeviceDRM::UpdateMixerAttributes() {
  uint32_t index = current_mode_index_;

  mixer_attributes_.width = display_attributes_[index].x_pixels;
  mixer_attributes_.height = display_attributes_[index].y_pixels;
  mixer_attributes_.split_left = display_attributes_[index].is_device_split
                                     ? hw_panel_info_.split_info.left_split
                                     : mixer_attributes_.width;
  mixer_attributes_.split_type = kNoSplit;
  if (display_attributes_[index].is_device_split) {
    mixer_attributes_.split_type = kDualSplit;
    if (display_attributes_[index].topology == kQuadLMMerge ||
        display_attributes_[index].topology == kQuadLMDSCMerge ||
        display_attributes_[index].topology == kQuadLMMergeDSC ||
        display_attributes_[index].topology == kQuadLMDSC4HSMerge) {
      mixer_attributes_.split_type = kQuadSplit;
    }
  }

  DLOGI("Mixer WxH %dx%d-%d for %s", mixer_attributes_.width, mixer_attributes_.height,
        mixer_attributes_.split_type, device_name_);
  update_mode_ = true;
}

void HWDeviceDRM::SetSecureConfig(const LayerBuffer &input_buffer, DRMSecureMode *fb_secure_mode,
                                  DRMSecurityLevel *security_level) {
  *security_level = DRMSecurityLevel::SECURE_NON_SECURE;
#ifdef TRUSTED_VM
  *fb_secure_mode = DRMSecureMode::NON_SECURE_DIR_TRANSLATION;
  return;
#else
  *fb_secure_mode = DRMSecureMode::NON_SECURE;
#endif
  if (input_buffer.flags.secure) {
    if (input_buffer.flags.secure_camera) {
      // IOMMU configuration for this framebuffer mode is secure domain & requires
      // only stage II translation, when this buffer is accessed by Display H/W.
      // Secure and non-secure planes can be attached to this CRTC.
      *fb_secure_mode = DRMSecureMode::SECURE_DIR_TRANSLATION;
    } else if (input_buffer.flags.secure_display) {
      // IOMMU configuration for this framebuffer mode is secure domain & requires
      // only stage II translation, when this buffer is accessed by Display H/W.
      // Only secure planes can be attached to this CRTC.
      *fb_secure_mode = DRMSecureMode::SECURE_DIR_TRANSLATION;
      *security_level = DRMSecurityLevel::SECURE_ONLY;
    } else {
      // IOMMU configuration for this framebuffer mode is secure domain & requires both
      // stage I and stage II translations, when this buffer is accessed by Display H/W.
      // Secure and non-secure planes can be attached to this CRTC.
      *fb_secure_mode = DRMSecureMode::SECURE;
    }
  }
}

void HWDeviceDRM::SetTopology(sde_drm::DRMTopology drm_topology, HWTopology *hw_topology) {
  switch (drm_topology) {
    case DRMTopology::SINGLE_LM:            *hw_topology = kSingleLM;           break;
    case DRMTopology::SINGLE_LM_DSC:        *hw_topology = kSingleLMDSC;        break;
    case DRMTopology::DUAL_LM:              *hw_topology = kDualLM;             break;
    case DRMTopology::DUAL_LM_DSC:          *hw_topology = kDualLMDSC;          break;
    case DRMTopology::DUAL_LM_MERGE:        *hw_topology = kDualLMMerge;        break;
    case DRMTopology::DUAL_LM_MERGE_DSC:    *hw_topology = kDualLMMergeDSC;     break;
    case DRMTopology::DUAL_LM_DSCMERGE:     *hw_topology = kDualLMDSCMerge;     break;
    case DRMTopology::QUAD_LM_MERGE:        *hw_topology = kQuadLMMerge;        break;
    case DRMTopology::QUAD_LM_DSCMERGE:     *hw_topology = kQuadLMDSCMerge;     break;
    case DRMTopology::QUAD_LM_MERGE_DSC:    *hw_topology = kQuadLMMergeDSC;     break;
    case DRMTopology::QUAD_LM_DSC4HSMERGE:  *hw_topology = kQuadLMDSC4HSMerge;  break;
    case DRMTopology::PPSPLIT:              *hw_topology = kPPSplit;            break;
    default:                                *hw_topology = kUnknown;            break;
  }
}

void HWDeviceDRM::SetMultiRectMode(const uint32_t flags, DRMMultiRectMode *target) {
  *target = DRMMultiRectMode::NONE;
  if (flags & kMultiRect) {
    *target = DRMMultiRectMode::SERIAL;
    if (flags & kMultiRectParallelMode) {
      *target = DRMMultiRectMode::PARALLEL;
    }
  }
}

void HWDeviceDRM::SetSsppTonemapFeatures(HWPipeInfo *pipe_info) {
  if (pipe_info->dgm_csc_info.op != kNoOp) {
    SDECsc csc = {};
    SetDGMCsc(pipe_info->dgm_csc_info, &csc);
    DLOGV_IF(kTagDriverConfig, "Call Perform DGM CSC Op = %s",
            (pipe_info->dgm_csc_info.op == kSet) ? "Set" : "Reset");
    drm_atomic_intf_->Perform(DRMOps::PLANE_SET_DGM_CSC_CONFIG, pipe_info->pipe_id,
                              reinterpret_cast<uint64_t>(&csc.csc_v1));
  }
  if (pipe_info->inverse_pma_info.op != kNoOp) {
    DLOGV_IF(kTagDriverConfig, "Call Perform Inverse PMA Op = %s",
            (pipe_info->inverse_pma_info.op == kSet) ? "Set" : "Reset");
    drm_atomic_intf_->Perform(DRMOps::PLANE_SET_INVERSE_PMA, pipe_info->pipe_id,
                             (pipe_info->inverse_pma_info.inverse_pma) ? 1: 0);
  }
  SetSsppLutFeatures(pipe_info);
}

void HWDeviceDRM::SetDGMCsc(const HWPipeCscInfo &dgm_csc_info, SDECsc *csc) {
  SetDGMCscV1(dgm_csc_info.csc, &csc->csc_v1);
}

void HWDeviceDRM::SetDGMCscV1(const HWCsc &dgm_csc, sde_drm_csc_v1 *csc_v1) {
  uint32_t i = 0;
  for (i = 0; i < MAX_CSC_MATRIX_COEFF_SIZE; i++) {
    csc_v1->ctm_coeff[i] = dgm_csc.ctm_coeff[i];
    DLOGV_IF(kTagDriverConfig, " DGM csc_v1[%d] = %lld", i, csc_v1->ctm_coeff[i]);
  }
  for (i = 0; i < MAX_CSC_BIAS_SIZE; i++) {
    csc_v1->pre_bias[i] = dgm_csc.pre_bias[i];
    csc_v1->post_bias[i] = dgm_csc.post_bias[i];
  }
  for (i = 0; i < MAX_CSC_CLAMP_SIZE; i++) {
    csc_v1->pre_clamp[i] = dgm_csc.pre_clamp[i];
    csc_v1->post_clamp[i] = dgm_csc.post_clamp[i];
  }
}

void HWDeviceDRM::SetSsppLutFeatures(HWPipeInfo *pipe_info) {
  for (HWPipeTonemapLutInfo &lut_info : pipe_info->lut_info) {
    if (lut_info.op != kNoOp) {
      std::shared_ptr<PPFeatureInfo> feature = lut_info.pay_load;
      if (feature == nullptr) {
        DLOGE("Null Pointer for Op = %d lut type = %d", lut_info.op, lut_info.type);
        continue;
      }
      DRMPPFeatureInfo kernel_params = {};
      std::vector<DRMPPFeatureID> drm_id = {};
      PPBlock pp_block = GetPPBlock(lut_info.type);
      hw_color_mgr_->ToDrmFeatureId(pp_block, feature->feature_id_, &drm_id);
      for (DRMPPFeatureID id : drm_id) {
        if (id >= kPPFeaturesMax) {
          DLOGE("Invalid feature id %d", id);
          continue;
        }
        kernel_params.id = id;
        bool disable = (lut_info.op == kReset);
        DLOGV_IF(kTagDriverConfig, "Lut Type = %d PPBlock = %d Op = %s Disable = %d Feature = %p",
                 lut_info.type, pp_block, (lut_info.op ==kSet) ? "Set" : "Reset", disable,
                 feature.get());
        int ret = hw_color_mgr_->GetDrmFeature(feature.get(), &kernel_params, disable);
        if (!ret) {
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_POST_PROC, pipe_info->pipe_id,
                                    &kernel_params);
          hw_color_mgr_->FreeDrmFeatureData(&kernel_params);
        } else {
          DLOGE("GetDrmFeature failed for Lut type = %d", lut_info.type);
        }
      }
      drm_id.clear();
    }
  }
}

void HWDeviceDRM::AddDimLayerIfNeeded() {
  if (secure_display_active_ && hw_resource_.secure_disp_blend_stage >= 0) {
    HWSolidfillStage sf = {};
    sf.z_order = UINT32(hw_resource_.secure_disp_blend_stage);
    sf.roi = { 0.0, 0.0, FLOAT(mixer_attributes_.width), FLOAT(mixer_attributes_.height) };
    solid_fills_.clear();
    AddSolidfillStage(sf, 0xFF);
    SetSolidfillStages();
  }

  if (!secure_display_active_) {
    DRMSecurityLevel crtc_security_level = DRMSecurityLevel::SECURE_NON_SECURE;
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_SECURITY_LEVEL, token_.crtc_id, crtc_security_level);
  }
}

DisplayError HWDeviceDRM::NullCommit(bool synchronous, bool retain_planes) {
  DTRACE_SCOPED();
  AddDimLayerIfNeeded();
  int ret = drm_atomic_intf_->Commit(synchronous , retain_planes);
  if (ret) {
    DLOGE("failed with error %d", ret);
    return kErrorHardware;
  }

  if (first_null_cycle_)
    first_null_cycle_ = false;

  return kErrorNone;
}

void HWDeviceDRM::DumpConnectorModeInfo() {
  for (uint32_t i = 0; i < (uint32_t)connector_info_.modes.size(); i++) {
    DLOGI("Mode[%d] Name:%s vref:%d hdisp:%d hsync_s:%d hsync_e:%d htotal:%d " \
          "vdisp:%d vsync_s:%d vsync_e:%d vtotal:%d\n", i, connector_info_.modes[i].mode.name,
          connector_info_.modes[i].mode.vrefresh, connector_info_.modes[i].mode.hdisplay,
          connector_info_.modes[i].mode.hsync_start, connector_info_.modes[i].mode.hsync_end,
          connector_info_.modes[i].mode.htotal, connector_info_.modes[i].mode.vdisplay,
          connector_info_.modes[i].mode.vsync_start, connector_info_.modes[i].mode.vsync_end,
          connector_info_.modes[i].mode.vtotal);
  }
}

void HWDeviceDRM::ResetROI() {
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ROI, token_.crtc_id, 0, nullptr);
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_ROI, token_.conn_id, 0, nullptr);
}

bool HWDeviceDRM::IsFullFrameUpdate(const HWLayersInfo &hw_layer_info) {
  LayerRect full_frame = {0, 0, FLOAT(mixer_attributes_.width), FLOAT(mixer_attributes_.height)};

  const LayerRect &frame_roi = hw_layer_info.left_frame_roi.at(0);
  // If multiple ROIs are present, then it's not fullscreen update.
  if (hw_layer_info.left_frame_roi.size() > 1 ||
      (IsValid(frame_roi) && !IsCongruent(full_frame, frame_roi))) {
    return false;
  }

  return true;
}

DisplayError HWDeviceDRM::SetDynamicDSIClock(uint64_t bit_clk_rate) {
  return kErrorNotSupported;
}

DisplayError HWDeviceDRM::GetDynamicDSIClock(uint64_t *bit_clk_rate) {
  return kErrorNotSupported;
}

void HWDeviceDRM::DumpHWLayers(HWLayers *hw_layers) {
  HWLayersInfo &hw_layer_info = hw_layers->info;
  DestScaleInfoMap &dest_scale_info_map = hw_layer_info.dest_scale_info_map;
  LayerStack *stack = hw_layer_info.stack;
  uint32_t hw_layer_count = UINT32(hw_layer_info.hw_layers.size());
  std::vector<LayerRect> &left_frame_roi = hw_layer_info.left_frame_roi;
  std::vector<LayerRect> &right_frame_roi = hw_layer_info.right_frame_roi;
  DLOGI("HWLayers Stack: layer_count: %d, app_layer_count: %d, gpu_target_index: %d",
         hw_layer_count, hw_layer_info.app_layer_count, hw_layer_info.gpu_target_index);
  DLOGI("LayerStackFlags = 0x%" PRIu32 ",  blend_cs = {primaries = %d, transfer = %d}",
         UINT32(stack->flags.flags), UINT32(stack->blend_cs.primaries),
         UINT32(stack->blend_cs.transfer));
  for (uint32_t i = 0; i < left_frame_roi.size(); i++) {
    DLOGI("left_frame_roi: x = %d, y = %d, w = %d, h = %d", INT(left_frame_roi[i].left),
        INT(left_frame_roi[i].top), INT(left_frame_roi[i].right), INT(left_frame_roi[i].bottom));
  }
  for (uint32_t i = 0; i < right_frame_roi.size(); i++) {
  DLOGI("right_frame_roi: x = %d, y = %d, w = %d h = %d", INT(right_frame_roi[i].left),
        INT(right_frame_roi[i].top), INT(right_frame_roi[i].right),
        INT(right_frame_roi[i].bottom));
  }

  for (uint32_t i = 0; i < dest_scale_info_map.size(); i++) {
    HWDestScaleInfo *dest_scalar_data = dest_scale_info_map[i];
    if (dest_scalar_data->scale_data.enable.scale) {
      HWScaleData &scale_data = dest_scalar_data->scale_data;
      DLOGI("Dest scalar index %d Mixer WxH %dx%d", i,
            dest_scalar_data->mixer_width, dest_scalar_data->mixer_height);
      DLOGI("Panel ROI [%d, %d, %d, %d]", INT(dest_scalar_data->panel_roi.left),
           INT(dest_scalar_data->panel_roi.top), INT(dest_scalar_data->panel_roi.right),
           INT(dest_scalar_data->panel_roi.bottom));
      DLOGI("Dest scalar Dst WxH %dx%d", scale_data.dst_width, scale_data.dst_height);
    }
  }

  for (uint32_t i = 0; i < hw_layer_count; i++) {
    HWLayerConfig &hw_config = hw_layers->config[i];
    HWRotatorSession &hw_rotator_session = hw_config.hw_rotator_session;
    HWSessionConfig &hw_session_config = hw_rotator_session.hw_session_config;
    DLOGI("========================= HW_layer: %d =========================", i);
    DLOGI("src_width = %d, src_height = %d, src_format = %d, src_LayerBufferFlags = 0x%" PRIx32 ,
             hw_layer_info.hw_layers[i].input_buffer.width,
             hw_layer_info.hw_layers[i].input_buffer.height,
             hw_layer_info.hw_layers[i].input_buffer.format,
             hw_layer_info.hw_layers[i].input_buffer.flags.flags);
    if (hw_config.use_inline_rot) {
      DLOGI("rotator = %s, rotation = %d, flip_horizontal = %s, flip_vertical = %s",
            "inline rotator", INT(hw_session_config.transform.rotation),
            hw_session_config.transform.flip_horizontal ? "true" : "false",
            hw_session_config.transform.flip_vertical ? "true" : "false");
    } else if (hw_rotator_session.mode == kRotatorOffline) {
      DLOGI("rotator = %s, rotation = %d, flip_horizontal = %s, flip_vertical = %s",
            "offline rotator", INT(hw_session_config.transform.rotation),
            hw_session_config.transform.flip_horizontal ? "true" : "false",
            hw_session_config.transform.flip_vertical ? "true" : "false");
    }
    if (hw_config.use_solidfill_stage) {
      HWSolidfillStage &hw_solidfill_stage = hw_config.hw_solidfill_stage;
      LayerSolidFill &solid_fill_info = hw_solidfill_stage.solid_fill_info;
      DLOGI("HW Solid fill info: z_order = %d, color = %d", hw_solidfill_stage.z_order,
            hw_solidfill_stage.color);
      DLOGI("bit_depth = %d, red = %d, green = %d, blue = %d, alpha = %d",
            solid_fill_info.bit_depth, solid_fill_info.red, solid_fill_info.green,
            solid_fill_info.blue, solid_fill_info.alpha);
    }
    for (uint32_t count = 0; count < 2; count++) {
      HWPipeInfo &left_pipe = hw_config.left_pipe;
      HWPipeInfo &right_pipe = hw_config.right_pipe;
      HWPipeInfo &pipe_info = (count == 0) ? left_pipe : right_pipe;
      HWScaleData &scale_data = pipe_info.scale_data;
      if (!pipe_info.valid) {
        continue;
      }
      std::string pipe = (count == 0) ? "left_pipe" : "right_pipe";
      DLOGI("pipe = %s, pipe_id = %d, z_order = %d, flags = 0x%X",
           pipe.c_str(), pipe_info.pipe_id, pipe_info.z_order, pipe_info.flags);
      DLOGI("src_rect: x = %d, y = %d, w = %d, h = %d", INT(pipe_info.src_roi.left),
            INT(pipe_info.src_roi.top), INT(pipe_info.src_roi.right - pipe_info.src_roi.left),
            INT(pipe_info.src_roi.bottom - pipe_info.src_roi.top));
      DLOGI("dst_rect: x = %d, y = %d, w = %d, h = %d", INT(pipe_info.dst_roi.left),
            INT(pipe_info.dst_roi.top), INT(pipe_info.dst_roi.right - pipe_info.dst_roi.left),
            INT(pipe_info.dst_roi.bottom - pipe_info.dst_roi.top));
      DLOGI("excl_rect: left = %d, top = %d, right = %d, bottom = %d",
            INT(pipe_info.excl_rect.left), INT(pipe_info.excl_rect.top),
            INT(pipe_info.excl_rect.right), INT(pipe_info.excl_rect.bottom));
      if (scale_data.enable.scale) {
      DLOGI("HWScaleData enable flags: scale = %s, direction_detection = %s, detail_enhance = %s,"
            " dyn_exp_disable = %s", scale_data.enable.scale ? "true" : "false",
            scale_data.enable.direction_detection ? "true" : "false",
            scale_data.enable.detail_enhance ? "true" : "false",
            scale_data.enable.dyn_exp_disable ? "true" : "false");
      DLOGI("lut_flags: lut_swap = 0x%X, lut_dir_wr = 0x%X, lut_y_cir_wr = 0x%X, "
            "lut_uv_cir_wr = 0x%X, lut_y_sep_wr = 0x%X, lut_uv_sep_wr = 0x%X",
            scale_data.lut_flag.lut_swap, scale_data.lut_flag.lut_dir_wr,
            scale_data.lut_flag.lut_y_cir_wr, scale_data.lut_flag.lut_uv_cir_wr,
            scale_data.lut_flag.lut_y_sep_wr, scale_data.lut_flag.lut_uv_sep_wr);
      DLOGI("dir_lut_idx = %d, y_rgb_cir_lut_idx = %d, uv_cir_lut_idx = %d, "
            "y_rgb_sep_lut_idx = %d, uv_sep_lut_idx = %d", scale_data.dir_lut_idx,
            scale_data.y_rgb_cir_lut_idx, scale_data.uv_cir_lut_idx,
            scale_data.y_rgb_sep_lut_idx, scale_data.uv_sep_lut_idx);
      }
    }
  }
}

DisplayError HWDeviceDRM::SetBlendSpace(const PrimariesTransfer &blend_space) {
  blend_space_ = blend_space;
  return kErrorNone;
}

DisplayError HWDeviceDRM::GetDRMPowerMode(const HWPowerState &power_state,
                                          DRMPowerMode *drm_power_mode) {
  if (!drm_power_mode) {
    return kErrorParameters;
  }

  switch (power_state) {
    case kPowerStateOn:
      *drm_power_mode = DRMPowerMode::ON;
      break;
    case kPowerStateOff:
      *drm_power_mode = DRMPowerMode::OFF;
      break;
    case kPowerStateDoze:
      *drm_power_mode = DRMPowerMode::DOZE;
      break;
    case kPowerStateDozeSuspend:
      *drm_power_mode = DRMPowerMode::DOZE_SUSPEND;
      break;
    default:
      return kErrorParameters;
  }
  return kErrorNone;
}

void HWDeviceDRM::SetTUIState() {
  if (tui_state_ == kTUIStateStart) {
    tui_state_ = kTUIStateInProgress;
  } else if (tui_state_ == kTUIStateEnd) {
    tui_state_ = kTUIStateNone;
  } else if (tui_state_ == kTUIStateInProgress) {
    tui_state_ = kTUIStateNone;
  }
}

void HWDeviceDRM::GetTopologySplit(HWTopology hw_topology, uint32_t *split_number) {
  switch (hw_topology) {
    case kDualLM:
    case kDualLMDSC:
    case kDualLMMerge:
    case kDualLMMergeDSC:
    case kDualLMDSCMerge:
      *split_number = 2;
      break;
    case kQuadLMMerge:
    case kQuadLMDSCMerge:
    case kQuadLMMergeDSC:
    case kQuadLMDSC4HSMerge:
      *split_number = 4;
      break;
    case kPPSplit:
    default:
      *split_number = 1;
  }
}
}  // namespace sdm
