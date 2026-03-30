/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once
#include <rs_driver/driver/decoder/decoder_mech.hpp>

namespace robosense
{
namespace lidar
{

#pragma pack(push, 1)
typedef struct
{
  uint8_t id[4];
  uint8_t reserved_0[4];
  uint8_t pkt_cnt_top2bot[4];
  uint8_t pkt_cnt_bot2top[4];
  uint8_t data_type[2];
  uint8_t reserved_1[2];
  RSTimestampUTC timestamp;
  uint8_t reserved_2;
  uint8_t lidar_type;
  uint8_t lidar_mode;
  uint8_t reserved_3[5];
  RSTemperature temp;
  RSTemperature topboard_temp;
} RSAIRYHeader;

typedef struct
{
  uint16_t distance;
  uint8_t intensity;
} RSAIRYChannel;

typedef struct
{
  uint8_t id[2];
  uint16_t azimuth;
  RSAIRYChannel channels[48];
} RSAIRYMsopBlock;

typedef struct
{
  RSAIRYHeader header;
  RSAIRYMsopBlock blocks[8];
  uint8_t tail[6];
  uint8_t reserved_4[16];
} RSAIRYMsopPkt;

typedef struct
{
  uint16_t vol_main;
  uint16_t vol_12v;
  uint16_t vol_mcu;
  uint16_t vol_1v;
  uint16_t current_main;
  uint8_t reserved[16];
} RSAIRYStatus;

typedef struct
{
  uint8_t id[8];
  uint16_t rpm;
  RSEthNetV2 eth;
  RSFOV fov;
  uint8_t reserved_1[4];
  RSVersionV1 version;
  uint8_t reserved_2[239];
  uint8_t install_mode;
  uint8_t reserved_3[2];
  RSSN sn;
  uint16_t zero_cal;
  uint8_t return_mode;
  uint8_t reserved_4[167];
  RSCalibrationAngle vert_angle_cali[96];
  RSCalibrationAngle horiz_angle_cali[96];
  uint8_t reserved_5[22];
  RSAIRYStatus status;
  uint32_t qx;
  uint32_t qy;
  uint32_t qz;
  uint32_t qw;
  uint32_t x;
  uint32_t y;
  uint32_t z;
  uint8_t ptp_err[4];
  uint32_t pitch;
  uint32_t yaw;
  uint32_t roll;
  uint8_t reserved_6[110];
  uint16_t tail;
} RSAIRYDifopPkt;
#pragma pack(pop)

enum RSAIRYLidarModel
{
  RSAIRY_CHANNEL_48 = 0,
  RSAIRY_CHANNEL_96 = 1,
  RSAIRY_CHANNEL_192 = 2,
};

template <typename T_PointCloud>
class DecoderRSAIRY : public DecoderMech<T_PointCloud>
{
public:
  virtual void decodeDifopPkt(const uint8_t* pkt, size_t size);
  virtual bool decodeMsopPkt(const uint8_t* pkt, size_t size);
  virtual ~DecoderRSAIRY() = default;

  explicit DecoderRSAIRY(const RSDecoderParam& param);

#ifndef UNIT_TEST
protected:
#endif

  static RSDecoderMechConstParam& getConstParam();
  static RSEchoMode getEchoMode(uint8_t mode);
  static RSAIRYLidarModel getLidarModel(uint8_t mode, uint16_t& channel_num);

  template <typename T_BlockIterator>
  bool internDecodeMsopPkt(const uint8_t* pkt, size_t size);

  RSAIRYLidarModel lidarModel_{ RSAIRY_CHANNEL_96 };
  uint16_t u16ChannelNum_{ 96 };
  bool bInit_{ false };
  bool loaded_install_info_{ false };
  uint16_t install_mode_{ 0xFF };
};

template <typename T_PointCloud>
inline RSDecoderMechConstParam& DecoderRSAIRY<T_PointCloud>::getConstParam()
{
  static RSDecoderMechConstParam param =
  {
    1248 // msop len
      , 1248 // difop len
      , 4 // msop id len
      , 8 // difop id len
      , {0x55, 0xAA, 0x05, 0x5A} // msop id
    , {0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55} // difop id
    , {0xFF, 0xEE} // block id
    , 96 // laser number
    , 8 // blocks per packet
      , 48 // channels per block
      , 0.1f // distance min
      , 60.0f // distance max
      , 0.005f // distance resolution
      , 0.0625f // temperature resolution

      // lens center
      , 0.0075f // RX
      , 0.00664f // RY
      , 0.04532f // RZ
  };

  INIT_ONLY_ONCE();

  float blk_ts = 111.080f;

  float firing_tss_airy_m[] =
  {
    0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f,
    7.616f, 7.616f, 7.616f, 7.616f, 7.616f, 7.616f, 7.616f, 7.616f,
    16.184f, 16.184f, 16.184f, 16.184f, 16.184f, 16.184f, 16.184f, 16.184f,
    24.752f, 24.752f, 24.752f, 24.752f, 24.752f, 24.752f, 24.752f, 24.752f,
    33.320f, 33.320f, 33.320f, 33.320f, 33.320f, 33.320f, 33.320f, 33.320f,
    42.840f, 42.840f, 42.840f, 42.840f, 42.840f, 42.840f, 42.840f, 42.840f,
    52.360f, 52.360f, 52.360f, 52.360f, 52.360f, 52.360f, 52.360f, 52.360f,
    61.880f, 61.880f, 61.880f, 61.880f, 61.880f, 61.880f, 61.880f, 61.880f,
    71.400f, 71.400f, 71.400f, 71.400f, 71.400f, 71.400f, 71.400f, 71.400f,
    79.968f, 79.968f, 79.968f, 79.968f, 79.968f, 79.968f, 79.968f, 79.968f,
    88.536f, 88.536f, 88.536f, 88.536f, 88.536f, 88.536f, 88.536f, 88.536f,
    97.104f, 97.104f, 97.104f, 97.104f, 97.104f, 97.104f, 97.104f, 97.104f,
  };

  param.BLOCK_DURATION = blk_ts / 1000000;
  for (uint16_t i = 0; i < sizeof(firing_tss_airy_m) / sizeof(firing_tss_airy_m[0]); i++)
  {
    param.CHAN_TSS[i] = (double)firing_tss_airy_m[i] / 1000000;
    param.CHAN_AZIS[i] = firing_tss_airy_m[i] / blk_ts;
  }

  return param;
}

template <typename T_PointCloud>
inline RSEchoMode DecoderRSAIRY<T_PointCloud>::getEchoMode(uint8_t mode)
{
  switch (mode)
  {
    case 0x03:
      return RSEchoMode::ECHO_DUAL;
    default:
      return RSEchoMode::ECHO_SINGLE;
  }
}

template <typename T_PointCloud>
inline RSAIRYLidarModel DecoderRSAIRY<T_PointCloud>::getLidarModel(uint8_t mode, uint16_t& channel_num)
{
  switch (mode)
  {
    case 0x01:
      channel_num = 48;
      return RSAIRY_CHANNEL_48;
    case 0x03:
      channel_num = 192;
      return RSAIRY_CHANNEL_192;
    case 0x02:
    default:
      channel_num = 96;
      return RSAIRY_CHANNEL_96;
  }
}

template <typename T_PointCloud>
inline DecoderRSAIRY<T_PointCloud>::DecoderRSAIRY(const RSDecoderParam& param)
  : DecoderMech<T_PointCloud>(getConstParam(), param)
{
}

template <typename T_PointCloud>
inline void DecoderRSAIRY<T_PointCloud>::decodeDifopPkt(const uint8_t* packet, size_t size)
{
  const RSAIRYDifopPkt& pkt = *(const RSAIRYDifopPkt*)(packet);
  this->template decodeDifopCommon<RSAIRYDifopPkt>(pkt);

  if ((uint16_t)(pkt.install_mode) != 0xFF)
  {
    this->install_mode_ = (uint16_t)(pkt.install_mode);
    this->loaded_install_info_ = true;
  }
}

template <typename T_PointCloud>
inline bool DecoderRSAIRY<T_PointCloud>::decodeMsopPkt(const uint8_t* pkt, size_t size)
{
  const RSAIRYMsopPkt& msopPkt = *(const RSAIRYMsopPkt*)(pkt);

  if (msopPkt.header.data_type[0] != 0)
  {
    return false;
  }

  if (!bInit_)
  {
    this->echo_mode_ = getEchoMode(msopPkt.header.data_type[1]);
    this->split_blks_per_frame_ = (this->echo_mode_ == RSEchoMode::ECHO_DUAL) ? (this->blks_per_frame_ << 1) : this->blks_per_frame_;

    lidarModel_ = getLidarModel(msopPkt.header.lidar_mode, u16ChannelNum_);

    float blk_ts = 111.080f;
    this->mech_const_param_.BLOCK_DURATION = blk_ts / 1000000;

    float firing_tss_48L[] =
    {
      0.00f, 0.00f, 0.00f, 0.00f, 7.616f, 7.616f, 7.616f, 7.616f,
      16.184f, 16.184f, 16.184f, 16.184f, 24.752f, 24.752f, 24.752f, 24.752f,
      33.320f, 33.320f, 33.320f, 33.320f, 42.840f, 42.840f, 42.840f, 42.840f,
      52.360f, 52.360f, 52.360f, 52.360f, 61.880f, 61.880f, 61.880f, 61.880f,
      71.400f, 71.400f, 71.400f, 71.400f, 79.968f, 79.968f, 79.968f, 79.968f,
      88.536f, 88.536f, 88.536f, 88.536f, 97.104f, 97.104f, 97.104f, 97.104f,
    };

    float firing_tss_side[] =
    {
      0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f,
      11.424f, 11.424f, 11.424f, 11.424f, 11.424f, 11.424f, 11.424f, 11.424f,
      22.848f, 22.848f, 22.848f, 22.848f, 22.848f, 22.848f, 22.848f, 22.848f,
      34.272f, 34.272f, 34.272f, 34.272f, 34.272f, 34.272f, 34.272f, 34.272f,
      45.696f, 45.696f, 45.696f, 45.696f, 45.696f, 45.696f, 45.696f, 45.696f,
      54.264f, 54.264f, 54.264f, 54.264f, 54.264f, 54.264f, 54.264f, 54.264f,
      62.832f, 62.832f, 62.832f, 62.832f, 62.832f, 62.832f, 62.832f, 62.832f,
      71.400f, 71.400f, 71.400f, 71.400f, 71.400f, 71.400f, 71.400f, 71.400f,
      79.016f, 79.016f, 79.016f, 79.016f, 79.016f, 79.016f, 79.016f, 79.016f,
      85.680f, 85.680f, 85.680f, 85.680f, 85.680f, 85.680f, 85.680f, 85.680f,
      92.344f, 92.344f, 92.344f, 92.344f, 92.344f, 92.344f, 92.344f, 92.344f,
      99.008f, 99.008f, 99.008f, 99.008f, 99.008f, 99.008f, 99.008f, 99.008f,
    };

    float firing_tss_normal[] =
    {
      0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f,
      5.712f, 5.712f, 5.712f, 5.712f, 5.712f, 5.712f, 5.712f, 5.712f,
      12.376f, 12.376f, 12.376f, 12.376f, 12.376f, 12.376f, 12.376f, 12.376f,
      19.040f, 19.040f, 19.040f, 19.040f, 19.040f, 19.040f, 19.040f, 19.040f,
      25.704f, 25.704f, 25.704f, 25.704f, 25.704f, 25.704f, 25.704f, 25.704f,
      33.320f, 33.320f, 33.320f, 33.320f, 33.320f, 33.320f, 33.320f, 33.320f,
      41.888f, 41.888f, 41.888f, 41.888f, 41.888f, 41.888f, 41.888f, 41.888f,
      50.456f, 50.456f, 50.456f, 50.456f, 50.456f, 50.456f, 50.456f, 50.456f,
      59.024f, 59.024f, 59.024f, 59.024f, 59.024f, 59.024f, 59.024f, 59.024f,
      70.448f, 70.448f, 70.448f, 70.448f, 70.448f, 70.448f, 70.448f, 70.448f,
      81.872f, 81.872f, 81.872f, 81.872f, 81.872f, 81.872f, 81.872f, 81.872f,
      93.296f, 93.296f, 93.296f, 93.296f, 93.296f, 93.296f, 93.296f, 93.296f
    };

    if (lidarModel_ == RSAIRYLidarModel::RSAIRY_CHANNEL_48)
    {
      for (uint16_t i = 0; i < sizeof(firing_tss_48L) / sizeof(firing_tss_48L[0]); i++)
      {
        this->mech_const_param_.CHAN_TSS[i] = (double)firing_tss_48L[i] / 1000000;
        this->mech_const_param_.CHAN_AZIS[i] = firing_tss_48L[i] / blk_ts;
      }
      bInit_ = true;
    }
    else
    {
      const float* firing = firing_tss_normal;
      if (loaded_install_info_ && (install_mode_ == 0x1))
      {
        firing = firing_tss_side;
      }

      for (uint16_t i = 0; i < sizeof(firing_tss_normal) / sizeof(firing_tss_normal[0]); i++)
      {
        this->mech_const_param_.CHAN_TSS[i] = (double)firing[i] / 1000000;
        this->mech_const_param_.CHAN_AZIS[i] = firing[i] / blk_ts;
      }
      bInit_ = true;
    }
  }

  if (lidarModel_ == RSAIRYLidarModel::RSAIRY_CHANNEL_48)
  {
    if (this->echo_mode_ == RSEchoMode::ECHO_SINGLE)
    {
      return internDecodeMsopPkt<SingleReturnBlockIterator<RSAIRYMsopPkt>>(pkt, size);
    }
    else
    {
      return internDecodeMsopPkt<DualReturnBlockIterator<RSAIRYMsopPkt>>(pkt, size);
    }
  }
  else if (lidarModel_ == RSAIRYLidarModel::RSAIRY_CHANNEL_96)
  {
    if (this->echo_mode_ == RSEchoMode::ECHO_SINGLE)
    {
      return internDecodeMsopPkt<TwoInOneBlockIterator<RSAIRYMsopPkt>>(pkt, size);
    }
    else
    {
      return internDecodeMsopPkt<FourInOneBlockIterator<RSAIRYMsopPkt>>(pkt, size);
    }
  }
  else
  {
    RS_ERROR << "Unsupported lidar model: " << (int)lidarModel_ << RS_REND;
    return false;
  }
}

template <typename T_PointCloud>
template <typename T_BlockIterator>
inline bool DecoderRSAIRY<T_PointCloud>::internDecodeMsopPkt(const uint8_t* packet, size_t size)
{
  const RSAIRYMsopPkt& pkt = *(const RSAIRYMsopPkt*)(packet);
  bool ret = false;

  this->temperature_ = parseTempInLe(&(pkt.header.temp)) * this->const_param_.TEMPERATURE_RES;

  double pkt_ts = 0;
  if (this->param_.use_lidar_clock)
  {
    pkt_ts = parseTimeUTCWithUs((RSTimestampUTC*)&pkt.header.timestamp) * 1e-6;
  }
  else
  {
    uint64_t ts = getTimeHost();
    pkt_ts = ts * 1e-6 - this->getPacketDuration();

    if (this->write_pkt_ts_)
    {
      createTimeUTCWithUs(ts, (RSTimestampUTC*)&pkt.header.timestamp);
    }
  }

  T_BlockIterator iter(pkt, this->const_param_.BLOCKS_PER_PKT, this->mech_const_param_.BLOCK_DURATION, this->block_az_diff_,
                       this->fov_blind_ts_diff_);

  static uint8_t blk_check_mode = 0;
  static uint8_t blk_id0 = 0;
  static uint8_t blk_id1 = 0;
  static bool blk_id_reported = false;

  if (blk_check_mode == 0)
  {
    blk_id0 = pkt.blocks[0].id[0];
    blk_id1 = pkt.blocks[0].id[1];

    bool all0_same = true;
    bool all01_same = true;
    for (uint16_t i = 0; i < this->const_param_.BLOCKS_PER_PKT; i++)
    {
      if (pkt.blocks[i].id[0] != blk_id0)
      {
        all0_same = false;
        all01_same = false;
        break;
      }
      if (pkt.blocks[i].id[1] != blk_id1)
      {
        all01_same = false;
      }
    }

    blk_check_mode = all01_same ? 2 : (all0_same ? 1 : 3);
  }

  for (uint16_t blk = 0; blk < this->const_param_.BLOCKS_PER_PKT; blk++)
  {
    const RSAIRYMsopBlock& block = pkt.blocks[blk];
    bool block_id_ok = true;
    if (blk_check_mode == 2)
    {
      block_id_ok = (block.id[0] == blk_id0 && block.id[1] == blk_id1);
    }
    else if (blk_check_mode == 1)
    {
      block_id_ok = (block.id[0] == blk_id0);
    }

    if (!block_id_ok)
    {
      if (!blk_id_reported)
      {
        blk_id_reported = true;
        this->cb_excep_(Error(ERRCODE_WRONGMSOPBLKID));
      }
      break;
    }

    int32_t block_az_diff;
    double block_ts_off;
    iter.get(blk, block_az_diff, block_ts_off);

    double block_ts = pkt_ts + block_ts_off;
    int32_t block_az = ntohs(block.azimuth);

    if (this->split_strategy_->newBlock(block_az))
    {
      this->cb_split_frame_(this->u16ChannelNum_, this->cloudTs());
      this->first_point_ts_ = block_ts;
      ret = true;
    }

    for (uint16_t chan = 0; chan < this->const_param_.CHANNELS_PER_BLOCK; chan++)
    {
      const RSAIRYChannel& channel = block.channels[chan];
      uint16_t chan_id = chan;
      if (lidarModel_ == RSAIRYLidarModel::RSAIRY_CHANNEL_96 && (blk % 2) == 1)
      {
        chan_id = chan + 48;
      }

      double chan_ts = block_ts + this->mech_const_param_.CHAN_TSS[chan_id];
      int32_t angle_horiz = block_az + (int32_t)((float)block_az_diff * this->mech_const_param_.CHAN_AZIS[chan_id]);

      int32_t angle_vert = this->chan_angles_.vertAdjust(chan_id);
      int32_t angle_horiz_final = this->chan_angles_.horizAdjust(chan_id, angle_horiz);

      uint16_t u16RawDistance = ntohs(channel.distance);
      uint16_t u16Distance = u16RawDistance & 0x3FFF;
      uint8_t feature = (u16RawDistance >> 14) & 0x03;
      float distance = u16Distance * this->const_param_.DISTANCE_RES;

      if (this->distance_section_.in(distance) && this->scan_section_.in(angle_horiz_final))
      {
        float x = distance * COS(angle_vert) * COS(angle_horiz_final) + this->lidar_lens_center_Rxy_ * COS(angle_horiz);
        float y = -distance * COS(angle_vert) * SIN(angle_horiz_final) - this->lidar_lens_center_Rxy_ * SIN(angle_horiz);
        float z = distance * SIN(angle_vert) + this->mech_const_param_.RZ;

        this->transformPoint(x, y, z);

        typename T_PointCloud::PointT point;
        setX(point, x);
        setY(point, y);
        setZ(point, z);
        setIntensity(point, channel.intensity);
        setRing(point, this->chan_angles_.toUserChan(chan_id));
        setTimestamp(point, chan_ts);
        setFeature(point, feature);
        this->point_cloud_->points.emplace_back(point);
      }
      else if (!this->param_.dense_points)
      {
        typename T_PointCloud::PointT point;
        setX(point, NAN);
        setY(point, NAN);
        setZ(point, NAN);
        setIntensity(point, 0);
        setRing(point, this->chan_angles_.toUserChan(chan_id));
        setTimestamp(point, chan_ts);
        setFeature(point, feature);
        this->point_cloud_->points.emplace_back(point);
      }

      this->prev_point_ts_ = chan_ts;
    }
  }

  this->prev_pkt_ts_ = pkt_ts;
  return ret;
}

}  // namespace lidar
}  // namespace robosense

