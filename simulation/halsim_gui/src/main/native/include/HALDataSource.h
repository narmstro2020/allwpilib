// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <glass/DataSource.h>

#define HALSIMGUI_DATASOURCE(cbname, id, TYPE, Type, vtype)                   \
  class cbname##Source : public ::glass::Type##Source {                       \
   public:                                                                    \
    cbname##Source()                                                          \
        : Type##Source{id},                                                   \
          m_callback{                                                         \
              HALSIM_Register##cbname##Callback(CallbackFunc, this, true)} {} \
                                                                              \
    ~cbname##Source() {                                                       \
      if (m_callback != 0)                                                    \
        HALSIM_Cancel##cbname##Callback(m_callback);                          \
    }                                                                         \
                                                                              \
   private:                                                                   \
    static void CallbackFunc(const char*, void* param,                        \
                             const HAL_Value* value) {                        \
      if (value->type == HAL_##TYPE)                                          \
        static_cast<cbname##Source*>(param)->SetValue(value->data.v_##vtype); \
    }                                                                         \
                                                                              \
    int32_t m_callback;                                                       \
  }

#define HALSIMGUI_DATASOURCE_BOOLEAN(cbname, id) \
  HALSIMGUI_DATASOURCE(cbname, id, BOOLEAN, Boolean, boolean)

#define HALSIMGUI_DATASOURCE_DOUBLE(cbname, id) \
  HALSIMGUI_DATASOURCE(cbname, id, DOUBLE, Double, double)

#define HALSIMGUI_DATASOURCE_INT(cbname, id) \
  HALSIMGUI_DATASOURCE(cbname, id, INT, Integer, int)

#define HALSIMGUI_DATASOURCE_INDEXED(cbname, id, TYPE, Type, vtype)           \
  class cbname##Source : public ::glass::Type##Source {                       \
   public:                                                                    \
    explicit cbname##Source(int32_t index, int channel = -1)                  \
        : Type##Source{::glass::MakeSourceId(id,                              \
                                             channel < 0 ? index : channel)}, \
          m_index{index},                                                     \
          m_channel{channel < 0 ? index : channel},                           \
          m_callback{HALSIM_Register##cbname##Callback(index, CallbackFunc,   \
                                                       this, true)} {}        \
                                                                              \
    ~cbname##Source() {                                                       \
      if (m_callback != 0)                                                    \
        HALSIM_Cancel##cbname##Callback(m_index, m_callback);                 \
    }                                                                         \
                                                                              \
    int32_t GetIndex() const {                                                \
      return m_index;                                                         \
    }                                                                         \
                                                                              \
    int GetChannel() const {                                                  \
      return m_channel;                                                       \
    }                                                                         \
                                                                              \
   private:                                                                   \
    static void CallbackFunc(const char*, void* param,                        \
                             const HAL_Value* value) {                        \
      if (value->type == HAL_##TYPE)                                          \
        static_cast<cbname##Source*>(param)->SetValue(value->data.v_##vtype); \
    }                                                                         \
                                                                              \
    int32_t m_index;                                                          \
    int m_channel;                                                            \
    int32_t m_callback;                                                       \
  }

#define HALSIMGUI_DATASOURCE_BOOLEAN_INDEXED(cbname, id) \
  HALSIMGUI_DATASOURCE_INDEXED(cbname, id, BOOLEAN, Boolean, boolean)

#define HALSIMGUI_DATASOURCE_DOUBLE_INDEXED(cbname, id) \
  HALSIMGUI_DATASOURCE_INDEXED(cbname, id, DOUBLE, Double, double)

#define HALSIMGUI_DATASOURCE_INDEXED2(cbname, id, TYPE, Type, vtype)          \
  class cbname##Source : public ::glass::Type##Source {                       \
   public:                                                                    \
    explicit cbname##Source(int32_t index, int32_t channel)                   \
        : Type##Source{::glass::MakeSourceId(id, index, channel)},            \
          m_index{index},                                                     \
          m_channel{channel},                                                 \
          m_callback{HALSIM_Register##cbname##Callback(                       \
              index, channel, CallbackFunc, this, true)} {}                   \
                                                                              \
    ~cbname##Source() {                                                       \
      if (m_callback != 0)                                                    \
        HALSIM_Cancel##cbname##Callback(m_index, m_channel, m_callback);      \
    }                                                                         \
                                                                              \
    int32_t GetIndex() const {                                                \
      return m_index;                                                         \
    }                                                                         \
                                                                              \
    int32_t GetChannel() const {                                              \
      return m_channel;                                                       \
    }                                                                         \
                                                                              \
   private:                                                                   \
    static void CallbackFunc(const char*, void* param,                        \
                             const HAL_Value* value) {                        \
      if (value->type == HAL_##TYPE)                                          \
        static_cast<cbname##Source*>(param)->SetValue(value->data.v_##vtype); \
    }                                                                         \
                                                                              \
    int32_t m_index;                                                          \
    int32_t m_channel;                                                        \
    int32_t m_callback;                                                       \
  }

#define HALSIMGUI_DATASOURCE_BOOLEAN_INDEXED2(cbname, id) \
  HALSIMGUI_DATASOURCE_INDEXED2(cbname, id, BOOLEAN, Boolean, boolean)

#define HALSIMGUI_DATASOURCE_DOUBLE_INDEXED2(cbname, id) \
  HALSIMGUI_DATASOURCE_INDEXED2(cbname, id, DOUBLE, Double, double)
