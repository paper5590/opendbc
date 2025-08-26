#pragma once

#include "opendbc/safety/safety_declarations.h"

// Volvo CMA platform CAN message addresses
#define VOLVO_LCA_STEER           88U    // TX from VCU1 to PSCM, LCA steering command (0x58)
#define VOLVO_BCM2_SPEED          103U   // RX from BCM, vehicle speed
#define VOLVO_BCM2                105U   // RX from BCM, brake pedal, cruise state
#define VOLVO_SAS                 85U    // RX from SAS, steering angle sensor
#define VOLVO_PSCM                22U    // RX from PSCM, driver steering input
#define VOLVO_GEAR_POSITION       128U   // RX from transmission, gear position
#define VOLVO_ECM_1               0x250

// CAN bus definitions for Volvo CMA platform
#define VOLVO_VCU1_BUS    0U  // VCU1 bus (where LCA originates)
#define VOLVO_PT_BUS      1U  // Front 1 CAN bus (where ECM is)
#define VOLVO_PSCM_BUS    2U  // PSCM bus (BCM2, SAS, EGSM, where LCA is sent to)

static void volvo_rx_hook(const CANPacket_t *msg) {
  // Basic vehicle state monitoring - very relaxed implementation

  // VCU1 bus (bus 0) messages
  if (msg->bus == VOLVO_VCU1_BUS) {
    // Gear position comes from VCU1 bus
    if (msg->addr == VOLVO_GEAR_POSITION) {
      // Signal: GEAR_POSITION (0: Park, 1: Reverse, 2: Neutral, 3: Drive)
      // This is used by carstate.py for gear shifter state
    }
  }

  if (msg->bus == VOLVO_PT_BUS) {
    if (msg->addr == VOLVO_ECM_1) {
      // Gas pedal position
      int gas_pedal_position = msg->data[3];
      gas_pressed = gas_pedal_position > 20+2; // 20 baseline + 2 tolerance
    }
  }

  // PSCM bus (bus 2) messages - BCM2, SAS, PSCM, EGSM
  if (msg->bus == VOLVO_PSCM_BUS) {
    // Update vehicle speed from BCM2_SPEED
    if (msg->addr == VOLVO_BCM2_SPEED) {
      // Signal: SPEED (0.01 m/s per bit)
      uint16_t speed_raw = ((msg->data[4] & 0x1F) << 7) | ((msg->data[5] & 0xFE) >> 1);      vehicle_moving = speed_raw > 10; // > 0.1 m/s
      UPDATE_VEHICLE_SPEED(speed_raw * 0.01);
    }

    // Update brake pedal state from BCM2
    if (msg->addr == VOLVO_BCM2) {
      // Signals: BRAKE_PEDAL_PRESSED_A, BRAKE_PEDAL_PRESSED_B
      bool brake_a = (msg->data[5] >> 7) & 1U; // Active low
      bool brake_b = (msg->data[5] >> 6) & 1U;
      brake_pressed = !brake_a || brake_b;

      // Signal: CRUISE_OR_PILOT_ASSIST_ENGAGED (also on PSCM bus)
      bool cruise_engaged = (msg->data[1] >> 4) & 1U;
      controls_allowed = cruise_engaged;
    }

    // Update steering angle from SAS
    if (msg->addr == VOLVO_SAS) {
      // Signal: SAS_ANGLE_SENSOR (-0.05596 deg per bit)
      int angle_raw = ((msg->data[0] & 0x7FU) << 8) | msg->data[1];
      if (msg->data[0] & 0x80U) {
        angle_raw = -angle_raw;
      }
      update_sample(&angle_meas, angle_raw);
    }

    // Update driver steering input from PSCM
    if (msg->addr == VOLVO_PSCM) {
      // Signal: DRIVER_INPUT_DEVIATION
      int driver_input = msg->data[5];
      update_sample(&torque_driver, driver_input);
    }
  }
}

static bool volvo_tx_hook(const CANPacket_t *msg) {
  bool tx = true;

  // Very relaxed safety policy - only basic frame ID checks
  if (msg->addr == VOLVO_LCA_STEER) {
    // LCA message flows: VCU1 (bus 0) -> PSCM (bus 2)
    // We're acting as VCU1, so we send LCA message to PSCM bus (bus 2)
    if (msg->bus != VOLVO_PSCM_BUS) {
      tx = false;  // Wrong bus
    }

    // Basic length check
    if (GET_LEN(msg) != 8U) {
      tx = false;  // Wrong message length
    }

    // Only allow when controls are enabled
    if (!controls_allowed) {
      tx = false;
    }
  }

  return tx;
}

static safety_config volvo_init(uint16_t param) {
  UNUSED(param);

  // Define allowed TX messages - very permissive
  static const CanMsg VOLVO_TX_MSGS[] = {
    {VOLVO_LCA_STEER, VOLVO_PSCM_BUS, 8, .check_relay = true},  // LCA steering command to PSCM bus
  };

  // Define RX checks - minimal monitoring for basic safety
  static RxCheck volvo_rx_checks[] = {
    // Gear position - from VCU1 bus (bus 0)
    {.msg = {{VOLVO_GEAR_POSITION, VOLVO_VCU1_BUS, 8, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},

    // Vehicle speed - required for basic safety (on PSCM bus)
    {.msg = {{VOLVO_BCM2_SPEED, VOLVO_PSCM_BUS, 8, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},

    // Brake pedal and cruise state - required for safety (on PSCM bus)
    {.msg = {{VOLVO_BCM2, VOLVO_PSCM_BUS, 8, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},

    // Steering angle - required for lateral control (on PSCM bus)
    {.msg = {{VOLVO_SAS, VOLVO_PSCM_BUS, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},

    // Driver steering input - required for override detection (on PSCM bus)
    {.msg = {{VOLVO_PSCM, VOLVO_PSCM_BUS, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},

    // Gas pedal position - required for safety (on PT bus)
    {.msg = {{VOLVO_ECM_1, VOLVO_PT_BUS, 8, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  return BUILD_SAFETY_CFG(volvo_rx_checks, VOLVO_TX_MSGS);
}

const safety_hooks volvo_hooks = {
  .init = volvo_init,
  .rx = volvo_rx_hook,
  .tx = volvo_tx_hook,
};