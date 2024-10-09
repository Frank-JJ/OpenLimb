#include <BluetoothHIDMaster.h>
#include <BluetoothLock.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/async_context.h"

#include "btstack_run_loop.h"
#include "btstack_config.h"
#include "btstack.h"
#include "classic/sdp_server.h"

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"


// #include <JoystickBT.h>
BluetoothHCI hci;
BluetoothHIDMaster hid;
// const uint8_t joystickAddress[] = {0xe8, 0x47, 0x3a, 0x21, 0x20, 0x61};

#define MAX_ATTRIBUTE_VALUE_SIZE 512

static bd_addr_t remote_addr;
static bd_addr_t connected_addr;
static btstack_packet_callback_registration_t hci_event_callback_registration;

// const uint8_t joystickAddress[] = {0xe8, 0x47, 0x3a, 0x21, 0x20, 0x61};
static const char * joystickAddress = "E8:47:3A:21:20:61";

static uint8_t hid_descriptor_storage[MAX_ATTRIBUTE_VALUE_SIZE];

static uint16_t hid_host_cid = 0;
static bool     hid_host_descriptor_available = false;
static hid_protocol_mode_t hid_host_report_mode = HID_PROTOCOL_MODE_REPORT;

static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

static void hid_host_setup(void){
	// Initialize L2CAP
	l2cap_init();

	sdp_init();

	// Initialize HID Host
	hid_host_init(hid_descriptor_storage, sizeof(hid_descriptor_storage));
	hid_host_register_packet_handler(packet_handler);

	// Allow sniff mode requests by HID device and support role switch
	gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_SNIFF_MODE | LM_LINK_POLICY_ENABLE_ROLE_SWITCH);

	// try to become master on incoming connections
	hci_set_master_slave_policy(HCI_ROLE_MASTER);

	// register for HCI events
	hci_event_callback_registration.callback = &packet_handler;
	hci_add_event_handler(&hci_event_callback_registration);
}

struct bt_hid_state {
	uint16_t buttons;
	uint8_t lx;
	uint8_t ly;
	uint8_t rx;
	uint8_t ry;
	uint8_t l2;
	uint8_t r2;
	uint8_t hat;
	uint8_t pad;
};

const struct bt_hid_state default_state = {
	.buttons = 0,
	.lx = 0x80,
	.ly = 0x80,
	.rx = 0x80,
	.ry = 0x80,
	.l2 = 0x80,
	.r2 = 0x80,
	.hat = 0x8,
};

struct bt_hid_state latest;

struct __attribute__((packed)) input_report_17 {
	uint8_t report_id;
	uint8_t pad[2];

	uint8_t lx, ly;
	uint8_t rx, ry;
	uint8_t buttons[3];
	uint8_t l2, r2;

	uint16_t timestamp;
	uint16_t temperature;
	uint16_t gyro[3];
	uint16_t accel[3];
	uint8_t pad2[5];
	uint8_t status[2];
	uint8_t pad3;
};

static void hid_host_handle_interrupt_report(const uint8_t *packet, uint16_t packet_len){
	static struct bt_hid_state last_state = { 0 };

  // Only interested in report_id 0x11
	if (packet_len < sizeof(struct input_report_17) + 1) {
		return;
	}

  if ((packet[0] != 0xa1) || (packet[1] != 0x11)) {
		return;
	}

  // printf_hexdump(packet, packet_len);

  struct input_report_17 *report = (struct input_report_17 *)&packet[1];

  // Note: This assumes that we're protected by async_context's
	// single-threaded-ness
	latest = (struct bt_hid_state){
		// Somewhat arbitrary packing of the buttons into a single 16-bit word
		.buttons = ((report->buttons[0] & 0xf0) << 8) | ((report->buttons[2] & 0x3) << 8) | (report->buttons[1]),

		.lx = report->lx,
		.ly = report->ly,
		.rx = report->rx,
		.ry = report->ry,
		.l2 = report->l2,
		.r2 = report->r2,

		.hat = (report->buttons[0] & 0xf),
	};

  // TODO: Parse out battery, touchpad, sixaxis, timestamp, temperature(?!)
	// Sensors will also need calibration
}

void bt_hid_get_latest(struct bt_hid_state *dst)
{
	async_context_t *context = cyw43_arch_async_context();
	async_context_acquire_lock_blocking(context);
	memcpy(dst, &latest, sizeof(*dst));
	async_context_release_lock(context);
}

static void bt_hid_disconnected(bd_addr_t addr)
{
	hid_host_cid = 0;
	hid_host_descriptor_available = false;

	memcpy(&latest, &default_state, sizeof(latest));
}


static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
	UNUSED(channel);
	UNUSED(size);

	uint8_t   event;
	uint8_t   hid_event;
	bd_addr_t event_addr;
	uint8_t   status;
	uint8_t reason;

	if (packet_type != HCI_EVENT_PACKET) {
		return;
	}

	event = hci_event_packet_get_type(packet);
  hid_protocol_mode_t proto;
	switch (event) {
    case BTSTACK_EVENT_STATE:
      // On boot, we try a manual connection
      if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING){
        printf("Starting hid_host_connect (%s)\n", bd_addr_to_str(remote_addr));
        status = hid_host_connect(remote_addr, hid_host_report_mode, &hid_host_cid);
        if (status != ERROR_CODE_SUCCESS){
          printf("hid_host_connect command failed: 0x%02x\n", status);
        }
      }
      break;
    case HCI_EVENT_CONNECTION_COMPLETE:
      status = hci_event_connection_complete_get_status(packet);
      printf("Connection complete: %x\n", status);
      break;
    case HCI_EVENT_DISCONNECTION_COMPLETE:
      status = hci_event_disconnection_complete_get_status(packet);
      reason = hci_event_disconnection_complete_get_reason(packet);
      printf("Disconnection complete: status: %x, reason: %x\n", status, reason);
      break;
    case HCI_EVENT_MAX_SLOTS_CHANGED:
      status = hci_event_max_slots_changed_get_lmp_max_slots(packet);
      printf("Max slots changed: %x\n", status);
      break;
    case HCI_EVENT_PIN_CODE_REQUEST:
      printf("Pin code request. Responding '0000'\n");
      hci_event_pin_code_request_get_bd_addr(packet, event_addr);
      gap_pin_code_response(event_addr, "0000");
      break;
    case HCI_EVENT_USER_CONFIRMATION_REQUEST:
      printf("SSP User Confirmation Request: %d\n", little_endian_read_32(packet, 8));
      break;
    case HCI_EVENT_HID_META:
      hid_event = hci_event_hid_meta_get_subevent_code(packet);
      switch (hid_event) {
        case HID_SUBEVENT_INCOMING_CONNECTION:
          hid_subevent_incoming_connection_get_address(packet, event_addr);
          printf("Accepting connection from %s\n", bd_addr_to_str(event_addr));
          hid_host_accept_connection(hid_subevent_incoming_connection_get_hid_cid(packet), hid_host_report_mode);
          break;
        case HID_SUBEVENT_CONNECTION_OPENED:
          status = hid_subevent_connection_opened_get_status(packet);
          hid_subevent_connection_opened_get_bd_addr(packet, event_addr);
          if (status != ERROR_CODE_SUCCESS) {
            printf("Connection to %s failed: 0x%02x\n", bd_addr_to_str(event_addr), status);
            bt_hid_disconnected(event_addr);
            return;
          }
          hid_host_descriptor_available = false;
          hid_host_cid = hid_subevent_connection_opened_get_hid_cid(packet);
          printf("Connected to %s\n", bd_addr_to_str(event_addr));
          bd_addr_copy(connected_addr, event_addr);
          break;
        case HID_SUBEVENT_DESCRIPTOR_AVAILABLE:
          status = hid_subevent_descriptor_available_get_status(packet);
          if (status == ERROR_CODE_SUCCESS){
            hid_host_descriptor_available = true;

            uint16_t dlen = hid_descriptor_storage_get_descriptor_len(hid_host_cid);
            printf("HID descriptor available. Len: %d\n", dlen);

            // Send FEATURE 0x05, to switch the controller to "full" report mode
            hid_host_send_get_report(hid_host_cid, HID_REPORT_TYPE_FEATURE, 0x05);
          } else {
            printf("Couldn't process HID Descriptor, status: %d\n", status);
          }
          break;
        case HID_SUBEVENT_REPORT:
          if (hid_host_descriptor_available){
            hid_host_handle_interrupt_report(hid_subevent_report_get_report(packet), hid_subevent_report_get_report_len(packet));
          } else {
            printf("No hid host descriptor available\n");
            printf_hexdump(hid_subevent_report_get_report(packet), hid_subevent_report_get_report_len(packet));
          }
          break;
        case HID_SUBEVENT_SET_PROTOCOL_RESPONSE:
          status = hid_subevent_set_protocol_response_get_handshake_status(packet);
          if (status != HID_HANDSHAKE_PARAM_TYPE_SUCCESSFUL){
            printf("Protocol handshake error: 0x%02x\n", status);
            break;
          }
          proto = static_cast<hid_protocol_mode_t>(hid_subevent_set_protocol_response_get_protocol_mode(packet));
          switch (proto) {
          case HID_PROTOCOL_MODE_BOOT:
            printf("Negotiated protocol: BOOT\n");
            break;
          case HID_PROTOCOL_MODE_REPORT:
            printf("Negotiated protocol: REPORT\n");
            break;
          default:
            printf("Negotiated unknown protocol: 0x%x\n", proto);
            break;
          }
          break;
        case HID_SUBEVENT_CONNECTION_CLOSED:
          printf("HID connection closed: %s\n", bd_addr_to_str(connected_addr));
          bt_hid_disconnected(connected_addr);
          break;
        case HID_SUBEVENT_GET_REPORT_RESPONSE:
          {
            status = hid_subevent_get_report_response_get_handshake_status(packet);
            uint16_t dlen =  hid_subevent_get_report_response_get_report_len(packet);
            printf("GET_REPORT response. status: %d, len: %d\n", status, dlen);
          }
          break;
        default:
          printf("Unknown HID subevent: 0x%x\n", hid_event);
          break;
      }
      break;
    default:
      //printf("Unknown HCI event: 0x%x\n", event);
      break;
	}
}

#define BLINK_MS 250
static btstack_timer_source_t blink_timer;
static void blink_handler(btstack_timer_source_t *ts)
{
	static bool on = 0;

	if (hid_host_cid != 0) {
		on = true;
	} else {
		on = !on;
	}

	cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, !!on);

	btstack_run_loop_set_timer(&blink_timer, BLINK_MS);
	btstack_run_loop_add_timer(&blink_timer);
}

void bt_main(void) {
	if (cyw43_arch_init()) {
		printf("Wi-Fi init failed\n");
		return;
	}

	gap_set_security_level(LEVEL_2);

	blink_timer.process = &blink_handler;
	btstack_run_loop_set_timer(&blink_timer, BLINK_MS);
	btstack_run_loop_add_timer(&blink_timer);

	hid_host_setup();
	sscanf_bd_addr(joystickAddress, remote_addr);
	bt_hid_disconnected(remote_addr);

	hci_power_control(HCI_POWER_ON);

	btstack_run_loop_execute();
}


// int main(void) {
// 	// stdio_init_all();

// 	sleep_ms(1000);
// 	printf("Hello\n");

// 	multicore_launch_core1(bt_main);
// 	// Wait for init (should do a handshake with the fifo here?)
// 	sleep_ms(1000);

// 	// struct chassis chassis = { 0 };
// 	// chassis_init(&chassis, 6, 8);

// 	struct bt_hid_state state;
// 	for ( ;; ) {
// 		sleep_ms(20);
// 		bt_hid_get_latest(&state);
// 		printf("buttons: %04x, l: %d,%d, r: %d,%d, l2,r2: %d,%d hat: %d\n",
// 				state.buttons, state.lx, state.ly, state.rx, state.ry,
// 				state.l2, state.r2, state.hat);

// 		// float speed_scale = 1.0;
// 		// int8_t linear = clamp8(-(state.ly - 128) * speed_scale);
// 		// int8_t rot = clamp8(-(state.rx - 128));
// 		// chassis_set(&chassis, linear, rot);
// 	}
//   return 0;
// }


struct bt_hid_state state;
void setup(){
  // stdio_init_all();

	sleep_ms(1000);
	printf("Hello\n");

	multicore_launch_core1(bt_main);
	// Wait for init (should do a handshake with the fifo here?)
	sleep_ms(1000);

	// struct chassis chassis = { 0 };
	// chassis_init(&chassis, 6, 8);

}

void loop(){
  sleep_ms(20);
  bt_hid_get_latest(&state);
  printf("buttons: %04x, l: %d,%d, r: %d,%d, l2,r2: %d,%d hat: %d\n",
      state.buttons, state.lx, state.ly, state.rx, state.ry,
      state.l2, state.r2, state.hat);

  // float speed_scale = 1.0;
  // int8_t linear = clamp8(-(state.ly - 128) * speed_scale);
  // int8_t rot = clamp8(-(state.rx - 128));
  // chassis_set(&chassis, linear, rot);
}

// enum ourCrazyStates{starting_bluetooth, waiting_for_bluetooth, scanning_bluetooth, connecting_joystich, wait_for_connection, running_program};

// // Joystick can get reports of 4 analog axes, 1 d-pad bitfield, and up to 32 buttons
// // Axes and hats that aren't reported by the joystick are read as 0
// void joy(void *cbdata, int x, int y, int z, int rz, uint8_t hat, uint32_t buttons) {
//   Serial.printf("Hello there joy!");

//   (void) cbdata;
//   const char *hats[16] = { "U", "UR", "R", "DR", "D", "DL", "L", "UL", "", "", "", "", "", "", "", "." };
//   Serial.printf("Joystick: (%4d, %4d) (%4d, %4d), Hat: %-2s, Buttons:", x, y, z, rz, hats[hat & 15]);
//   for (int i = 0; i < 32; i++) {
//     Serial.printf(" %c", (buttons & 1 << i) ? '*' : '.');
//   }
//   Serial.println();
// }

// void BTBasicSetup() {
//   l2cap_init();
//   sm_init();
//   // hid_host_init(hid_descriptor_storage, sizeof(hid_descriptor_storage))
//   gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_SNIFF_MODE | LM_LINK_POLICY_ENABLE_ROLE_SWITCH);
//   hci_set_master_slave_policy(HCI_ROLE_MASTER);
//   hci_power_control(HCI_POWER_ON);
//   // hci_set_inquiry_mode(INQUIRY_MODE_RSSI_AND_EIR);

//   hci.install();
//   hci.begin();

// }

// void setup() {
//   // put your setup code here, to run once:
//   // 00002508 | e8:47:3a:21:20:61 |  -47 | DualSense Wireless 
  
//   Serial.begin();
//   delay(5000);
//   Serial.printf("\n\nNew session started!.\n");
//   // BTBasicSetup();

//   Serial.printf("Starting HID master, put your device in pairing mode now.\n");

//   hid.onJoystick(joy);

// }

// int loop_iterations = 0;

// // bool bluetoothStarting = false;
// // bool bluetoothStarted = false;
// // void startBlutooth(){
// //   if (bluetoothStarting == false){
// //     Serial.printf("hid beginning\n");
// //     hid.begin();
// //     bluetoothStarting = true;
// //     Serial.printf("Waiting for bluetooth to run!\n");
// //   }
// //   else if (bluetoothStarted == false){
// //     if (hid.running()){
// //       bluetoothStarted = true;
// //     }
// //   }
// // }

// // bool scanningDone = false;
// // void scan(){
// //   Serial.printf("BEGIN SCAN @%lu ...", millis());

// //   // auto l = hid.scan(BluetoothHIDMaster::joystick_cod);
// //   auto l = hid.scan(BluetoothHIDMaster::any_cod);
// //   // Serial.printf("hid running: %i\n", hid.running());

// //   Serial.printf("END SCAN @%lu\n\n", millis());
// //   Serial.printf("%-8s | %-17s | %-4s | %s\n", "Class", "Address", "RSSI", "Name");
// //   Serial.printf("%-8s | %-17s | %-4s | %s\n", "--------", "-----------------", "----", "----------------");
// //   for (auto e : l) {
// //     Serial.printf("%08lx | %17s | %4d | %s\n", e.deviceClass(), e.addressString(), e.rssi(), e.name());
// //   }
// //   Serial.printf("\n\n\n");
  
// //   scanningDone = true;
// // }
  
// // bool connectedSuccessfully = false;
// // void connectToJoystick(){
// //   bool connectionResult = hid.connectJoystick();
// //   // bool connectionSuccessfull = hid.connectAny();
// //   Serial.printf("connection successull: %i\n", connectionResult);
// //   if (connectionResult){
// //     connectedSuccessfully = true;
// //   }
// // }


// // ourCrazyStates current_state = ourCrazyStates::starting_bluetooth;
// int wait_iterations = 0;
// void loop() {
//   loop_iterations++;
//   // switch (current_state) {
//   //   case ourCrazyStates::starting_bluetooth: {
//   //     Serial.printf("hid beginning\n");
//   //     hid.begin();
//   //     current_state = ourCrazyStates::scanning_bluetooth;
//   //     delay(5000);
//   //     break;
//   //   }
//   //   case ourCrazyStates::scanning_bluetooth: {
//   //     Serial.printf("BEGIN SCAN @%lu ...", millis());

//   //     // auto l = hid.scan(BluetoothHIDMaster::joystick_cod);
//   //     auto l = hid.scan(BluetoothHIDMaster::any_cod);
//   //     // Serial.printf("hid running: %i\n", hid.running());

//   //     Serial.printf("END SCAN @%lu\n\n", millis());
//   //     Serial.printf("%-8s | %-17s | %-4s | %s\n", "Class", "Address", "RSSI", "Name");
//   //     Serial.printf("%-8s | %-17s | %-4s | %s\n", "--------", "-----------------", "----", "----------------");
//   //     for (auto e : l) {
//   //       Serial.printf("%08lx | %17s | %4d | %s\n", e.deviceClass(), e.addressString(), e.rssi(), e.name());
//   //     }
//   //     Serial.printf("\n\n\n");
//   //     current_state = ourCrazyStates::connecting_joystich;
//   //     break;
//   //   }
//   //   case ourCrazyStates::connecting_joystich: {
//   //     bool connectionResult = hid.connectJoystick();
//   //     Serial.printf("connection successull: %i\n", connectionResult);
//   //     Serial.printf("waiting for connection to stabilize\n", connectionResult);
//   //     current_state = ourCrazyStates::wait_for_connection;
//   //     delay(5000);
//   //     break;
//   //   }
//   //   // case ourCrazyStates::waiting_for_bluetooth: {
//   //   //   if (hid.running()){
//   //   //     Serial.printf("hid running\n");
//   //   //     current_state = ourCrazyStates::connecting_joystich;
//   //   //   }
//   //   //   else{
//   //   //     Serial.printf("hid failed to start, retrying\n");
//   //   //     // current_state = ourCrazyStates::starting_bluetooth;
//   //   //     delay(5000);
//   //   //   }
//   //   //   break;
//   //   // }
//   //   case ourCrazyStates::wait_for_connection: {
//   //     if (hid.connected())
//   //     {
//   //       Serial.printf("hid connected: yes\n");
//   //       current_state = ourCrazyStates::running_program;
//   //     }
//   //     else{
//   //       Serial.printf("hid failed to connect, removing pairing and retrying!\n");
//   //       current_state = ourCrazyStates::starting_bluetooth;
//   //     }
//   //     break;
//   //   }
//   //   default: {
//   //     break;
//   //   }
//   // }

//   // if (bluetoothStarted == false){
//   //   startBlutooth();
//   // }
//   // else if(scanningDone == false){
//   //   scan();
//   // }
//   // else if(connectedSuccessfully == false){
//   //   connectToJoystick();
//   // }

//   // if (hid.connected()){
//   //   Serial.printf("hid connected: yes %i\n", loop_iterations);
//   // }
//   // if (hid.running()){
//   //   Serial.printf("hid running: yes %i\n", loop_iterations);
//   // }
//   // Serial.printf("hid running: %i | connected: %i\n", hid.running(), hid.connected());
//   // put your main code here, to run repeatedly:
//   if (BOOTSEL) {
//     while (BOOTSEL) {
//       delay(1);
//     }
//     hid.disconnect();
//     hid.clearPairing();
//     hid.onJoystick(joy);
//     Serial.printf("Restarting HID master, put your device in pairing mode now.\n");
//     hid.connectAny();
//     // Serial.printf("Waiting for connection\n");
//     // while (hid.connected() == false)
//     // {
//     //   delay(1);
//     // }
//     Serial.printf("Connected!\n");
//   }
// }
