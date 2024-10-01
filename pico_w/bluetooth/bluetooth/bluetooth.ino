#include <BluetoothHIDMaster.h>

// #include <JoystickBT.h>

BluetoothHIDMaster hid;
const uint8_t joystickAddress[] = {0xe8, 0x47, 0x3a, 0x21, 0x20, 0x61};

// Joystick can get reports of 4 analog axes, 1 d-pad bitfield, and up to 32 buttons
// Axes and hats that aren't reported by the joystick are read as 0
void joy(void *cbdata, int x, int y, int z, int rz, uint8_t hat, uint32_t buttons) {
  (void) cbdata;
  const char *hats[16] = { "U", "UR", "R", "DR", "D", "DL", "L", "UL", "", "", "", "", "", "", "", "." };
  Serial.printf("Joystick: (%4d, %4d) (%4d, %4d), Hat: %-2s, Buttons:", x, y, z, rz, hats[hat & 15]);
  for (int i = 0; i < 32; i++) {
    Serial.printf(" %c", (buttons & 1 << i) ? '*' : '.');
  }
  Serial.println();
}

void BTBasicSetup() {
  l2cap_init();
  sm_init();
  gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_SNIFF_MODE | LM_LINK_POLICY_ENABLE_ROLE_SWITCH);
  hci_set_master_slave_policy(HCI_ROLE_MASTER);
  hci_set_inquiry_mode(INQUIRY_MODE_RSSI_AND_EIR);

}

void setup() {
  // put your setup code here, to run once:
  // 00002508 | e8:47:3a:21:20:61 |  -47 | DualSense Wireless 
  
  Serial.begin();
  delay(5000);
  Serial.printf("\n\nNew session started!.\n");
  // BTBasicSetup();

  Serial.printf("Starting HID master, put your device in pairing mode now.\n");

  hid.onJoystick(joy);

  Serial.printf("hid running: %i\n", hid.running());
  Serial.printf("hid beginning");
  hid.begin();
  Serial.printf("hid running: %i\n", hid.running());

  hid.clearPairing();
  Serial.printf("hid running: %i\n", hid.running());

  Serial.printf("BEGIN SCAN @%lu ...", millis());

  // auto l = hid.scan(BluetoothHIDMaster::joystick_cod);
  auto l = hid.scan(BluetoothHIDMaster::any_cod);
  Serial.printf("hid running: %i\n", hid.running());

  Serial.printf("END SCAN @%lu\n\n", millis());
  Serial.printf("%-8s | %-17s | %-4s | %s\n", "Class", "Address", "RSSI", "Name");
  Serial.printf("%-8s | %-17s | %-4s | %s\n", "--------", "-----------------", "----", "----------------");
  for (auto e : l) {
    Serial.printf("%08lx | %17s | %4d | %s\n", e.deviceClass(), e.addressString(), e.rssi(), e.name());
  }
  Serial.printf("\n\n\n");

  Serial.printf("hci running: %i\n", hid.hciRunning());
  Serial.printf("hid running: %i\n", hid.running());

  bool connectionSuccessfull = hid.connectJoystick();
  // bool connectionSuccessfull = hid.connectAny();
  Serial.printf("connection successull: %i\n", connectionSuccessfull);
  
  Serial.printf("hid running: %i\n", hid.running());

  // Serial.printf("Waiting for connection\n");
  // while (hid.connected() == false)
  // {
  //   delay(1);
  // }
  Serial.printf("connected: %i\n", hid.connected());
  Serial.printf("hid running: %i\n", hid.running());
}

void loop() {
  // put your main code here, to run repeatedly:
  if (BOOTSEL) {
    while (BOOTSEL) {
      delay(1);
    }
    hid.disconnect();
    hid.clearPairing();
    Serial.printf("Restarting HID master, put your device in pairing mode now.\n");
    hid.connectAny();
    // Serial.printf("Waiting for connection\n");
    // while (hid.connected() == false)
    // {
    //   delay(1);
    // }
    Serial.printf("Connected!\n");
  }
}
