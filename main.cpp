#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#define PN532_PREAMBLE (0x00)
#define PN532_STARTCODE1 (0x00)
#define PN532_STARTCODE2 (0xFF)
#define PN532_POSTAMBLE (0x00)

#define PN532_HOSTTOPN532 (0xD4)
#define PN532_PN532TOHOST (0xD5)

#define PN532_ACK_WAIT_TIME (10) // ms, timeout of waiting for ACK

#define PN532_INVALID_ACK (-1)
#define PN532_TIMEOUT (-2)
#define PN532_INVALID_FRAME (-3)
#define PN532_NO_SPACE (-4)

void wakeup(int serial) {
  uint8_t msg[] = {0x55, 0x55, 0x00, 0x00, 0x00};
  write(serial, msg, sizeof(msg));
}

int8_t receive(int serial, uint8_t *buf, int len) {
  int read_bytes = 0;
  while (read_bytes < len) {
    int res = read(serial, buf + read_bytes, len - read_bytes);
    if (res == -1) {
      printf("Error %i from receive: %s\n", errno, strerror(errno));
      exit(-1);
    } else if (res == 0) {
      return PN532_TIMEOUT;
    } else {
      read_bytes += res;
    }
  }
  return read_bytes;
}

int8_t readAckFrame(int serial) {
  const uint8_t PN532_ACK[] = {0, 0, 0xFF, 0, 0xFF, 0};
  uint8_t ackBuf[sizeof(PN532_ACK)];

  if (receive(serial, ackBuf, sizeof(PN532_ACK)) <= 0) {
    printf("ACK Timeout\n");
    return PN532_TIMEOUT;
  }

  if (memcmp(ackBuf, PN532_ACK, sizeof(PN532_ACK))) {
    printf("Invalid ACK\n");
    return PN532_INVALID_ACK;
  }
  return 0;
}

static int command;

int8_t writeCommand(int serial, const uint8_t *header, uint8_t hlen, const uint8_t *body = 0, uint8_t blen = 0) {
  command = header[0];
  uint8_t length = hlen + blen + 1;
  uint8_t startMsg[] = {
      PN532_PREAMBLE,
      PN532_STARTCODE1,
      PN532_STARTCODE2,
      length,
      static_cast<uint8_t>(~length + 1), // checksum of length
      PN532_HOSTTOPN532,
  };
  write(serial, startMsg, sizeof(startMsg));
  write(serial, header, hlen);
  write(serial, body, blen);

  uint8_t sum = PN532_HOSTTOPN532; // sum of TFI + DATA
  for (uint8_t i = 0; i < hlen; i++) {
    sum += header[i];
  }
  for (uint8_t i = 0; i < blen; i++) {
    sum += body[i];
  }
  uint8_t checksum = ~sum + 1;
  uint8_t endMsg[] = {checksum, PN532_POSTAMBLE};
  write(serial, endMsg, sizeof(endMsg));

  return readAckFrame(serial);
}

int16_t readResponse(int serial, uint8_t buf[], uint8_t len) {
  uint8_t tmp[3];

  /** Frame Preamble and Start Code */
  if (receive(serial, tmp, 3) <= 0) {
    return PN532_TIMEOUT;
  }
  if (0 != tmp[0] || 0 != tmp[1] || 0xFF != tmp[2]) {
    printf("Preamble error\n");
    return PN532_INVALID_FRAME;
  }

  /** receive length and check */
  uint8_t length[2];
  if (receive(serial, length, 2) <= 0) {
    return PN532_TIMEOUT;
  }
  if (0 != (uint8_t)(length[0] + length[1])) {
    printf("Length error\n");
    return PN532_INVALID_FRAME;
  }
  length[0] -= 2;
  if (length[0] > len) {
    return PN532_NO_SPACE;
  }

  /** receive command byte */
  uint8_t cmd = command + 1; // response command
  if (receive(serial, tmp, 2) <= 0) {
    return PN532_TIMEOUT;
  }
  if (PN532_PN532TOHOST != tmp[0] || cmd != tmp[1]) {
    printf("Command error\n");
    return PN532_INVALID_FRAME;
  }

  if (receive(serial, buf, length[0]) != length[0]) {
    return PN532_TIMEOUT;
  }
  uint8_t sum = PN532_PN532TOHOST + cmd;
  for (uint8_t i = 0; i < length[0]; i++) {
    sum += buf[i];
  }

  /** checksum and postamble */
  if (receive(serial, tmp, 2) <= 0) {
    return PN532_TIMEOUT;
  }
  if (0 != (uint8_t)(sum + tmp[0]) || 0 != tmp[1]) {
    printf("Checksum error\n");
    return PN532_INVALID_FRAME;
  }

  return length[0];
}

// PN532 Commands
#define PN532_COMMAND_DIAGNOSE (0x00)
#define PN532_COMMAND_GETFIRMWAREVERSION (0x02)
#define PN532_COMMAND_GETGENERALSTATUS (0x04)
#define PN532_COMMAND_READREGISTER (0x06)
#define PN532_COMMAND_WRITEREGISTER (0x08)
#define PN532_COMMAND_READGPIO (0x0C)
#define PN532_COMMAND_WRITEGPIO (0x0E)
#define PN532_COMMAND_SETSERIALBAUDRATE (0x10)
#define PN532_COMMAND_SETPARAMETERS (0x12)
#define PN532_COMMAND_SAMCONFIGURATION (0x14)
#define PN532_COMMAND_POWERDOWN (0x16)
#define PN532_COMMAND_RFCONFIGURATION (0x32)
#define PN532_COMMAND_RFREGULATIONTEST (0x58)
#define PN532_COMMAND_INJUMPFORDEP (0x56)
#define PN532_COMMAND_INJUMPFORPSL (0x46)
#define PN532_COMMAND_INLISTPASSIVETARGET (0x4A)
#define PN532_COMMAND_INATR (0x50)
#define PN532_COMMAND_INPSL (0x4E)
#define PN532_COMMAND_INDATAEXCHANGE (0x40)
#define PN532_COMMAND_INCOMMUNICATETHRU (0x42)
#define PN532_COMMAND_INDESELECT (0x44)
#define PN532_COMMAND_INRELEASE (0x52)
#define PN532_COMMAND_INSELECT (0x54)
#define PN532_COMMAND_INAUTOPOLL (0x60)
#define PN532_COMMAND_TGINITASTARGET (0x8C)
#define PN532_COMMAND_TGSETGENERALBYTES (0x92)
#define PN532_COMMAND_TGGETDATA (0x86)
#define PN532_COMMAND_TGSETDATA (0x8E)
#define PN532_COMMAND_TGSETMETADATA (0x94)
#define PN532_COMMAND_TGGETINITIATORCOMMAND (0x88)
#define PN532_COMMAND_TGRESPONSETOINITIATOR (0x90)
#define PN532_COMMAND_TGGETTARGETSTATUS (0x8A)

#define PN532_RESPONSE_INDATAEXCHANGE (0x41)
#define PN532_RESPONSE_INLISTPASSIVETARGET (0x4B)

// FeliCa Commands
#define FELICA_CMD_POLLING (0x00)
#define FELICA_CMD_REQUEST_SERVICE (0x02)
#define FELICA_CMD_REQUEST_RESPONSE (0x04)
#define FELICA_CMD_READ_WITHOUT_ENCRYPTION (0x06)
#define FELICA_CMD_WRITE_WITHOUT_ENCRYPTION (0x08)
#define FELICA_CMD_REQUEST_SYSTEM_CODE (0x0C)

static uint8_t pn532_packetbuffer[64];

void PrintHex(const uint8_t *data, const uint32_t numBytes) {
  for (uint8_t i = 0; i < numBytes; i++) {
    printf("%02X", data[i]);
  }
  printf("\n");
}

uint32_t getFirmwareVersion(int serial) {
  uint32_t response;

  pn532_packetbuffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;

  if (writeCommand(serial, pn532_packetbuffer, 1)) {
    return 0;
  }

  // read data packet
  int16_t status = readResponse(serial, pn532_packetbuffer, sizeof(pn532_packetbuffer));
  if (0 > status) {
    return 0;
  }

  response = pn532_packetbuffer[0];
  response <<= 8;
  response |= pn532_packetbuffer[1];
  response <<= 8;
  response |= pn532_packetbuffer[2];
  response <<= 8;
  response |= pn532_packetbuffer[3];

  return response;
}

bool setRFField(int serial, uint8_t autoRFCA, uint8_t rFOnOff) {
  pn532_packetbuffer[0] = PN532_COMMAND_RFCONFIGURATION;
  pn532_packetbuffer[1] = 1;
  pn532_packetbuffer[2] = 0x00 | autoRFCA | rFOnOff;

  if (writeCommand(serial, pn532_packetbuffer, 3)) {
    return 0x0; // command failed
  }

  return (0 < readResponse(serial, pn532_packetbuffer, sizeof(pn532_packetbuffer)));
}

int8_t felica_Polling(int serial, uint16_t systemCode, uint8_t requestCode, uint8_t *idm, uint8_t *pmm,
                      uint16_t *systemCodeResponse) {
  pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
  pn532_packetbuffer[1] = 1;
  pn532_packetbuffer[2] = 1;
  pn532_packetbuffer[3] = FELICA_CMD_POLLING;
  pn532_packetbuffer[4] = (systemCode >> 8) & 0xFF;
  pn532_packetbuffer[5] = systemCode & 0xFF;
  pn532_packetbuffer[6] = requestCode;
  pn532_packetbuffer[7] = 0;

  if (writeCommand(serial, pn532_packetbuffer, 8)) {
    printf("Could not send Polling command\n");
    return -1;
  }

  int16_t status = readResponse(serial, pn532_packetbuffer, 22);
  if (status < 0) {
    printf("Could not receive response\n");
    return -2;
  }

  // Check NbTg (pn532_packetbuffer[7])
  if (pn532_packetbuffer[0] == 0) {
    printf("No card had detected\n");
    return 0;
  } else if (pn532_packetbuffer[0] != 1) {
    printf("Unhandled number of targets inlisted. NbTg: ");
    // DMSG_HEX(pn532_packetbuffer[7]);
    printf("\n");
    return -3;
  }

  uint8_t inListedTag = pn532_packetbuffer[1];
  printf("Tag number: ");
  PrintHex(&pn532_packetbuffer[1], 1);
  printf("\n");

  // length check
  uint8_t responseLength = pn532_packetbuffer[2];
  if (responseLength != 18 && responseLength != 20) {
    printf("Wrong response length\n");
    return -4;
  }

  uint8_t i;
  for (i = 0; i < 8; ++i) {
    idm[i] = pn532_packetbuffer[4 + i];
    pmm[i] = pn532_packetbuffer[12 + i];
  }

  if (responseLength == 20) {
    *systemCodeResponse = (uint16_t)((pn532_packetbuffer[20] << 8) + pn532_packetbuffer[21]);
  }

  return 1;
}

bool SAMConfig(int serial) {
  pn532_packetbuffer[0] = PN532_COMMAND_SAMCONFIGURATION;
  pn532_packetbuffer[1] = 0x01; // normal mode;
  pn532_packetbuffer[2] = 0x14; // timeout 50ms * 20 = 1 second
  pn532_packetbuffer[3] = 0x01; // use IRQ pin!

  if (writeCommand(serial, pn532_packetbuffer, 4))
    return false;

  return (0 < readResponse(serial, pn532_packetbuffer, sizeof(pn532_packetbuffer)));
}

int main() {
  int serial = open("/dev/ttyUSB0", O_RDWR);

  if (serial < 0) {
    printf("Error %i from open: %s\n", errno, strerror(errno));
    exit(-1);
  }

  struct termios tty;

  if (tcgetattr(serial, &tty) != 0) {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }

  tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in
                                 // communication (most common)
  tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
  tty.c_cflag |= CS8;            // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;                   // Disable echo
  tty.c_lflag &= ~ECHOE;                  // Disable erasure
  tty.c_lflag &= ~ECHONL;                 // Disable new-line echo
  tty.c_lflag &= ~ISIG;                   // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &=
      ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g.
                         // newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

  tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as
                        // soon as any data is received.
  tty.c_cc[VMIN] = 0;

  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  if (tcsetattr(serial, TCSANOW, &tty) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }

  wakeup(serial);
  SAMConfig(serial);

  uint32_t firmwareVersion = getFirmwareVersion(serial);
  printf("Firmware version: 0x%8x\n", firmwareVersion);

  setRFField(serial, false, false);

  uint8_t idm[8];
  uint8_t pmm[8];
  uint16_t systemCodeResponse;
  // felica_Polling(serial, 0x88B4 /* FeliCa Lite-S */, 0x01, idm,
  //                pmm, &systemCodeResponse);
  while (felica_Polling(serial, 0xFFFF /* All FeliCa */, 0x01 /*System Code Request*/, idm, pmm, &systemCodeResponse) <=
         0) {
    printf("Sending new request.\n");
  }
  printf("IDM: ");
  PrintHex(idm, sizeof(idm));
  printf("PMM: ");
  PrintHex(pmm, sizeof(idm));
  printf("System Code: %04X\n", systemCodeResponse);

  printf("Done.\n");

  close(serial);
}
